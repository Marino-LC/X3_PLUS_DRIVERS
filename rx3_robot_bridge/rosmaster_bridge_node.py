#!/usr/bin/env python3
"""
rosmaster_bridge_node.py
========================
Puente ROS 2 para RosBoardDrv (ruedas + brazo + odometría por encoders).

- Suscribe a /cmd_vel (Twist) para mover la base.
- Suscribe a ARM_TOPIC/GRIP_TOPIC (JointTrajectory) para mover el brazo.
- Convierte radianes del URDF a grados de servo con calibración ajustable.
- NUEVO: publica /odom_encoder (nav_msgs/Odometry) integrando los conteos
  absolutos de FUNC_REPORT_ENCODER vía MecanumEncoderOdometry (ver
  encoder_odometry_math.py), para comparar contra /odom (rf2o_laser_odometry)
  y decidir cuál fuente usar para evaluar los controladores AG/ZN.

═══════════════════════════════════════════════════════════════════════════
DECISIÓN DE DISEÑO: la odometría por encoders vive EN ESTE NODO, no en uno
separado.
═══════════════════════════════════════════════════════════════════════════
El puerto serie (/dev/ttyCH341USB0) solo admite un lector/escritor seguro
de forma simultánea. Este nodo ya es el único dueño de la instancia
RosBoardDrv (self._bot), y ya escribe sobre ella desde _cmd_vel_cb y los
callbacks de trayectoria del brazo. Un segundo proceso/nodo abriendo una
segunda conexión al mismo puerto para leer encoders competiría por el
recurso y podría corromper tramas de cualquiera de los dos lados (lectura
y escritura entrelazadas sobre el mismo buffer serie).

Por eso la integración odométrica (clase pura MecanumEncoderOdometry, sin
dependencias de ROS/serie) se instancia AQUÍ, y el poll de encoders ocurre
en el mismo hilo/temporizador que ya maneja este nodo, reutilizando la
MISMA conexión self._bot.

PRERREQUISITO — concurrencia de create_receive_threading():
  get_motor_encoder() depende de que el hilo de recepción interno de
  RosBoardDrv esté corriendo (FUNC_REPORT_ENCODER llega de forma asíncrona
  vía el reporte automático del firmware). Ese hilo estaba deshabilitado
  en el proyecto (ver notas de memoria: "create_receive_threading()
  currently disabled — serial port concurrency must be validated").
  Este nodo lo activa explícitamente en __init__. ANTES de confiar en
  /odom_encoder para evaluar controladores, correr la validación aislada
  descrita en la tesis: robot quieto, verificar que get_motor_encoder()
  devuelve valores ESTABLES (no basura/ceros intermitentes) mientras se
  siguen enviando comandos de rueda/brazo simultáneamente.
"""

import time
import threading
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist, TransformStamped
from trajectory_msgs.msg import JointTrajectory
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import tf2_ros

from rosboard_drv.rosboard_drv import RosBoardDrv

from rx3_robot_bridge.pid_battery_common import (
    ARM_TOPIC, GRIP_TOPIC, ARM_MOVE_DUR,
)
from rx3_robot_bridge.encoder_odometry_math import (
    MecanumEncoderOdometry, EncoderOdometryConfig,
)

# ─── CALIBRACIÓN (valores medidos) ─────────────────────────────────────
CALIB = {
    1: {'zero': 90,  'sign':  1},   # arm_joint_01
    2: {'zero': 100, 'sign':  1},   # arm_joint_02
    3: {'zero': 65,  'sign':  1},   # arm_joint_03
    4: {'zero': 90,  'sign':  1},   # arm_joint_04
    5: {'zero': 90,  'sign':  1},   # arm_joint_05
}
LIMITS_SERVO = {
    1: (0, 180),
    2: (0, 180),
    3: (0, 180),
    4: (0, 180),
    5: (0, 270),   # servo 5 tiene rango extendido
}

GRIP_RAD_OPEN     = 0.0
GRIP_RAD_CLOSED   = -1.54
GRIP_SERVO_OPEN   = 0
GRIP_SERVO_CLOSED = 130
GRIP_LIMITS_SERVO = (0, 130)

WATCHDOG_TIMEOUT = 0.5  # s

# ─── Odometría por encoders — configuración por defecto ────────────────
# lx/ly EN ESTE ROBOT FÍSICO: reutilizamos como punto de partida los
# valores ya usados en simulación (omni_dofbot_description/xacro_properties),
# pero DEBEN remedirse sobre el chasis real -- no hay garantía de que
# coincidan exactamente. Ver protocolo de comparación LIDAR vs encoders.
ENCODER_ODOM_RATE_HZ = 30.0
DEFAULT_WHEEL_RADIUS = 0.040
DEFAULT_LX = 0.110
DEFAULT_LY = 0.102


class RosmasterBridgeNode(Node):

    def __init__(self):
        super().__init__('rosmaster_bridge_node')

        # ── Parámetros ROS ──────────────────────────────────────────────
        self.declare_parameter('car_type', 1)
        # CH341 -> Linux numera la interfaz como /dev/ttyCH341USB0 (o el
        # symlink de udev /dev/rosmaster si está configurado). El default
        # genérico /dev/ttyUSB0 fallaba al lanzar sin overrides explícitos.
        self.declare_parameter('com', '/dev/ttyCH341USB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.06)
        self.declare_parameter('kd', 0.5)

        # ── Parámetros de odometría por encoders ─────────────────────────
        self.declare_parameter('enc_wheel_radius', DEFAULT_WHEEL_RADIUS)
        self.declare_parameter('enc_lx', DEFAULT_LX)
        self.declare_parameter('enc_ly', DEFAULT_LY)
        self.declare_parameter('enc_odom_frame', 'odom_encoder')
        self.declare_parameter('enc_base_frame', 'base_footprint')
        self.declare_parameter('enc_publish_tf', False)   # False por defecto:
        # ya existe un TF odom->base_footprint publicado por rf2o; publicar
        # un segundo TF al mismo child_frame_id generaría un árbol TF
        # ambiguo. Se deja como opción para pruebas aisladas de comparación
        # donde rf2o esté apagado.

        self.add_on_set_parameters_callback(self._on_params_change)

        car_type = self.get_parameter('car_type').value
        port = self.get_parameter('com').value
        baud = self.get_parameter('baudrate').value

        self._bot = RosBoardDrv(car_type=car_type, com=port, baudrate=baud, debug=False)

        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self._bot.set_pid_param(kp, ki, kd, forever=False)

        # ── Habilitar hilo de recepción — REQUERIDO para get_motor_encoder() ──
        # Ver docstring del módulo: prerrequisito bloqueante antes de confiar
        # en /odom_encoder. Se activa aquí porque este nodo es el único
        # punto donde es seguro hacerlo (dueño exclusivo del puerto serie).
        self._bot.create_receive_threading()

        # create_receive_threading() solo abre el hilo LECTOR en Python.
        # En algunas versiones del firmware Yahboom, la placa NO empieza a
        # emitir FUNC_REPORT_ENCODER (ni el resto de tramas de auto-reporte)
        # de forma periódica hasta recibir explícitamente esta orden. Sin
        # ella, el hilo escucha un puerto vacío y get_motor_encoder()
        # devuelve datos obsoletos/nulos indefinidamente, no solo al
        # arranque.
        self._bot.set_auto_report_state(True, forever=False)

        self.get_logger().warn(
            'create_receive_threading() + set_auto_report_state(True) '
            'activados — validar que no hay corrupción de tramas por '
            'concurrencia con escrituras de rueda/brazo antes de usar '
            '/odom_encoder para evaluar controladores (ver nota de diseño '
            'en el módulo).')

        # ── Ventana de gracia de arranque para /odom_encoder ──────────────
        # Tras pedir el auto-reporte, la primera trama FUNC_REPORT_ENCODER
        # tarda un ciclo de firmware en llegar (además de que
        # get_motor_encoder() en RosBoardDrv puede no existir aún como
        # atributo inicializado en el primer poll). Durante esta ventana,
        # una respuesta None/incompleta es esperada y NO se reporta como
        # warning para no llenar la consola al iniciar; solo se convierte
        # en warning si persiste más allá de la ventana de gracia.
        self._enc_startup_time = time.time()
        self._enc_startup_grace_s = 1.0

        # ── Estado del watchdog (ruedas) ──────────────────────────────
        self._lock = threading.Lock()
        self._last_cmd_time = time.time()
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_cb)

        # ── Estado del brazo (última pose conocida de CADA canal) ───────
        self._arm_lock = threading.Lock()
        self._last_arm_rad  = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._last_grip_rad = GRIP_RAD_OPEN

        self.create_subscription(
            JointTrajectory, ARM_TOPIC, self._arm_trajectory_cb, 10)
        self.create_subscription(
            JointTrajectory, GRIP_TOPIC, self._grip_trajectory_cb, 10)

        # ── Odometría por encoders ────────────────────────────────────────
        enc_cfg = EncoderOdometryConfig(
            wheel_radius=self.get_parameter('enc_wheel_radius').value,
            lx=self.get_parameter('enc_lx').value,
            ly=self.get_parameter('enc_ly').value,
        )
        self._enc_odom = MecanumEncoderOdometry(enc_cfg)
        self._enc_odom_pub = self.create_publisher(Odometry, '/odom_encoder', 10)
        self._enc_tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self._enc_reset_srv = self.create_service(
            Empty, '~/reset_pose', self._enc_reset_cb)

        self._enc_timer = self.create_timer(
            1.0 / ENCODER_ODOM_RATE_HZ, self._encoder_odom_cb)

        self.get_logger().info(
            f'Bridge listo — puerto={port} car_type={car_type} '
            f'Kp={kp:.3f} Ki={ki:.3f} Kd={kd:.3f} — '
            f'odometría por encoders: r={enc_cfg.wheel_radius:.3f} '
            f'lx={enc_cfg.lx:.3f} ly={enc_cfg.ly:.3f} '
            f'({ENCODER_ODOM_RATE_HZ:.0f} Hz -> /odom_encoder)')

    # ─── Ruedas ──────────────────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        with self._lock:
            self._last_cmd_time = time.time()
        self._bot.set_car_motion(float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))

    def _watchdog_cb(self):
        with self._lock:
            stale = (time.time() - self._last_cmd_time) > WATCHDOG_TIMEOUT
        if stale:
            self._bot.set_car_motion(0.0, 0.0, 0.0)

    # ─── Odometría por encoders ──────────────────────────────────────────
    def _encoder_odom_cb(self):
        raw = None
        try:
            raw = self._bot.get_motor_encoder()   # FL, FR, BR, BL
            c1, c2, c3, c4 = raw
        except (TypeError, ValueError) as e:
            # Durante la ventana de gracia de arranque (ver __init__), es
            # esperado que get_motor_encoder() no tenga aún una trama
            # completa (retorno None o tupla incompleta) mientras el
            # firmware procesa set_auto_report_state(True). Se degrada a
            # debug() en ese caso para no ensuciar la consola; pasada la
            # ventana, sí se reporta como warn() porque indicaría que el
            # auto-reporte nunca arrancó o el hilo de recepción se cayó.
            elapsed = time.time() - self._enc_startup_time
            msg = f'get_motor_encoder() devolvió dato no desempaquetable ({raw!r}): {e}'
            if elapsed < self._enc_startup_grace_s:
                self.get_logger().debug(msg)
            else:
                self.get_logger().warn(msg)
            return
        except Exception as e:
            self.get_logger().warn(f'get_motor_encoder() falló: {e}')
            return

        now = self.get_clock().now()
        result = self._enc_odom.update(c1, c2, c3, c4, now=now.nanoseconds * 1e-9)
        if result is None:
            return   # primer ciclo, sin delta aún

        x, y, theta, vx, vy, wz = result
        stamp = now.to_msg()
        odom_frame = self.get_parameter('enc_odom_frame').value
        base_frame = self.get_parameter('enc_base_frame').value

        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0]  = 0.01
        odom.pose.covariance[7]  = 0.01
        odom.pose.covariance[35] = 0.05
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = wz
        odom.twist.covariance[0]  = 0.01
        odom.twist.covariance[7]  = 0.01
        odom.twist.covariance[35] = 0.05
        self._enc_odom_pub.publish(odom)

        if self.get_parameter('enc_publish_tf').value:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = odom_frame
            t.child_frame_id = base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self._enc_tf_broadcaster.sendTransform(t)

    def _enc_reset_cb(self, request, response):
        """Resetea la pose acumulada de /odom_encoder a (0,0,0).
        Llamar junto con el reset de rf2o (si aplica) en cada
        _manual_reset() de hw_pid_battery.py, para que ambas fuentes
        arranquen sincronizadas en cada repetición de prueba."""
        self._enc_odom.reset(0.0, 0.0, 0.0)
        self.get_logger().info('Odometría por encoders reseteada a (0, 0, 0).')
        return response

    # ─── Brazo: conversión radianes → grados servo ──────────────────────
    def _rad_to_servo(self, s_id, rad):
        if s_id == 6:
            frac = (rad - GRIP_RAD_OPEN) / (GRIP_RAD_CLOSED - GRIP_RAD_OPEN)
            frac = max(0.0, min(1.0, frac))
            servo_deg = GRIP_SERVO_OPEN + frac * (GRIP_SERVO_CLOSED - GRIP_SERVO_OPEN)
            lim = GRIP_LIMITS_SERVO
        else:
            deg = math.degrees(rad)
            cal = CALIB.get(s_id, {'zero': 90, 'sign': 1})
            servo_deg = cal['zero'] + cal['sign'] * deg
            lim = LIMITS_SERVO.get(s_id, (0, 180))

        servo_deg = max(lim[0], min(lim[1], servo_deg))
        return int(round(servo_deg))

    def _send_arm_pose(self, pose_rad, run_time_ms=1000):
        """pose_rad: 6 flotantes en radianes (joints 1-5 + gripper)."""
        if len(pose_rad) != 6:
            self.get_logger().error("La pose debe tener 6 elementos")
            return
        servo_angles = [self._rad_to_servo(i + 1, pose_rad[i]) for i in range(6)]
        self.get_logger().info(f"Enviando ángulos a servos: {servo_angles}")
        self._bot.set_uart_servo_angle_array(servo_angles, run_time_ms)

    # ─── Callbacks de JointTrajectory ────────────────────────────────────
    def _arm_trajectory_cb(self, msg: JointTrajectory):
        """Recibe las 5 articulaciones; conserva el último gripper conocido."""
        if not msg.points:
            return
        positions = list(msg.points[0].positions)[:5]
        while len(positions) < 5:
            positions.append(0.0)

        with self._arm_lock:
            self._last_arm_rad = positions
            full_pose = positions + [self._last_grip_rad]

        self._send_arm_pose(full_pose, int(ARM_MOVE_DUR * 1000))

    def _grip_trajectory_cb(self, msg: JointTrajectory):
        """Recibe el gripper; conserva la última pose de brazo conocida."""
        if not msg.points:
            return
        grip_rad = msg.points[0].positions[0] if msg.points[0].positions else GRIP_RAD_OPEN

        with self._arm_lock:
            self._last_grip_rad = grip_rad
            full_pose = self._last_arm_rad + [grip_rad]

        self._send_arm_pose(full_pose, int(ARM_MOVE_DUR * 1000))

    # ─── Parámetros dinámicos ─────────────────────────────────────────────
    def _on_params_change(self, params):
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value

        # Valores actuales de la config de odometría por encoders, para
        # detectar si alguno de los tres cambió en esta llamada.
        enc_changed = False
        wheel_radius = self._enc_odom.cfg.wheel_radius
        lx = self._enc_odom.cfg.lx
        ly = self._enc_odom.cfg.ly

        for p in params:
            if p.name == 'kp': kp = p.value
            elif p.name == 'ki': ki = p.value
            elif p.name == 'kd': kd = p.value
            elif p.name == 'enc_wheel_radius':
                wheel_radius = p.value; enc_changed = True
            elif p.name == 'enc_lx':
                lx = p.value; enc_changed = True
            elif p.name == 'enc_ly':
                ly = p.value; enc_changed = True

        self._bot.set_pid_param(kp, ki, kd, forever=False)
        self.get_logger().info(f'PID actualizado — Kp={kp:.4f} Ki={ki:.4f} Kd={kd:.4f}')

        if enc_changed:
            # El callback de rclpy dispara DESPUÉS de que los parámetros
            # ya se validaron y están a punto de aplicarse, así que aquí
            # es seguro leer los valores nuevos vía la variable local
            # (get_parameter() todavía devolvería el valor viejo dentro
            # de este mismo callback). Se actualiza self.cfg in-place en
            # vez de crear una instancia nueva de MecanumEncoderOdometry,
            # para NO perder _prev_counts/_prev_time/x/y/theta ya
            # acumulados — un cambio de radio de rueda a mitad de una
            # prueba no debe reiniciar la pose, solo afectar los
            # incrementos futuros.
            self._enc_odom.cfg.wheel_radius = wheel_radius
            self._enc_odom.cfg.lx = lx
            self._enc_odom.cfg.ly = ly
            self.get_logger().info(
                f'Config de odometría por encoders actualizada — '
                f'wheel_radius={wheel_radius:.4f} lx={lx:.4f} ly={ly:.4f} '
                f'(k=lx+ly={lx+ly:.4f}). Pose acumulada NO se reinició.')

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = RosmasterBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()