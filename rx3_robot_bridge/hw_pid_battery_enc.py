#!/usr/bin/env python3
"""
hw_pid_battery_encoder.py
=========================
Batería de pruebas P1/P2/P3 sobre el ROSMASTER X3 PLUS físico.

CONTROL: Odometría por Encoders (/odom_encoder)
SEGURIDAD: Paro de Emergencia Activo (LIDAR /scan y Ctrl+X)
REGISTRO: LIDAR (/odom vía rf2o) en paralelo para comparación.

═══════════════════════════════════════════════════════════════════════════
CORRECCIONES respecto a la versión anterior (bitácora de fixes)
═══════════════════════════════════════════════════════════════════════════
1. E-STOP AHORA VIGILA FRENTE Y ATRÁS, no solo el cono frontal:
   La versión anterior solo revisaba abs(angle) < E_STOP_HALF_ANGLE (cono
   frontal). Como P1_atras mueve el robot con vx negativo (hacia atrás),
   un obstáculo detrás del robot era invisible para el E-stop pese a que
   el LIDAR sí lo detecta -- el filtro angular lo descartaba. Ahora se
   vigilan DOS conos fijos (frontal y trasero, ±E_STOP_HALF_ANGLE
   alrededor de 0 rad y de pi rad), cubriendo ambos sentidos de la
   batería de pruebas actual (P1 avanza y retrocede; P2 es rotación pura,
   sin desplazamiento, así que ambos conos cubren igual el punto de giro;
   P3 combina avance+giro+avance, siempre en +x). No se vigila el cono
   lateral porque la batería vigente no incluye desplazamiento lateral
   -- documentado como limitación conocida, igual que el resto de la
   tesis señala explícitamente sus supuestos de alcance.

2. _manual_reset() ahora revisa self._e_stop DESPUÉS del loop de espera,
   no solo antes. Si el E-stop se dispara mientras el operador está
   reposicionando el robot, ya no se procede a resetear /odom_encoder ni
   a fijar el origen -- evitaría fijar un origen "válido" justo después
   de una emergencia.

3. _keyboard_monitor() ahora:
   a) protege termios.tcgetattr() con try/except para no crashear en
      silencio si stdin no es una tty (p. ej. lanzado desde un archivo
      launch.py sin terminal interactiva).
   b) restaura la terminal de forma EXPLÍCITA en el finally de main(),
      no solo confiando en el finally del hilo daemon (que puede no
      ejecutarse si el proceso termina de forma abrupta).
   c) usa una bandera _kb_thread_running explícita para poder unirse al
      hilo con timeout en shutdown, en vez de depender únicamente de
      rclpy.ok().

4. run_once() ya no aliasa la misma lista/objeto SegmentLog entre las
   tres pruebas en sus valores por defecto -- cada una obtiene su propia
   instancia independiente.

5. _reset_encoder_odom() ahora loggea explícitamente cuando el servicio
   no está disponible o hace timeout, en vez de fallar en silencio.

6. El costo de aborto total por E-stop ya no es el número mágico
   "PENALTY_TO * 3": se nombra EMERGENCY_ABORT_COST con un comentario
   explicando por qué es mayor que un PENALTY_TO simple (representa el
   fallo simultáneo de las tres pruebas, no de un solo segmento).

7. _scan_cb() ahora descarta mensajes /scan vacíos antes de iterar.
"""

import os
import sys
import select
import termios
import tty
import math
import time
import json
import threading
from dataclasses import dataclass, field
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

from rx3_robot_bridge.pid_battery_common import (
    DIST_X, DIST_RETURN, ROT_ANGLE, VX_REF, WZ_REF,
    CTRL_DT, SETTLE_TIME, TIMEOUT_MOVE, TIMEOUT_ROT, POS_TOL, YAW_TOL,
    MAX_POS_ERROR_ABORT, W1, W2, W3, PENALTY_TO,
    ARM_JOINTS, GRIP_JOINTS, ARM_TOPIC, GRIP_TOPIC, ARM_MOVE_DUR,
    ARM_HOLD_DUR, ARM_HOME, ARM_PICK_LEFT, ARM_PICK_RIGHT,
    ARM_CHOREOGRAPHY, GRIP_OPEN, GRIP_CLOSED, SegmentLog, Pose2D,
)

BRIDGE_PARAM_SERVICE = "/rosmaster_bridge_node/set_parameters"
BRIDGE_ENC_RESET_SERVICE = "/rosmaster_bridge_node/reset_pose"
OUT_JSON = "hw_battery_results_encoder_control.json"

# ── Parámetros de seguridad del E-stop ──────────────────────────────────────
E_STOP_MIN_RANGE = 0.30        # m — distancia mínima antes de frenar
E_STOP_HALF_ANGLE = 0.52       # rad (~30°) — semiancho del cono frontal vigilado
E_STOP_FRONT_CENTER = 0.0      # rad

# ═══════════════════════════════════════════════════════════════════════════
# CORRECCIÓN — el LIDAR NO puede cubrir la parte trasera del chasis
# ═══════════════════════════════════════════════════════════════════════════
# Las tarjetas de control montadas en el chasis bloquean físicamente el campo
# de visión del YDLIDAR TG30 hacia atrás. Un chequeo de "cono trasero" sobre
# esas lecturas NO detecta obstáculos: detecta las propias tarjetas a muy
# corta distancia, de forma constante -- lo que dispararía el E-stop en
# falso desde el arranque del nodo (el chasis siempre estaría a menos de
# E_STOP_MIN_RANGE en esas lecturas). Por eso:
#   1. El E-stop automático por LIDAR SOLO vigila el cono frontal.
#   2. El arco físicamente bloqueado se ENMASCARA (se ignora por completo,
#      ni siquiera se evalúa) para que esas lecturas de auto-detección no
#      contaminen ningún chequeo, presente o futuro.
#   3. La falta de cobertura trasera se compensa con una confirmación visual
#      MANUAL del operador antes de cada movimiento hacia atrás (único caso
#      en la batería actual: P1_atras) -- ver _confirm_rear_clear().
#
# CALIBRAR EN EL ROBOT FÍSICO: estos valores son un punto de partida
# razonable, no una medición. Ajustar lidar_blind_arc_center_deg /
# lidar_blind_arc_halfwidth_deg (parámetros ROS, ver __init__) hasta que el
# arco cubra exactamente la sombra real de las tarjetas de control sobre el
# barrido del TG30 -- ni más (perdería cobertura frontal útil) ni menos
# (dejaría pasar auto-detecciones del chasis).
LIDAR_BLIND_ARC_CENTER_DEG_DEFAULT = 180.0
LIDAR_BLIND_ARC_HALFWIDTH_DEG_DEFAULT = 60.0

# Costo de aborto cuando el E-stop se dispara ANTES de que corra cualquier
# prueba (run_once no alcanza a ejecutar ni P1). Es mayor que un PENALTY_TO
# simple porque representa el fallo simultáneo de las tres pruebas P1+P2+P3,
# no la falla de un solo segmento dentro de una prueba (que ya usa
# PENALTY_TO tal cual, sin multiplicar, dentro de _run_testN).
EMERGENCY_ABORT_COST = PENALTY_TO * 3


class HardwareBatteryEvaluatorEnc(Node):
    def __init__(self):
        super().__init__("hw_pid_battery_eval_enc")
        cbg = ReentrantCallbackGroup()

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._arm_pub = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self._grip_pub = self.create_publisher(JointTrajectory, GRIP_TOPIC, 10)

        self._pid_cli = self.create_client(
            SetParameters, BRIDGE_PARAM_SERVICE, callback_group=cbg)
        self._enc_reset_cli = self.create_client(
            Empty, BRIDGE_ENC_RESET_SERVICE, callback_group=cbg)

        self._lock = threading.Lock()

        # ── ESTADO GLOBAL DE SEGURIDAD (E-STOP) Y TECLADO ──
        self._e_stop = False
        self._waiting_for_enter = False
        self._kb_thread_running = False   # FIX #3c: bandera explícita de vida del hilo

        # ── Arco ciego del LIDAR bloqueado por las tarjetas de control ──────
        # Parámetros ROS para poder calibrar sin recompilar. Ver nota de
        # diseño junto a LIDAR_BLIND_ARC_*_DEFAULT sobre por qué esto es
        # obligatorio (no opcional) en este chasis.
        self.declare_parameter('lidar_blind_arc_center_deg', LIDAR_BLIND_ARC_CENTER_DEG_DEFAULT)
        self.declare_parameter('lidar_blind_arc_halfwidth_deg', LIDAR_BLIND_ARC_HALFWIDTH_DEG_DEFAULT)
        self._blind_arc_center = math.radians(
            self.get_parameter('lidar_blind_arc_center_deg').value)
        self._blind_arc_half = math.radians(
            self.get_parameter('lidar_blind_arc_halfwidth_deg').value)

        # ── Estado encoder (FUENTE DE CONTROL) ──
        self._enc_pose = Pose2D()
        self._origin_enc = Pose2D()
        self._current_vref = (0.0, 0.0, 0.0)
        self._itae_target_enc = Pose2D()
        self._yaw_ref_live_enc = 0.0
        self._measuring = False
        self._seg_log_enc = None
        self._itae_accum = 0.0
        self._start_eval_t = 0.0
        self._last_enc_odom_t = 0.0

        # ── Estado rf2o (LIDAR) (SOLO REGISTRO) ──
        self._pose = Pose2D()
        self._origin = Pose2D()
        self._itae_target = Pose2D()
        self._yaw_ref_live = 0.0
        self._seg_log = None
        self._last_odom_t = 0.0

        self._arm_active = False

        self.create_subscription(Odometry, "/odom", self._odom_cb, qos_profile_sensor_data, callback_group=cbg)
        self.create_subscription(Odometry, "/odom_encoder", self._encoder_odom_cb, qos_profile_sensor_data, callback_group=cbg)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data, callback_group=cbg)

        self._kb_thread = threading.Thread(target=self._keyboard_monitor, daemon=True)
        self._kb_thread.start()

        self.get_logger().info("HardwareBatteryEvaluatorEnc listo.")
        self.get_logger().warn(
            f"CONTROL: Encoders. E-STOP AUTOMÁTICO por LIDAR SOLO cubre el "
            f"cono frontal (±{math.degrees(E_STOP_HALF_ANGLE):.0f}°) < "
            f"{E_STOP_MIN_RANGE:.2f}m. Arco ciego enmascarado: "
            f"{math.degrees(self._blind_arc_center):.0f}°±"
            f"{math.degrees(self._blind_arc_half):.0f}°. "
            f"LA PARTE TRASERA NO TIENE SENSADO AUTOMÁTICO -- cada movimiento "
            f"hacia atrás pedirá confirmación visual manual. Ctrl+X sigue "
            f"disponible en cualquier momento.")

    # ──  SEGURIDAD Y PARO DE EMERGENCIA  ──
    def _scan_cb(self, msg: LaserScan):
        if self._e_stop:
            return
        if not msg.ranges:   # FIX #7 — mensaje vacío, nada que revisar
            return

        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < msg.range_min:
                continue
            if r >= E_STOP_MIN_RANGE:
                continue

            angle = angle_min + i * angle_inc
            angle = (angle + math.pi) % (2 * math.pi) - math.pi

            # Enmascarar el arco bloqueado por las tarjetas de control ANTES
            # de cualquier evaluación: esas lecturas son auto-detección del
            # propio chasis, no del entorno, y deben ignorarse por completo
            # (ni siquiera loguearse como "no coincide con el frente" --
            # simplemente no participan del chequeo).
            if abs(self._angle_diff(angle, self._blind_arc_center)) < self._blind_arc_half:
                continue

            # El E-stop automático por LIDAR SOLO vigila el cono frontal --
            # ver nota de diseño al inicio del módulo sobre por qué la parte
            # trasera no puede monitorearse con este sensor en este chasis.
            in_front = abs(self._angle_diff(angle, E_STOP_FRONT_CENTER)) < E_STOP_HALF_ANGLE
            if in_front:
                self.get_logger().error(
                    f"🛑 ¡COLISIÓN INMINENTE (FRONTAL)! Obstáculo a {r:.2f}m. "
                    f"ACTIVANDO E-STOP 🛑")
                self._e_stop = True
                self._stop()
                break

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Diferencia angular normalizada a (-pi, pi], para comparar contra
        un centro de cono sin problemas de wraparound en ±pi."""
        d = a - b
        return (d + math.pi) % (2 * math.pi) - math.pi

    def _keyboard_monitor(self):
        # FIX #3a/#4 — proteger contra stdin no interactivo (p. ej. lanzado
        # desde un launch.py sin terminal). Sin esto, termios.tcgetattr()
        # lanza una excepción que mata el hilo daemon en silencio y el
        # Ctrl+X deja de funcionar sin ningún aviso.
        if not sys.stdin.isatty():
            self.get_logger().warn(
                "stdin no es una terminal interactiva — Ctrl+X deshabilitado "
                "en esta sesión. El E-stop por LIDAR sigue activo.")
            return

        try:
            settings = termios.tcgetattr(sys.stdin)
        except termios.error as e:
            self.get_logger().warn(f"No se pudo leer configuración de terminal: {e}")
            return

        self._kb_thread_running = True
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok() and self._kb_thread_running:
                dr, _, _ = select.select([sys.stdin], [], [], 0.1)
                if dr:
                    key = sys.stdin.read(1)
                    if key == '\x18':  # Ctrl+X
                        self.get_logger().error(
                            "🛑 ¡PARO DE EMERGENCIA MANUAL (Ctrl+X) ACTIVADO! 🛑\r")
                        self._e_stop = True
                        self._waiting_for_enter = False
                        self._stop()
                    elif key == '\x03':  # Ctrl+C
                        break
                    elif key in ('\r', '\n') and self._waiting_for_enter:
                        self._waiting_for_enter = False
        finally:
            # FIX #3b — restauración local por si el hilo termina por su
            # propia cuenta (break normal); la restauración "de emergencia"
            # ante muerte abrupta del proceso vive en main()/restore_terminal().
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            except termios.error:
                pass
            self._kb_thread_running = False

    def stop_keyboard_monitor(self):
        """Señaliza al hilo de teclado que termine y espera brevemente a
        que lo haga, para poder unirse a él de forma ordenada desde
        main()."""
        self._kb_thread_running = False
        if hasattr(self, "_kb_thread") and self._kb_thread.is_alive():
            self._kb_thread.join(timeout=0.5)

    # ── Odometría por encoders — FUENTE DE CONTROL ────
    def _encoder_odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z

        with self._lock:
            self._enc_pose = Pose2D(msg.pose.pose.position.x,
                                    msg.pose.pose.position.y, yaw)
            if self._measuring:
                now = time.time()
                dt = max(now - self._last_enc_odom_t, 0.001)
                self._last_enc_odom_t = now

                ex = self._itae_target_enc.x - self._enc_pose.x
                ey = self._itae_target_enc.y - self._enc_pose.y
                err = math.hypot(ex, ey)
                t_real = now - self._start_eval_t

                self._itae_accum += t_real * err * dt

                if self._seg_log_enc is not None:
                    vref_x, vref_y, vref_wz = self._current_vref
                    self._seg_log_enc.t.append(t_real)
                    self._seg_log_enc.vx_ref.append(vref_x)
                    self._seg_log_enc.vy_ref.append(vref_y)
                    self._seg_log_enc.wz_ref.append(vref_wz)
                    self._seg_log_enc.vx_real.append(vx)
                    self._seg_log_enc.vy_real.append(vy)
                    self._seg_log_enc.wz_real.append(wz)
                    self._seg_log_enc.pos_err.append(err)
                    self._seg_log_enc.x_ref.append(self._itae_target_enc.x)
                    self._seg_log_enc.y_ref.append(self._itae_target_enc.y)
                    self._seg_log_enc.yaw_ref.append(self._yaw_ref_live_enc)
                    self._seg_log_enc.x_real.append(self._enc_pose.x)
                    self._seg_log_enc.y_real.append(self._enc_pose.y)
                    self._seg_log_enc.yaw_real.append(self._enc_pose.yaw)

    # ── Odometría rf2o (LIDAR) — SOLO REGISTRO ──
    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z

        with self._lock:
            self._pose = Pose2D(msg.pose.pose.position.x,
                                msg.pose.pose.position.y, yaw)
            if self._measuring and self._seg_log is not None:
                now = time.time()
                self._last_odom_t = now

                ex = self._itae_target.x - self._pose.x
                ey = self._itae_target.y - self._pose.y
                err = math.hypot(ex, ey)
                t_real = now - self._start_eval_t

                vref_x, vref_y, vref_wz = self._current_vref
                self._seg_log.t.append(t_real)
                self._seg_log.vx_ref.append(vref_x)
                self._seg_log.vy_ref.append(vref_y)
                self._seg_log.wz_ref.append(vref_wz)
                self._seg_log.vx_real.append(vx)
                self._seg_log.vy_real.append(vy)
                self._seg_log.wz_real.append(wz)
                self._seg_log.pos_err.append(err)
                self._seg_log.x_ref.append(self._itae_target.x)
                self._seg_log.y_ref.append(self._itae_target.y)
                self._seg_log.yaw_ref.append(self._yaw_ref_live)
                self._seg_log.x_real.append(self._pose.x)
                self._seg_log.y_real.append(self._pose.y)
                self._seg_log.yaw_real.append(self._pose.yaw)

    def _get_pose(self) -> Pose2D:
        with self._lock:
            return self._pose.copy()

    def _get_pose_enc(self) -> Pose2D:
        with self._lock:
            return self._enc_pose.copy()

    def _fix_origin(self):
        with self._lock:
            self._origin = self._pose.copy()
            self._origin_enc = self._enc_pose.copy()

    def _pose_rel(self) -> Pose2D:
        with self._lock:
            return Pose2D(self._pose.x - self._origin.x,
                          self._pose.y - self._origin.y,
                          self._pose.yaw)

    def _pose_rel_enc(self) -> Pose2D:
        with self._lock:
            return Pose2D(self._enc_pose.x - self._origin_enc.x,
                          self._enc_pose.y - self._origin_enc.y,
                          self._enc_pose.yaw)

    def _start_itae(self, target_enc: Pose2D, seg_log_enc: SegmentLog = None,
                     target_rf2o: Pose2D = None, seg_log_rf2o: SegmentLog = None):
        with self._lock:
            self._itae_target_enc = target_enc.copy()
            self._itae_accum = 0.0
            self._start_eval_t = time.time()
            self._last_enc_odom_t = time.time()
            self._measuring = True
            self._seg_log_enc = seg_log_enc
            self._yaw_ref_live_enc = target_enc.yaw

            t_rf2o = target_rf2o if target_rf2o is not None else target_enc
            self._itae_target = t_rf2o.copy()
            self._seg_log = seg_log_rf2o
            self._yaw_ref_live = t_rf2o.yaw
            self._last_odom_t = time.time()

    def _stop_itae(self) -> float:
        with self._lock:
            self._measuring = False
            self._seg_log = None
            self._seg_log_enc = None
            return self._itae_accum

    def _reset_encoder_odom(self) -> bool:
        if not self._enc_reset_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Timeout: Servicio ~/reset_pose no disponible.")
            return False
            
        fut = self._enc_reset_cli.call_async(Empty.Request())
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 2.0:
            time.sleep(0.05)
            
        if not fut.done():
            self.get_logger().error("Fallo crítico: El bridge no resolvió el reset a tiempo.")
            return False
            
        return True
        
    def _wait_for_enter(self, prompt: str) -> bool:
        """Bloquea hasta que el operador presione ENTER (vía el hilo de
        teclado) o se dispare el E-stop. Devuelve True si se completó con
        ENTER, False si fue interrumpido por E-stop -- el caller SIEMPRE
        debe revisar el valor de retorno (o self._e_stop directamente)
        antes de proceder, en vez de asumir que la espera terminó de forma
        normal."""
        print(f"\n>>> {prompt}\r")
        self._waiting_for_enter = True
        while self._waiting_for_enter and rclpy.ok() and not self._e_stop:
            time.sleep(0.1)
        if self._e_stop:
            self._waiting_for_enter = False
            return False
        return True

    def _confirm_rear_clear(self) -> bool:
        """Confirmación visual MANUAL antes de cualquier movimiento hacia
        atrás. El LIDAR no puede sustituir esto en este chasis -- ver nota
        de diseño junto a LIDAR_BLIND_ARC_*_DEFAULT. Devuelve False si el
        E-stop interrumpió la espera (en cuyo caso el _drive() que sigue
        detectará self._e_stop y se saltará el movimiento de todos modos,
        pero se revisa aquí también para loggear la causa con claridad)."""
        ok = self._wait_for_enter(
            "PARTE TRASERA: verifica visualmente que está despejada "
            "(el LIDAR NO la cubre en este chasis). Presiona ENTER para "
            "iniciar el movimiento hacia atrás...")
        if not ok:
            self.get_logger().warn(
                "Confirmación de parte trasera interrumpida por E-stop — "
                "el movimiento hacia atrás se omitirá.")
        return ok

    def _manual_reset(self):
        self._stop()
        if self._e_stop: return 
        time.sleep(0.2)
        print("\n>>> Reposiciona el robot manualmente sobre la marca de origen y presiona ENTER...\r")
        self._waiting_for_enter = True
        while self._waiting_for_enter and rclpy.ok() and not self._e_stop:
            time.sleep(0.1)
        time.sleep(0.5)
        
        # EL FIX: Aborto seguro si el reset falla
        reset_exitoso = self._reset_encoder_odom()
        if not reset_exitoso:
            self.get_logger().error("ABORTANDO: No se puede garantizar un yaw inicial limpio por fallo de reset.")
            self._e_stop = True  # Esto interrumpe automáticamente la batería de pruebas
            return
            
        time.sleep(0.15)   
        self._fix_origin()

    @staticmethod
    def _compute_target(p0: Pose2D, dist_m: float, axis: str) -> Pose2D:
        if axis == "x":
            return Pose2D(p0.x + math.cos(p0.yaw)*dist_m, p0.y + math.sin(p0.yaw)*dist_m, p0.yaw)
        else:
            return Pose2D(p0.x - math.sin(p0.yaw)*dist_m, p0.y + math.cos(p0.yaw)*dist_m, p0.yaw)

    def _send(self, vx=0.0, vy=0.0, wz=0.0):
        if self._e_stop:
            vx = vy = wz = 0.0
        with self._lock:
            self._current_vref = (vx, vy, wz)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(wz)
        self._cmd_pub.publish(msg)

    def _stop(self):
        self._send()

    def _drive(self, dist_m, axis, vx=0.0, vy=0.0, timeout=TIMEOUT_MOVE, seg_name=""):
        if self._e_stop:
            self.get_logger().warn(f"[{seg_name}] Omitido debido a Paro de Emergencia Activo.")
            return 0.0, 0.0, False, SegmentLog(name=seg_name+"_enc"), SegmentLog(name=seg_name)

        self.get_logger().info(f"[{seg_name}] Iniciando movimiento: obj={dist_m:.2f}m, vel={vx:.2f}m/s")

        p0_enc = self._get_pose_enc()
        target_enc = self._compute_target(p0_enc, dist_m, axis)
        p0 = self._get_pose()
        target = self._compute_target(p0, dist_m, axis)

        slog_enc = SegmentLog(name=seg_name + "_enc")
        slog = SegmentLog(name=seg_name)
        self._start_itae(target_enc, slog_enc, target, slog)
        t0, ok, aborted = time.time(), False, False

        while time.time() - t0 < timeout:
            if self._e_stop:
                self.get_logger().error(f"[{seg_name}] Movimiento abortado por E-STOP en plena ejecución.")
                aborted = True
                break

            p_enc = self._get_pose_enc()
            dx, dy = p_enc.x - p0_enc.x, p_enc.y - p0_enc.y
            traveled = (dx*math.cos(p0_enc.yaw) + dy*math.sin(p0_enc.yaw) if axis == "x" else -dx*math.sin(p0_enc.yaw) + dy*math.cos(p0_enc.yaw))
            lateral_dev = math.hypot(dx, dy) - abs(traveled)

            if abs(lateral_dev) > MAX_POS_ERROR_ABORT:
                self.get_logger().warn(f"[{seg_name}] Abortado por desvío lateral severo: {lateral_dev:.3f}m")
                aborted = True
                break
            if abs(traveled) >= abs(dist_m) - POS_TOL:
                self.get_logger().info(f"[{seg_name}] ¡Meta alcanzada de forma exitosa! (Recorrido: {traveled:.3f}m)")
                ok = True
                break
            if abs(traveled) > abs(dist_m) + 0.12:
                self.get_logger().warn(f"[{seg_name}] Sobrecarga o Overshoot detectado: {traveled:.3f}m")
                break

            self._send(vx=vx, vy=vy)
            time.sleep(CTRL_DT)

        self._stop()
        itae = self._stop_itae()
        elapsed = time.time() - t0
        time.sleep(SETTLE_TIME)
        return itae, elapsed, (ok and not aborted), slog_enc, slog

    def _rotate(self, angle_rad, timeout=TIMEOUT_ROT, seg_name=""):
        if self._e_stop:
            self.get_logger().warn(f"[{seg_name}] Giro omitido por Paro de Emergencia.")
            return 0.0, False, SegmentLog(name=seg_name+"_enc"), 0.0, SegmentLog(name=seg_name)

        self.get_logger().info(f"[{seg_name}] Iniciando giro: obj={math.degrees(angle_rad):.1f}°")

        p0_enc = self._get_pose_enc()
        goal_yaw_enc = p0_enc.yaw + angle_rad
        target_enc = Pose2D(p0_enc.x, p0_enc.y, goal_yaw_enc)
        sign = math.copysign(1.0, angle_rad)
        t0, ok = time.time(), False

        p0 = self._get_pose()
        goal_yaw = p0.yaw + angle_rad
        target = Pose2D(p0.x, p0.y, goal_yaw)

        slog_enc = SegmentLog(name=seg_name + "_enc") if seg_name else None
        slog = SegmentLog(name=seg_name) if seg_name else None
        self._start_itae(target_enc, slog_enc, target, slog)

        while time.time() - t0 < timeout:
            if self._e_stop:
                self.get_logger().error(f"[{seg_name}] Giro abortado por E-STOP en ejecución.")
                ok = False
                break

            p_enc = self._get_pose_enc()
            diff = (goal_yaw_enc - p_enc.yaw + math.pi) % (2*math.pi) - math.pi
            if abs(diff) <= YAW_TOL:
                self.get_logger().info(f"[{seg_name}] Giro completado exitosamente. Error final: {math.degrees(diff):.2f}°")
                ok = True
                break
            wz = sign * max(0.15, min(WZ_REF, abs(diff)*1.5))
            t_rel = time.time() - t0
            est_total = max(abs(angle_rad)/WZ_REF, 1e-3)
            frac = min(1.0, t_rel/est_total)
            with self._lock:
                self._yaw_ref_live_enc = p0_enc.yaw + angle_rad*frac
                self._yaw_ref_live = p0.yaw + angle_rad*frac
                self._current_vref = (0.0, 0.0, wz)
            self._send(wz=wz)
            time.sleep(CTRL_DT)

        self._stop()
        with self._lock:
            self._yaw_ref_live_enc = goal_yaw_enc
            self._yaw_ref_live = goal_yaw
        self._stop_itae()
        p_enc = self._get_pose_enc()
        diff = (goal_yaw_enc - p_enc.yaw + math.pi) % (2*math.pi) - math.pi
        elapsed = time.time() - t0
        time.sleep(SETTLE_TIME)
        return abs(diff), ok, slog_enc, elapsed, slog

    def _set_pid(self, kp, ki, kd):
        if not self._pid_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"{BRIDGE_PARAM_SERVICE} no disponible — PID no actualizado.")
            return
        def _p(n, v): return Parameter(name=n, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))
        req = SetParameters.Request()
        req.parameters = [_p("kp", kp), _p("ki", ki), _p("kd", kd)]
        fut = self._pid_cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time()-t0 < 3.0: time.sleep(0.05)

    def _send_arm(self, positions, duration_sec=ARM_MOVE_DUR):
        if self._e_stop: return
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        pt.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        msg.points.append(pt)
        self._arm_pub.publish(msg)

    def _send_grip(self, position, duration_sec=ARM_MOVE_DUR):
        if self._e_stop: return
        msg = JointTrajectory()
        msg.joint_names = GRIP_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(position)]
        pt.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        msg.points.append(pt)
        self._grip_pub.publish(msg)

    def _wait_arm(self, duration_sec):
        t0 = time.time()
        while self._arm_active and time.time()-t0 < duration_sec:
            if self._e_stop: return False
            time.sleep(0.05)
        return self._arm_active

    def _arm_loop(self):
        side = "left"
        while self._arm_active and not self._e_stop:
            pick = ARM_PICK_LEFT if side == "left" else ARM_PICK_RIGHT
            place = ARM_PICK_RIGHT if side == "left" else ARM_PICK_LEFT
            self._send_arm(pick)
            self._send_grip(GRIP_OPEN)
            if not self._wait_arm(ARM_MOVE_DUR + ARM_HOLD_DUR): break
            self._send_grip(GRIP_CLOSED)
            if not self._wait_arm(ARM_HOLD_DUR): break
            self._send_arm(ARM_HOME)
            if not self._wait_arm(ARM_MOVE_DUR + ARM_HOLD_DUR): break
            self._send_arm(place)
            if not self._wait_arm(ARM_MOVE_DUR + ARM_HOLD_DUR): break
            self._send_grip(GRIP_OPEN)
            if not self._wait_arm(ARM_HOLD_DUR): break
            self._send_arm(ARM_HOME)
            if not self._wait_arm(ARM_MOVE_DUR + ARM_HOLD_DUR): break
            side = "right" if side == "left" else "left"
        if not self._e_stop:
            self._send_arm(ARM_HOME)
            self._send_grip(GRIP_OPEN)

    def _start_arm(self):
        self._arm_active = True
        self._arm_thread = threading.Thread(target=self._arm_loop, daemon=True)
        self._arm_thread.start()

    def _stop_arm(self):
        self._arm_active = False
        if hasattr(self, "_arm_thread"):
            self._arm_thread.join(timeout=ARM_MOVE_DUR + 1.0)

    def _run_test1(self):
        self.get_logger().info("── P1: línea recta adelante-atrás ──")
        self._manual_reset()
        i1, t1, ok1, s1e, s1 = self._drive(DIST_X, "x", vx=+VX_REF, seg_name="P1_adelante")

        # El LIDAR no cubre la parte trasera en este chasis (ver nota de
        # diseño al inicio del módulo) -- se pide confirmación visual del
        # operador antes de mover el robot hacia atrás. Si el E-stop se
        # dispara durante esta espera, _drive() detectará self._e_stop y
        # devolverá el resultado "omitido" sin intentar moverse.
        self._confirm_rear_clear()
        i2, t2, ok2, s2e, s2 = self._drive(-DIST_X, "x", vx=-VX_REF, seg_name="P1_atras")

        rel_enc = self._pose_rel_enc()
        err_f_enc = math.hypot(rel_enc.x, rel_enc.y)
        rel = self._pose_rel()
        err_f = math.hypot(rel.x, rel.y)

        ITAE_REF, TIME_REF = 0.05, 2*DIST_X/VX_REF
        cost = 0.60*(i1+i2)/ITAE_REF + 0.30*(t1+t2)/TIME_REF + 0.10*err_f_enc/POS_TOL
        if not ok1 or not ok2: cost += PENALTY_TO
        self.get_logger().info(f"   ITAE={i1+i2:.4f} cost={cost:.4f} err_f_encoder={err_f_enc:.3f}m")
        return cost, [s1e, s2e], [s1, s2], {"encoder_m": err_f_enc, "rf2o_m": err_f}

    def _run_test2(self):
        self.get_logger().info("── P2: rotación pura +90°/-90° ──")
        self._manual_reset()
        e1, ok1, s1e, t1, s1 = self._rotate(+ROT_ANGLE, seg_name="P2_giro_horario")
        e2, ok2, s2e, t2, s2 = self._rotate(-ROT_ANGLE, seg_name="P2_giro_antihorario")

        rel_enc = self._pose_rel_enc()
        err_f_enc = math.hypot(rel_enc.x, rel_enc.y)
        rel = self._pose_rel()
        err_f = math.hypot(rel.x, rel.y)

        TIME_REF = 2*ROT_ANGLE/WZ_REF
        cost = 0.55*(e1+e2)/(2*YAW_TOL) + 0.25*(t1+t2)/TIME_REF + 0.20*err_f_enc/POS_TOL
        if not ok1 or not ok2: cost += PENALTY_TO
        self.get_logger().info(f"   err_yaw={math.degrees(e1+e2):.1f}° cost={cost:.4f} err_f_encoder={err_f_enc:.3f}m")
        return cost, [s1e, s2e], [s1, s2], {"encoder_m": err_f_enc, "rf2o_m": err_f}

    def _run_test3(self):
        self.get_logger().info("── P3: avance + giro + regreso ──")
        self._manual_reset()
        i1, t1, ok1, s1e, s1 = self._drive(DIST_X, "x", vx=+VX_REF, seg_name="P3_adelante")
        ey, okr, sre, tr, sr = self._rotate(-ROT_ANGLE, seg_name="P3_giro")
        i2, t2, ok2, s2e, s2 = self._drive(DIST_RETURN, "x", vx=+VX_REF, seg_name="P3_regreso")

        rel_enc = self._pose_rel_enc()
        err_f_enc = math.hypot(rel_enc.x, rel_enc.y)
        rel = self._pose_rel()
        err_f = math.hypot(rel.x, rel.y)

        ITAE_REF = 0.08
        TIME_REF = (DIST_X/VX_REF) + (ROT_ANGLE/WZ_REF) + (DIST_RETURN/VX_REF)
        cost = (0.45*(i1+i2)/ITAE_REF + 0.20*(t1+t2)/TIME_REF + 0.15*ey/YAW_TOL + 0.20*err_f_enc/POS_TOL)
        if not ok1 or not ok2 or not okr: cost += PENALTY_TO
        self.get_logger().info(f"   err_yaw={math.degrees(ey):.1f}° cost={cost:.4f} err_f_encoder={err_f_enc:.3f}m")
        return cost, [s1e, sre, s2e], [s1, sr, s2], {"encoder_m": err_f_enc, "rf2o_m": err_f}

    def run_once(self, kp, ki, kd):
        self._set_pid(kp, ki, kd)
        time.sleep(0.3)
        self._start_arm()
        time.sleep(0.5)

        # FIX #4 — cada prueba obtiene su propia lista/SegmentLog
        # independiente en vez de aliasar el mismo objeto tres veces
        # (segs1e = segs2e = segs3e = [...] compartía la MISMA lista).
        c1 = c2 = c3 = EMERGENCY_ABORT_COST
        segs1e, segs2e, segs3e = ([SegmentLog(name="E-STOP")],
                                   [SegmentLog(name="E-STOP")],
                                   [SegmentLog(name="E-STOP")])
        segs1, segs2, segs3 = ([SegmentLog(name="E-STOP")],
                                [SegmentLog(name="E-STOP")],
                                [SegmentLog(name="E-STOP")])
        err1 = err2 = err3 = {"encoder_m": 0.0, "rf2o_m": 0.0}

        try:
            if not self._e_stop:
                c1, segs1e, segs1, err1 = self._run_test1()
            if not self._e_stop:
                c2, segs2e, segs2, err2 = self._run_test2()
            if not self._e_stop:
                c3, segs3e, segs3, err3 = self._run_test3()
        finally:
            self._stop_arm()
            if self._e_stop:
                self.get_logger().error("🛑 CANCELANDO BATERÍA: VARIABLES GUARDADAS DE EMERGENCIA 🛑")

        fitness = W1*c1 + W2*c2 + W3*c3
        return fitness, (c1, c2, c3), (segs1e, segs2e, segs3e), (segs1, segs2, segs3), (err1, err2, err3)


def _seg_to_dict(s: SegmentLog) -> dict:
    if not s: return {}
    return {"name": s.name, "t": s.t, "vx_ref": s.vx_ref, "vy_ref": s.vy_ref, "wz_ref": s.wz_ref, "vx_real": s.vx_real, "vy_real": s.vy_real, "wz_real": s.wz_real, "pos_err": s.pos_err, "x_ref": s.x_ref, "y_ref": s.y_ref, "yaw_ref": s.yaw_ref, "x_real": s.x_real, "y_real": s.y_real, "yaw_real": s.yaw_real}


def main(args=None):
    rclpy.init(args=args)
    node = HardwareBatteryEvaluatorEnc()

    node.declare_parameter("label", "unnamed")
    node.declare_parameter("kp", 1.0)
    node.declare_parameter("ki", 0.0)
    node.declare_parameter("kd", 0.0)
    node.declare_parameter("reps", 5)

    label = node.get_parameter("label").value
    kp = node.get_parameter("kp").value
    ki = node.get_parameter("ki").value
    kd = node.get_parameter("kd").value
    reps = int(node.get_parameter("reps").value)

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        node.get_logger().info(f"=== Batería hardware — {label} — Kp={kp} Ki={ki} Kd={kd} — {reps} reps — CONTROL X ENCODER ===")
        runs = []
        for r in range(reps):
            if node._e_stop:
                node.get_logger().error("Ejecución global abortada por seguridad.")
                break
            node.get_logger().info(f"--- Repetición {r+1}/{reps} ---")
            fitness, costs, segs_enc, segs, errs = node.run_once(kp, ki, kd)
            runs.append({
                "rep": r, "fitness": fitness, "cost_p1": costs[0], "cost_p2": costs[1], "cost_p3": costs[2],
                "final_position_error_m": {"test1": errs[0], "test2": errs[1], "test3": errs[2]},
                "segments_encoder": {"test1": [_seg_to_dict(s) for s in segs_enc[0]], "test2": [_seg_to_dict(s) for s in segs_enc[1]], "test3": [_seg_to_dict(s) for s in segs_enc[2]]},
                "segments_rf2o": {"test1": [_seg_to_dict(s) for s in segs[0]], "test2": [_seg_to_dict(s) for s in segs[1]], "test3": [_seg_to_dict(s) for s in segs[2]]},
            })

        if runs:
            fitnesses = [r["fitness"] for r in runs]
            mean_fit = sum(fitnesses) / len(fitnesses)
            std_fit = (sum((f-mean_fit)**2 for f in fitnesses) / len(fitnesses)) ** 0.5

            def _mean_err(src, tst):
                vals = [r["final_position_error_m"][tst][src] for r in runs]
                return sum(vals) / len(vals)

            err_sum = {t: {"encoder_mean_m": _mean_err("encoder_m", t), "rf2o_mean_m": _mean_err("rf2o_m", t)} for t in ("test1", "test2", "test3")}

            results = {
                "label": label, "kp": kp, "ki": ki, "kd": kd, "reps": reps, "mean_fitness": mean_fit, "std_fitness": std_fit,
                "final_position_error_summary": err_sum,
                "note": "Control usa exclusivamente /odom_encoder. LIDAR pasivo (solo registro + E-stop frontal/trasero). E-Stop integrado.",
                "runs": runs,
            }
            out_path = os.path.abspath(f"{label}_{OUT_JSON}")
            with open(out_path, "w") as f: json.dump(results, f, indent=2)
            node.get_logger().info(f"=== Resultados guardados en {out_path} ===")
    finally:
        # FIX #3b — restauración de terminal garantizada independientemente
        # de cómo haya terminado el hilo de teclado (join con timeout +
        # el propio finally del hilo como respaldo).
        node.stop_keyboard_monitor()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()