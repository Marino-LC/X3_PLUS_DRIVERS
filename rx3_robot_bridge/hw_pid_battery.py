#!/usr/bin/env python3
"""
hw_pid_battery.py
==================
Batería de pruebas P1/P2/P3 sobre el ROSMASTER X3 PLUS físico.
Con Paro de Emergencia por colisión (LIDAR) y manual (Ctrl+X).
"""

import os
import math
import time
import subprocess
import json
import threading
import sys
import select
import termios
import tty
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
from sensor_msgs.msg import LaserScan  # <-- NUEVO: Importar LaserScan para el LIDAR

from rx3_robot_bridge.pid_battery_common import (
    DIST_X, DIST_RETURN, ROT_ANGLE, VX_REF, WZ_REF,
    CTRL_DT, SETTLE_TIME, TIMEOUT_MOVE, TIMEOUT_ROT, POS_TOL, YAW_TOL,
    MAX_POS_ERROR_ABORT, W1, W2, W3, PENALTY_TO,
    ARM_JOINTS, GRIP_JOINTS, ARM_TOPIC, GRIP_TOPIC, ARM_MOVE_DUR,
    ARM_HOLD_DUR, ARM_HOME, ARM_PICK_LEFT, ARM_PICK_RIGHT,
    ARM_CHOREOGRAPHY, GRIP_OPEN, GRIP_CLOSED, SegmentLog, Pose2D,
)

BRIDGE_PARAM_SERVICE = "/rosmaster_bridge_node/set_parameters"
OUT_JSON = "hw_battery_results.json"

# Configuración del LIDAR para el E-Stop
MIN_SAFE_DIST = 0.25  # Distancia mínima permitida (en metros)
FRONT_CONE_DEG = 30.0 # Checar +- 30 grados frente al robot
ROBOT_FOOTPRINT_RADIUS = 0.18 
MIN_OBSTACLE_RAYS = 4

class HardwareBatteryEvaluator(Node):
    def __init__(self):
        super().__init__("hw_pid_battery_evaluator")
        cbg = ReentrantCallbackGroup()

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._arm_pub = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self._grip_pub = self.create_publisher(JointTrajectory, GRIP_TOPIC, 10)

        self._pid_cli = self.create_client(
            SetParameters, BRIDGE_PARAM_SERVICE, callback_group=cbg)

        self._lock = threading.Lock()
        self._pose = Pose2D()
        self._origin = Pose2D()
        self._current_vref = (0.0, 0.0, 0.0)
        self._itae_target = Pose2D()
        self._yaw_ref_live = 0.0
        self._measuring = False
        self._seg_log = None
        self._itae_accum = 0.0
        self._start_eval_t = 0.0
        self._last_odom_t = 0.0
        self._arm_active = False
        
        # <-- NUEVO: Bandera de paro de emergencia
        self._e_stop = False 

        # Suscripción a Odometría
        self.create_subscription(
            Odometry, "/odom", self._odom_cb, qos_profile_sensor_data,
            callback_group=cbg)

        # <-- NUEVO: Suscripción a LIDAR
        self.create_subscription(
            LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data,
            callback_group=cbg)

        # <-- NUEVO: Hilo para escuchar teclado (Ctrl+X)
        self._kb_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self._kb_thread.start()

        self.get_logger().info("HardwareBatteryEvaluator listo.")
        self.get_logger().info("==========================================")
        self.get_logger().info(" PRESIONA [Ctrl+X] PARA PARO DE EMERGENCIA")
        self.get_logger().info("==========================================")

    # <-- NUEVO: Función para detonar el Paro de Emergencia
    def _trigger_e_stop(self, reason: str):
        with self._lock:
            if self._e_stop:
                return # Ya estaba activado
            self._e_stop = True
            
        self.get_logger().error(f"\n PARO DE EMERGENCIA ACTIVADO: {reason} \n")
        
        # Detener motores de base inmediatamente
        msg = Twist()
        self._cmd_pub.publish(msg)
        
        # Detener el brazo si estaba activo
        self._arm_active = False

    # <-- NUEVO: Callback del LIDAR
    # ── Callback del LIDAR Modificado ──────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        if self._e_stop:
            return # Ignorar si ya estamos en paro
            
        obstacle_rays_count = 0
        
        # Revisar el escaneo frontal
        for i, r in enumerate(msg.ranges):
            # 1er Filtro: Ignorar lecturas nulas, infinitas o dentro de la "huella" del robot (el brazo)
            if math.isinf(r) or math.isnan(r) or r < ROBOT_FOOTPRINT_RADIUS:
                continue
                
            angle = msg.angle_min + i * msg.angle_increment
            # Normalizar el ángulo a [-pi, pi] (0 es el frente del robot)
            angle = (angle + math.pi) % (2 * math.pi) - math.pi
            
            # Checar si el ángulo está dentro del cono frontal definido
            if abs(angle) < math.radians(FRONT_CONE_DEG):
                if r < MIN_SAFE_DIST:
                    obstacle_rays_count += 1
                    
                    # 2do Filtro: ¿El objeto es lo suficientemente grueso para ser real?
                    if obstacle_rays_count >= MIN_OBSTACLE_RAYS:
                        self._trigger_e_stop(f"Colisión inminente (Obstáculo detectado a {r:.2f}m)")
                        break

    # <-- NUEVO: Hilo esclucha del teclado
    def _keyboard_listener(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok() and not self._e_stop:
                # Esperar entrada por 0.1s para no bloquear el hilo infinitamente
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    c = sys.stdin.read(1)
                    if c == '\x18': # Código ASCII para Ctrl+X
                        self._trigger_e_stop("Activado manualmente (Ctrl+X)")
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    # ── Odometría ──────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z

        with self._lock:
            self._pose = Pose2D(msg.pose.pose.position.x,
                                msg.pose.pose.position.y, yaw)
            if self._measuring:
                now = time.time()
                dt = max(now - self._last_odom_t, 0.001)
                self._last_odom_t = now

                ex = self._itae_target.x - self._pose.x
                ey = self._itae_target.y - self._pose.y
                err = math.hypot(ex, ey)
                t_real = now - self._start_eval_t
                self._itae_accum += t_real * err * dt

                if self._seg_log is not None:
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

    def _fix_origin(self):
        with self._lock:
            self._origin = self._pose.copy()

    def _pose_rel(self) -> Pose2D:
        with self._lock:
            return Pose2D(self._pose.x - self._origin.x,
                          self._pose.y - self._origin.y,
                          self._pose.yaw)

    def _start_itae(self, target: Pose2D, seg_log: SegmentLog = None):
        with self._lock:
            self._itae_target = target.copy()
            self._itae_accum = 0.0
            self._start_eval_t = time.time()
            self._last_odom_t = time.time()
            self._measuring = True
            self._seg_log = seg_log
            self._yaw_ref_live = target.yaw

    def _stop_itae(self) -> float:
        with self._lock:
            self._measuring = False
            self._seg_log = None
            return self._itae_accum

    def _hard_restart_rf2o(self):
        """
        Fallback SOLO si es físicamente imposible deslizar el robot sin
        levantarlo. Reinicia el PROCESO de rf2o_laser_odometry_node con 
        cierre limpio (SIGINT) y blindaje de tópicos.
        """
        self.get_logger().warn(
            "Reiniciando proceso de rf2o_laser_odometry_node "
            "(el robot fue levantado, no solo deslizado)..."
        )
        
        # 1. Matamos el proceso con SIGINT (Ctrl+C simulado) en lugar de un kill abrupto.
        # Vital para que FastDDS avise al YDLidar que se desconectó y libere la memoria compartida.
        subprocess.run(["pkill", "-INT", "-f", "rf2o_laser_odometry"])
        
        # 2. Le damos 2.0 segundos completos al núcleo DDS para purgar sus cachés
        time.sleep(2.0) 
        
        import time as time_module
        nuevo_nombre = f"rf2o_recovery_{int(time_module.time())}"
        
        self.get_logger().info(f"Levantando nueva instancia como: {nuevo_nombre}")
        
        # 3. Lanzamos con TODAS las combinaciones posibles de remapeo y parámetros
        subprocess.Popen([
            "ros2", "run", "rf2o_laser_odometry", "rf2o_laser_odometry_node",
            "--ros-args",
            "-r", f"__node:={nuevo_nombre}",
            "-r", "scan:=/scan",               # Remapeo blindado A
            "-r", "laser_scan:=/scan",         # Remapeo blindado B
            "-p", "laser_scan_topic:=/scan",   # Parámetro estándar de la rama principal
            "-p", "scan_topic:=/scan",         # Variante de parámetro (otras ramas)
            "-p", "odom_topic:=/odom",
            "-p", "publish_tf:=true",
            "-p", "base_frame_id:=base_footprint",
            "-p", "odom_frame_id:=odom",
            "-p", "freq:=12.0",
        ])
        
        self.get_logger().info("Esperando sincronización de /scan al nuevo nodo de odometría...")
        time.sleep(5.0)

    def _verify_odom_stable(self, timeout=1.0, max_drift=0.05):
        """
        Confirma que /odom no sigue derivando con el robot inmóvil.
        Si falla, es señal de que rf2o no convergió limpiamente.
        """
        p0 = self._get_pose()
        time.sleep(timeout)
        p1 = self._get_pose()
        
        # Calculamos la distancia Euclídea entre la pose 0 y la pose 1
        # Asegúrate de usar la lógica que coincida con cómo guardas la pose
        import math
        drift = math.hypot(p1.x - p0.x, p1.y - p0.y)

        if drift > max_drift:
            self.get_logger().warn(
                f"[odom] Deriva de {drift:.3f}m detectada con el robot supuestamente "
                f"quieto — Posible falla de convergencia en rf2o. "
                f"Considera reintentar el reposicionamiento."
            )
            return False
            
        self.get_logger().info("[odom] Odometría estable confirmada.")
        return True

    def _manual_reset(self):
        """
        Reposicionamiento puramente matemático. 
        Ignoramos el salto del LIDAR absorbiéndolo como el nuevo origen.
        """
        self.get_logger().info("=== REPOSICIONAMIENTO MANUAL ===")
        self.get_logger().info("Levanta el robot y acomódalo en su marca de inicio.")
        
        # Pausamos el flujo hasta que el operador confirme
        input("Presiona ENTER cuando el robot esté en el piso y tus piernas estén lejos del LIDAR...")
        
        self.get_logger().info("Esperando 3 segundos a que el LIDAR estabilice el entorno...")
        time.sleep(3.0) 
        
        # Validamos que rf2o ya dejó de arrastrar el error
        estable = self._verify_odom_stable()
        while not estable:
            self.get_logger().warn("El LIDAR sigue detectando movimiento fantasma. Aléjate un poco más del robot.")
            input("Presiona ENTER para reintentar la estabilización...")
            time.sleep(2.0)
            estable = self._verify_odom_stable()
            
        # El truco maestro: Sobreescribimos el origen.
        # Cualquier salto gigante que haya dado el LIDAR al levantarlo queda anulado matemáticamente.
        self._fix_origin()
        
        self.get_logger().info("=== ORIGEN ACTUALIZADO MATEMÁTICAMENTE. REANUDANDO PRUEBA ===")

    # ── Primitivas de movimiento ──────────────────────────────────────────
    def _send(self, vx=0.0, vy=0.0, wz=0.0):
        # <-- MODIFICADO: Bloquear envío de velocidades si hay paro
        if self._e_stop:
            vx, vy, wz = 0.0, 0.0, 0.0
            
        with self._lock:
            self._current_vref = (vx, vy, wz)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(wz)
        self._cmd_pub.publish(msg)

    def _stop(self):
        self._send()

    def _drive(self, dist_m, axis, vx=0.0, vy=0.0,
               timeout=TIMEOUT_MOVE, seg_name=""):
        p0 = self._get_pose()
        if axis == "x":
            target = Pose2D(p0.x + math.cos(p0.yaw)*dist_m,
                            p0.y + math.sin(p0.yaw)*dist_m, p0.yaw)
        else:
            target = Pose2D(p0.x - math.sin(p0.yaw)*dist_m,
                            p0.y + math.cos(p0.yaw)*dist_m, p0.yaw)

        slog = SegmentLog(name=seg_name)
        self._start_itae(target, slog)
        t0, ok, aborted = time.time(), False, False

        while time.time() - t0 < timeout:
            # <-- MODIFICADO: Romper bucle si hay paro de emergencia
            if self._e_stop:
                aborted = True
                break
                
            p = self._get_pose()
            dx, dy = p.x - p0.x, p.y - p0.y
            traveled = (dx*math.cos(p0.yaw) + dy*math.sin(p0.yaw)
                        if axis == "x"
                        else -dx*math.sin(p0.yaw) + dy*math.cos(p0.yaw))
            lateral_dev = math.hypot(dx, dy) - abs(traveled)

            if abs(lateral_dev) > MAX_POS_ERROR_ABORT:
                aborted = True
                break
            if abs(traveled) >= abs(dist_m) - POS_TOL:
                ok = True
                break
            if abs(traveled) > abs(dist_m) + 0.12:
                break

            self._send(vx=vx, vy=vy)
            time.sleep(CTRL_DT)

        self._stop()
        itae = self._stop_itae()
        elapsed = time.time() - t0
        time.sleep(SETTLE_TIME)
        return itae, elapsed, (ok and not aborted), slog

    def _rotate(self, angle_rad, timeout=TIMEOUT_ROT, seg_name=""):
        p0 = self._get_pose()
        goal_yaw = p0.yaw + angle_rad
        sign = math.copysign(1.0, angle_rad)
        t0, ok = time.time(), False

        target = Pose2D(p0.x, p0.y, goal_yaw)
        slog = SegmentLog(name=seg_name) if seg_name else None
        self._start_itae(target, slog)

        while time.time() - t0 < timeout:
            # <-- MODIFICADO: Romper bucle si hay paro de emergencia
            if self._e_stop:
                break
                
            p = self._get_pose()
            diff = (goal_yaw - p.yaw + math.pi) % (2*math.pi) - math.pi
            if abs(diff) <= YAW_TOL:
                ok = True
                break
            wz = sign * max(0.15, min(WZ_REF, abs(diff)*1.5))
            t_rel = time.time() - t0
            est_total = max(abs(angle_rad)/WZ_REF, 1e-3)
            frac = min(1.0, t_rel/est_total)
            with self._lock:
                self._yaw_ref_live = p0.yaw + angle_rad*frac
                self._current_vref = (0.0, 0.0, wz)
            self._send(wz=wz)
            time.sleep(CTRL_DT)

        self._stop()
        with self._lock:
            self._yaw_ref_live = goal_yaw
        self._stop_itae()
        p = self._get_pose()
        diff = (goal_yaw - p.yaw + math.pi) % (2*math.pi) - math.pi
        elapsed = time.time() - t0
        time.sleep(SETTLE_TIME)
        return abs(diff), ok, slog, elapsed

    # ── PID ────────────────────────────────────────────────────────────────
    def _set_pid(self, kp, ki, kd):
        if not self._pid_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{BRIDGE_PARAM_SERVICE} no disponible")
            return

        def _p(n, v):
            return Parameter(name=n, value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))

        req = SetParameters.Request()
        req.parameters = [_p("kp", kp), _p("ki", ki), _p("kd", kd)]
        fut = self._pid_cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time()-t0 < 3.0:
            time.sleep(0.05)

    # ── Brazo ──────────────────────────────────────────────────────────────
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
        while self._arm_active and not self._e_stop and time.time()-t0 < duration_sec:
            time.sleep(0.05)
        return self._arm_active and not self._e_stop

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
        if hasattr(self, "_arm_thread") and self._arm_thread.is_alive():
            self._arm_thread.join(timeout=ARM_MOVE_DUR + 1.0)

    # ── Pruebas ────────────────────────────────────────────────────────────
    def _run_test1(self):
        if self._e_stop: return 0.0, []
        self.get_logger().info("── P1: línea recta adelante-atrás ──")
        self._manual_reset()
        if self._e_stop: return 0.0, []
        try:
            i1, t1, ok1, s1 = self._drive(DIST_X, "x", vx=+VX_REF, seg_name="P1_adelante")
            i2, t2, ok2, s2 = self._drive(-DIST_X, "x", vx=-VX_REF, seg_name="P1_atras")
        finally:
            pass
        if self._e_stop: return 0.0, []
        rel = self._pose_rel()
        err_f = math.hypot(rel.x, rel.y)
        ITAE_REF, TIME_REF = 0.05, 2*DIST_X/VX_REF
        cost = 0.60*(i1+i2)/ITAE_REF + 0.30*(t1+t2)/TIME_REF + 0.10*err_f/POS_TOL
        if not ok1 or not ok2: cost += PENALTY_TO
        self.get_logger().info(f"   ITAE={i1+i2:.4f} cost={cost:.4f}")
        return cost, [s1, s2]

    def _run_test2(self):
        if self._e_stop: return 0.0, []
        self.get_logger().info("── P2: rotación pura +90°/-90° ──")
        self._manual_reset()
        if self._e_stop: return 0.0, []
        try:
            e1, ok1, s1, t1 = self._rotate(+ROT_ANGLE, seg_name="P2_giro_horario")
            e2, ok2, s2, t2 = self._rotate(-ROT_ANGLE, seg_name="P2_giro_antihorario")
        finally:
            pass
        if self._e_stop: return 0.0, []
        rel = self._pose_rel()
        err_f = math.hypot(rel.x, rel.y)
        TIME_REF = 2*ROT_ANGLE/WZ_REF
        cost = 0.55*(e1+e2)/(2*YAW_TOL) + 0.25*(t1+t2)/TIME_REF + 0.20*err_f/POS_TOL
        if not ok1 or not ok2: cost += PENALTY_TO
        self.get_logger().info(f"   err_yaw={math.degrees(e1+e2):.1f}° cost={cost:.4f}")
        return cost, [s1, s2]

    def _run_test3(self):
        if self._e_stop: return 0.0, []
        self.get_logger().info("── P3: avance + giro + regreso ──")
        self._manual_reset()
        if self._e_stop: return 0.0, []
        try:
            i1, t1, ok1, s1 = self._drive(DIST_X, "x", vx=+VX_REF, seg_name="P3_adelante")
            ey, okr, sr, tr = self._rotate(-ROT_ANGLE, seg_name="P3_giro")
            i2, t2, ok2, s2 = self._drive(DIST_RETURN, "x", vx=+VX_REF, seg_name="P3_regreso")
        finally:
            pass
        if self._e_stop: return 0.0, []
        rel = self._pose_rel()
        err_f = math.hypot(rel.x, rel.y)
        ITAE_REF = 0.08
        TIME_REF = (DIST_X/VX_REF) + (ROT_ANGLE/WZ_REF) + (DIST_RETURN/VX_REF)
        cost = (0.45*(i1+i2)/ITAE_REF + 0.20*(t1+t2)/TIME_REF
                + 0.15*ey/YAW_TOL + 0.20*err_f/POS_TOL)
        if not ok1 or not ok2 or not okr: cost += PENALTY_TO
        self.get_logger().info(f"   err_yaw={math.degrees(ey):.1f}° cost={cost:.4f}")
        return cost, [s1, sr, s2]

    def run_once(self, kp, ki, kd):
        self._set_pid(kp, ki, kd)
        time.sleep(0.3)
        self._start_arm()
        time.sleep(0.5)
        try:
            c1, segs1 = self._run_test1()
            c2, segs2 = self._run_test2()
            c3, segs3 = self._run_test3()
        finally:
            self._stop_arm()
            
        if self._e_stop:
            raise Exception("Ejecución interrumpida por E-Stop")
            
        fitness = W1*c1 + W2*c2 + W3*c3
        return fitness, (c1, c2, c3), (segs1, segs2, segs3)


# ══════════════════════════════════════════════════════════════════════════
# Función de módulo (NO método de clase)
# ══════════════════════════════════════════════════════════════════════════
def _seg_to_dict(s: SegmentLog) -> dict:
    return {
        "name": s.name,
        "t": s.t,
        "vx_ref": s.vx_ref,
        "vy_ref": s.vy_ref,
        "wz_ref": s.wz_ref,
        "vx_real": s.vx_real,
        "vy_real": s.vy_real,
        "wz_real": s.wz_real,
        "pos_err": s.pos_err,
        "x_ref": s.x_ref,
        "y_ref": s.y_ref,
        "yaw_ref": s.yaw_ref,
        "x_real": s.x_real,
        "y_real": s.y_real,
        "yaw_real": s.yaw_real,
    }


def main(args=None):
    rclpy.init(args=args)
    node = HardwareBatteryEvaluator()
    
    node.declare_parameter("label", "unnamed")
    node.declare_parameter("kp", 2.534)
    node.declare_parameter("ki", 0.3347)
    node.declare_parameter("kd", 0.7565)
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
        node.get_logger().info(
            f"=== Batería hardware — {label} — Kp={kp} Ki={ki} Kd={kd} — "
            f"{reps} repeticiones ===")
        runs = []
        for r in range(reps):
            if node._e_stop: break # Salir si el paro está activo
            node.get_logger().info(f"--- Repetición {r+1}/{reps} ---")
            
            try:
                fitness, costs, segs = node.run_once(kp, ki, kd)
            except Exception as e:
                node.get_logger().error(str(e))
                break
                
            runs.append({
                "rep": r,
                "fitness": fitness,
                "cost_p1": costs[0],
                "cost_p2": costs[1],
                "cost_p3": costs[2],
                "segments": {
                    "test1": [_seg_to_dict(s) for s in segs[0]],
                    "test2": [_seg_to_dict(s) for s in segs[1]],
                    "test3": [_seg_to_dict(s) for s in segs[2]],
                },
            })

        if runs:
            fitnesses = [r["fitness"] for r in runs]
            mean_fit = sum(fitnesses) / len(fitnesses)
            std_fit = (sum((f-mean_fit)**2 for f in fitnesses) / len(fitnesses)) ** 0.5

            results = {
                "label": label,
                "kp": kp,
                "ki": ki,
                "kd": kd,
                "reps": reps,
                "mean_fitness": mean_fit,
                "std_fitness": std_fit,
                "runs": runs,
            }
            out_path = os.path.abspath(f"{label}_{OUT_JSON}")
            with open(out_path, "w") as f:
                json.dump(results, f, indent=2)
            node.get_logger().info(
                f"=== {label}: fitness medio={mean_fit:.4f} ± {std_fit:.4f} — "
                f"guardado en {out_path} ===")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()