#!/usr/bin/env python3
"""
hw_pid_battery.py
==================
Batería de pruebas P1/P2/P3 sobre el ROSMASTER X3 PLUS físico.

ACTUALIZACIÓN — comparación /odom (rf2o) vs /odom_encoder:
  Este script ahora suscribe TAMBIÉN a /odom_encoder (publicado por
  rosmaster_bridge_node.py) y registra una serie temporal paralela e
  independiente para cada segmento de cada prueba, con el mismo reloj
  (_start_eval_t) que la serie basada en rf2o.

  IMPORTANTE — el control sigue basándose exclusivamente en /odom (rf2o):
  _drive()/_rotate() deciden cuándo detenerse, si hubo overshoot, etc.
  usando self._pose (rf2o), exactamente igual que antes. /odom_encoder
  se registra en paralelo SOLO con fines de comparación/evaluación
  posterior — no participa en ninguna decisión de control durante la
  prueba. Esto evita contaminar el comportamiento ya validado del lazo
  de control con una fuente de odometría todavía no validada como
  confiable para ese propósito.

  Cada objetivo de posición (target) se calcula de forma independiente
  para cada frame: target (rf2o) se calcula desde self._pose al inicio
  del segmento, target_enc se calcula desde self._enc_pose al inicio del
  MISMO segmento. Así, el error de posición registrado para cada fuente
  es el error respecto a su propia estimación de "inicio de segmento",
  y ambas series son directamente comparables en el mismo instante t_real.
"""

import os
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
OUT_JSON = "hw_battery_results.json"


class HardwareBatteryEvaluator(Node):
    def __init__(self):
        super().__init__("hw_pid_battery_evaluator")
        cbg = ReentrantCallbackGroup()

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._arm_pub = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self._grip_pub = self.create_publisher(JointTrajectory, GRIP_TOPIC, 10)

        self._pid_cli = self.create_client(
            SetParameters, BRIDGE_PARAM_SERVICE, callback_group=cbg)

        # Cliente para resetear /odom_encoder a (0,0,0) en cada reposicionamiento
        # manual — mantiene los valores absolutos acotados entre repeticiones,
        # igual que ya se hacía (implícitamente, vía _fix_origin) para rf2o.
        self._enc_reset_cli = self.create_client(
            Empty, BRIDGE_ENC_RESET_SERVICE, callback_group=cbg)

        self._lock = threading.Lock()

        # ── Estado rf2o (fuente de control, sin cambios de comportamiento) ──
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

        # ── Estado encoder (solo registro paralelo, NUNCA controla) ─────────
        self._enc_pose = Pose2D()
        self._origin_enc = Pose2D()
        self._itae_target_enc = Pose2D()
        self._yaw_ref_live_enc = 0.0
        self._seg_log_enc = None
        self._last_enc_odom_t = 0.0

        self._arm_active = False

        self.create_subscription(
            Odometry, "/odom", self._odom_cb, qos_profile_sensor_data,
            callback_group=cbg)
        self.create_subscription(
            Odometry, "/odom_encoder", self._encoder_odom_cb, qos_profile_sensor_data,
            callback_group=cbg)

        self.get_logger().info(
            "HardwareBatteryEvaluator listo — registrando /odom (control) "
            "y /odom_encoder (comparación paralela, no controla).")

    # ── Odometría rf2o — FUENTE DE CONTROL, sin cambios de comportamiento ──
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

    # ── Odometría por encoders — SOLO REGISTRO, no participa en control ────
    def _encoder_odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z

        with self._lock:
            self._enc_pose = Pose2D(msg.pose.pose.position.x,
                                    msg.pose.pose.position.y, yaw)
            if self._measuring and self._seg_log_enc is not None:
                now = time.time()
                dt = max(now - self._last_enc_odom_t, 0.001)
                self._last_enc_odom_t = now

                ex = self._itae_target_enc.x - self._enc_pose.x
                ey = self._itae_target_enc.y - self._enc_pose.y
                err = math.hypot(ex, ey)
                t_real = now - self._start_eval_t   # mismo reloj que rf2o

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

    def _start_itae(self, target: Pose2D, seg_log: SegmentLog = None,
                     target_enc: Pose2D = None, seg_log_enc: SegmentLog = None):
        with self._lock:
            self._itae_target = target.copy()
            self._itae_accum = 0.0
            self._start_eval_t = time.time()
            self._last_odom_t = time.time()
            self._measuring = True
            self._seg_log = seg_log
            self._yaw_ref_live = target.yaw

            # target_enc puede ser None si el caller no tiene aún una pose de
            # encoder válida (p. ej. antes de la primera trama) — se usa
            # target (rf2o) como fallback para no romper el registro, aunque
            # en ese caso el error relativo de esa serie no sería confiable.
            t_enc = target_enc if target_enc is not None else target
            self._itae_target_enc = t_enc.copy()
            self._seg_log_enc = seg_log_enc
            self._yaw_ref_live_enc = t_enc.yaw
            self._last_enc_odom_t = time.time()

    def _stop_itae(self) -> float:
        with self._lock:
            self._measuring = False
            self._seg_log = None
            self._seg_log_enc = None
            return self._itae_accum

    # ── Reset manual ──────────────────────────────────────────────────────
    def _reset_encoder_odom(self):
        """Resetea /odom_encoder a (0,0,0) vía el servicio del bridge. Si el
        servicio no está disponible (p. ej. bridge lanzado sin la extensión
        de odometría por encoders), se avisa pero NO se aborta la prueba:
        _fix_origin() sigue haciendo el error relativo correcto aunque los
        valores absolutos de /odom_encoder crezcan sin límite entre
        repeticiones."""
        if not self._enc_reset_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f"Servicio {BRIDGE_ENC_RESET_SERVICE} no disponible — "
                "/odom_encoder no se reinició (el error relativo por "
                "_fix_origin() sigue siendo válido).")
            return
        fut = self._enc_reset_cli.call_async(Empty.Request())
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 2.0:
            time.sleep(0.05)
        if fut.done():
            self.get_logger().info("Odometría por encoders reseteada a (0,0,0).")
        else:
            self.get_logger().warn("Timeout esperando reset de /odom_encoder.")

    def _manual_reset(self):
        self._stop()
        time.sleep(0.2)
        input(
            "\n>>> Reposiciona el robot manualmente sobre la marca de "
            "origen (orientación incluida) y presiona ENTER para continuar..."
        )
        time.sleep(0.5)
        self._reset_encoder_odom()
        time.sleep(0.15)   # dejar llegar al menos una trama fresca de /odom_encoder
        self._fix_origin()

    # ── Cálculo de objetivo, compartido entre frame rf2o y frame encoder ───
    @staticmethod
    def _compute_target(p0: Pose2D, dist_m: float, axis: str) -> Pose2D:
        if axis == "x":
            return Pose2D(p0.x + math.cos(p0.yaw)*dist_m,
                          p0.y + math.sin(p0.yaw)*dist_m, p0.yaw)
        else:
            return Pose2D(p0.x - math.sin(p0.yaw)*dist_m,
                          p0.y + math.cos(p0.yaw)*dist_m, p0.yaw)

    # ── Primitivas de movimiento ────────────────────────────────────────
    def _send(self, vx=0.0, vy=0.0, wz=0.0):
        with self._lock:
            self._current_vref = (vx, vy, wz)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(wz)
        self._cmd_pub.publish(msg)

    def _stop(self):
        self._send()

    def _drive(self, dist_m, axis, vx=0.0, vy=0.0,
               timeout=TIMEOUT_MOVE, seg_name=""):
        # CONTROL: exclusivamente sobre p0/pose rf2o, sin cambios.
        p0 = self._get_pose()
        target = self._compute_target(p0, dist_m, axis)

        # REGISTRO PARALELO: mismo cálculo, pero desde el frame de encoders.
        p0_enc = self._get_pose_enc()
        target_enc = self._compute_target(p0_enc, dist_m, axis)

        slog = SegmentLog(name=seg_name)
        slog_enc = SegmentLog(name=seg_name + "_enc")
        self._start_itae(target, slog, target_enc, slog_enc)
        t0, ok, aborted = time.time(), False, False

        while time.time() - t0 < timeout:
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
        return itae, elapsed, (ok and not aborted), slog, slog_enc

    def _rotate(self, angle_rad, timeout=TIMEOUT_ROT, seg_name=""):
        p0 = self._get_pose()
        goal_yaw = p0.yaw + angle_rad
        sign = math.copysign(1.0, angle_rad)
        t0, ok = time.time(), False

        target = Pose2D(p0.x, p0.y, goal_yaw)

        p0_enc = self._get_pose_enc()
        goal_yaw_enc = p0_enc.yaw + angle_rad
        target_enc = Pose2D(p0_enc.x, p0_enc.y, goal_yaw_enc)

        slog = SegmentLog(name=seg_name) if seg_name else None
        slog_enc = SegmentLog(name=seg_name + "_enc") if seg_name else None
        self._start_itae(target, slog, target_enc, slog_enc)

        while time.time() - t0 < timeout:
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
                self._yaw_ref_live_enc = p0_enc.yaw + angle_rad*frac
                self._current_vref = (0.0, 0.0, wz)
            self._send(wz=wz)
            time.sleep(CTRL_DT)

        self._stop()
        with self._lock:
            self._yaw_ref_live = goal_yaw
            self._yaw_ref_live_enc = goal_yaw_enc
        self._stop_itae()
        p = self._get_pose()
        diff = (goal_yaw - p.yaw + math.pi) % (2*math.pi) - math.pi
        elapsed = time.time() - t0
        time.sleep(SETTLE_TIME)
        return abs(diff), ok, slog, elapsed, slog_enc

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
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        pt.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        msg.points.append(pt)
        self._arm_pub.publish(msg)

    def _send_grip(self, position, duration_sec=ARM_MOVE_DUR):
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
            time.sleep(0.05)
        return self._arm_active

    def _arm_loop(self):
        side = "left"
        while self._arm_active:
            pick = ARM_PICK_LEFT if side == "left" else ARM_PICK_RIGHT
            place = ARM_PICK_RIGHT if side == "left" else ARM_PICK_LEFT
            self._send_arm(pick)
            self._send_grip(GRIP_OPEN)
            if not self._wait_arm(ARM_MOVE_DUR + ARM_HOLD_DUR):
                break
            self._send_grip(GRIP_CLOSED)
            if not self._wait_arm(ARM_HOLD_DUR):
                break
            self._send_arm(ARM_HOME)
            if not self._wait_arm(ARM_MOVE_DUR + ARM_HOLD_DUR):
                break
            self._send_arm(place)
            if not self._wait_arm(ARM_MOVE_DUR + ARM_HOLD_DUR):
                break
            self._send_grip(GRIP_OPEN)
            if not self._wait_arm(ARM_HOLD_DUR):
                break
            self._send_arm(ARM_HOME)
            if not self._wait_arm(ARM_MOVE_DUR + ARM_HOLD_DUR):
                break
            side = "right" if side == "left" else "left"
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

    # ── Pruebas ────────────────────────────────────────────────────────────
    def _run_test1(self):
        self.get_logger().info("── P1: línea recta adelante-atrás ──")
        self._manual_reset()
        i1, t1, ok1, s1, s1e = self._drive(DIST_X, "x", vx=+VX_REF, seg_name="P1_adelante")
        i2, t2, ok2, s2, s2e = self._drive(-DIST_X, "x", vx=-VX_REF, seg_name="P1_atras")

        rel = self._pose_rel()
        err_f = math.hypot(rel.x, rel.y)
        rel_enc = self._pose_rel_enc()
        err_f_enc = math.hypot(rel_enc.x, rel_enc.y)

        ITAE_REF, TIME_REF = 0.05, 2*DIST_X/VX_REF
        cost = 0.60*(i1+i2)/ITAE_REF + 0.30*(t1+t2)/TIME_REF + 0.10*err_f/POS_TOL
        if not ok1 or not ok2:
            cost += PENALTY_TO
        self.get_logger().info(
            f"   ITAE={i1+i2:.4f} cost={cost:.4f} "
            f"err_f_rf2o={err_f:.3f}m err_f_encoder={err_f_enc:.3f}m")
        errs = {"rf2o_m": err_f, "encoder_m": err_f_enc}
        return cost, [s1, s2], [s1e, s2e], errs

    def _run_test2(self):
        self.get_logger().info("── P2: rotación pura +90°/-90° ──")
        self._manual_reset()
        e1, ok1, s1, t1, s1e = self._rotate(+ROT_ANGLE, seg_name="P2_giro_horario")
        e2, ok2, s2, t2, s2e = self._rotate(-ROT_ANGLE, seg_name="P2_giro_antihorario")

        rel = self._pose_rel()
        err_f = math.hypot(rel.x, rel.y)
        rel_enc = self._pose_rel_enc()
        err_f_enc = math.hypot(rel_enc.x, rel_enc.y)

        TIME_REF = 2*ROT_ANGLE/WZ_REF
        cost = 0.55*(e1+e2)/(2*YAW_TOL) + 0.25*(t1+t2)/TIME_REF + 0.20*err_f/POS_TOL
        if not ok1 or not ok2:
            cost += PENALTY_TO
        self.get_logger().info(
            f"   err_yaw={math.degrees(e1+e2):.1f}° cost={cost:.4f} "
            f"err_f_rf2o={err_f:.3f}m err_f_encoder={err_f_enc:.3f}m")
        errs = {"rf2o_m": err_f, "encoder_m": err_f_enc}
        return cost, [s1, s2], [s1e, s2e], errs

    def _run_test3(self):
        self.get_logger().info("── P3: avance + giro + regreso ──")
        self._manual_reset()
        i1, t1, ok1, s1, s1e = self._drive(DIST_X, "x", vx=+VX_REF, seg_name="P3_adelante")
        ey, okr, sr, tr, sre = self._rotate(-ROT_ANGLE, seg_name="P3_giro")
        i2, t2, ok2, s2, s2e = self._drive(DIST_RETURN, "x", vx=+VX_REF, seg_name="P3_regreso")

        rel = self._pose_rel()
        err_f = math.hypot(rel.x, rel.y)
        rel_enc = self._pose_rel_enc()
        err_f_enc = math.hypot(rel_enc.x, rel_enc.y)

        ITAE_REF = 0.08
        TIME_REF = (DIST_X/VX_REF) + (ROT_ANGLE/WZ_REF) + (DIST_RETURN/VX_REF)
        cost = (0.45*(i1+i2)/ITAE_REF + 0.20*(t1+t2)/TIME_REF
                + 0.15*ey/YAW_TOL + 0.20*err_f/POS_TOL)
        if not ok1 or not ok2 or not okr:
            cost += PENALTY_TO
        self.get_logger().info(
            f"   err_yaw={math.degrees(ey):.1f}° cost={cost:.4f} "
            f"err_f_rf2o={err_f:.3f}m err_f_encoder={err_f_enc:.3f}m")
        errs = {"rf2o_m": err_f, "encoder_m": err_f_enc}
        return cost, [s1, sr, s2], [s1e, sre, s2e], errs

    def run_once(self, kp, ki, kd):
        self._set_pid(kp, ki, kd)
        time.sleep(0.3)
        self._start_arm()
        time.sleep(0.5)
        try:
            c1, segs1, segs1e, err1 = self._run_test1()
            c2, segs2, segs2e, err2 = self._run_test2()
            c3, segs3, segs3e, err3 = self._run_test3()
        finally:
            self._stop_arm()
        fitness = W1*c1 + W2*c2 + W3*c3
        return (fitness, (c1, c2, c3),
                (segs1, segs2, segs3), (segs1e, segs2e, segs3e),
                (err1, err2, err3))


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
        node.get_logger().info(
            f"=== Batería hardware — {label} — Kp={kp} Ki={ki} Kd={kd} — "
            f"{reps} repeticiones — comparando /odom vs /odom_encoder ===")
        runs = []
        for r in range(reps):
            node.get_logger().info(f"--- Repetición {r+1}/{reps} ---")
            fitness, costs, segs, segs_enc, errs = node.run_once(kp, ki, kd)
            runs.append({
                "rep": r,
                "fitness": fitness,
                "cost_p1": costs[0],
                "cost_p2": costs[1],
                "cost_p3": costs[2],
                "final_position_error_m": {
                    "test1": errs[0],
                    "test2": errs[1],
                    "test3": errs[2],
                },
                "segments": {
                    "test1": [_seg_to_dict(s) for s in segs[0]],
                    "test2": [_seg_to_dict(s) for s in segs[1]],
                    "test3": [_seg_to_dict(s) for s in segs[2]],
                },
                "segments_encoder": {
                    "test1": [_seg_to_dict(s) for s in segs_enc[0]],
                    "test2": [_seg_to_dict(s) for s in segs_enc[1]],
                    "test3": [_seg_to_dict(s) for s in segs_enc[2]],
                },
            })

        fitnesses = [r["fitness"] for r in runs]
        mean_fit = sum(fitnesses) / len(fitnesses)
        std_fit = (sum((f-mean_fit)**2 for f in fitnesses) / len(fitnesses)) ** 0.5

        # Resumen agregado del error final por fuente, para ver de un
        # vistazo si rf2o o encoder se degrada más entre repeticiones
        # (protocolo de comparación, Fase 3).
        def _mean_err(source_key, test_key):
            vals = [r["final_position_error_m"][test_key][source_key] for r in runs]
            return sum(vals) / len(vals)

        error_summary = {
            test_key: {
                "rf2o_mean_m": _mean_err("rf2o_m", test_key),
                "encoder_mean_m": _mean_err("encoder_m", test_key),
            }
            for test_key in ("test1", "test2", "test3")
        }

        results = {
            "label": label,
            "kp": kp,
            "ki": ki,
            "kd": kd,
            "reps": reps,
            "mean_fitness": mean_fit,
            "std_fitness": std_fit,
            "final_position_error_summary": error_summary,
            "note": "El control usa exclusivamente /odom (rf2o). "
                    "/odom_encoder se registra en paralelo únicamente para "
                    "comparación -- no participa en ninguna decisión de "
                    "_drive()/_rotate().",
            "runs": runs,
        }
        out_path = os.path.abspath(f"{label}_{OUT_JSON}")
        with open(out_path, "w") as f:
            json.dump(results, f, indent=2)
        node.get_logger().info(
            f"=== {label}: fitness medio={mean_fit:.4f} ± {std_fit:.4f} — "
            f"guardado en {out_path} ===")
        node.get_logger().info(f"Resumen error final por fuente: {error_summary}")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()