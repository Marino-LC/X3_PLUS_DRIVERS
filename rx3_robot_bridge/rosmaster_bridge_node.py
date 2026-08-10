#!/usr/bin/env python3
"""
rosmaster_bridge_node.py
========================
Puente ROS 2 para RosBoardDrv (ruedas + brazo).
- Suscribe a /cmd_vel (Twist) para mover la base.
- Suscribe a ARM_TOPIC/GRIP_TOPIC (JointTrajectory) para mover el brazo,
  usando exactamente la coreografía que publique quien esté corriendo
  la batería de pruebas (hw_pid_battery.py) — este nodo NO conoce ni
  hardcodea ninguna pose, solo traduce lo que le llega.
- Convierte radianes del URDF a grados de servo con calibración ajustable.
"""

import time
import threading
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory

from rosboard_drv.rosboard_drv import RosBoardDrv

from rx3_robot_bridge.pid_battery_common import (
    ARM_TOPIC, GRIP_TOPIC, ARM_MOVE_DUR,
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


class RosmasterBridgeNode(Node):

    def __init__(self):
        super().__init__('rosmaster_bridge_node')

        # ── Parámetros ROS ──────────────────────────────────────────────
        self.declare_parameter('car_type', 1)
        self.declare_parameter('com', '/dev/ttyCH341USB1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)

        # ── NUEVO: parámetros de rampa (protección de picos de corriente) ──
        self.declare_parameter('max_linear_accel', 0.6)   # m/s² — AJUSTAR con medición real de corriente
        self.declare_parameter('max_angular_accel', 2.0)  # rad/s²
        self.declare_parameter('ramp_rate_hz', 30.0)       # frecuencia del lazo de rampeo


        self.add_on_set_parameters_callback(self._on_params_change)

        car_type = self.get_parameter('car_type').value
        port = self.get_parameter('com').value
        baud = self.get_parameter('baudrate').value

        self._bot = RosBoardDrv(car_type=car_type, com=port, baudrate=baud, debug=False)

        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self._bot.set_pid_param(kp, ki, kd, forever=False)

        # ── Estado del watchdog (ruedas) ──────────────────────────────
        self._lock = threading.Lock()
        self._last_cmd_time = time.time()
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_cb)

        # ── NUEVO: estado de la rampa ──
        self._ramp_lock = threading.Lock()
        self._target_vx = 0.0
        self._target_vy = 0.0
        self._target_wz = 0.0
        self._cmd_vx = 0.0   # valor YA rampado, el que de verdad se manda al STM32
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0

        # NUEVO: timer de rampeo — corre independiente de cuándo llega /cmd_vel
        ramp_hz = self.get_parameter('ramp_rate_hz').value
        self._ramp_dt = 1.0 / ramp_hz
        self._ramp_timer = self.create_timer(self._ramp_dt, self._ramp_cb)
        # ── Estado del brazo (última pose conocida de CADA canal) ───────
        # CRÍTICO: sin este estado sincronizado, un comando de gripper
        # sobreescribiría el brazo con una pose vieja/cero, y viceversa
        # — este era el bug que hacía que el brazo pareciera no moverse.
        self._arm_lock = threading.Lock()
        self._last_arm_rad  = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._last_grip_rad = GRIP_RAD_OPEN

        self.create_subscription(
            JointTrajectory, ARM_TOPIC, self._arm_trajectory_cb, 10)
        self.create_subscription(
            JointTrajectory, GRIP_TOPIC, self._grip_trajectory_cb, 10)

        self.get_logger().info(
            f'Bridge listo — puerto={port} car_type={car_type} '
            f'Kp={kp:.3f} Ki={ki:.3f} Kd={kd:.3f}')

    # ─── Ruedas ──────────────────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        with self._lock:
            self._last_cmd_time = time.time()
        # CAMBIO: ya NO se llama set_car_motion aquí directo.
        # Solo actualiza el objetivo; _ramp_cb() es quien manda al motor.
        with self._ramp_lock:
            self._target_vx = float(msg.linear.x)
            self._target_vy = float(msg.linear.y)
            self._target_wz = float(msg.angular.z)

    def _ramp_cb(self):
        """Lazo de rampeo — corre a ramp_rate_hz, acerca cmd_* a target_*
        sin exceder max_linear_accel/max_angular_accel, y es quien
        efectivamente escribe al STM32 vía set_car_motion()."""
        max_lin_a = self.get_parameter('max_linear_accel').value
        max_ang_a = self.get_parameter('max_angular_accel').value

        with self._ramp_lock:
            self._cmd_vx = self._slew(self._cmd_vx, self._target_vx, max_lin_a, self._ramp_dt)
            self._cmd_vy = self._slew(self._cmd_vy, self._target_vy, max_lin_a, self._ramp_dt)
            self._cmd_wz = self._slew(self._cmd_wz, self._target_wz, max_ang_a, self._ramp_dt)
            vx, vy, wz = self._cmd_vx, self._cmd_vy, self._cmd_wz

        self._bot.set_car_motion(vx, vy, wz)

    @staticmethod
    def _slew(current, target, max_rate, dt):
        max_delta = max_rate * dt
        delta = target - current
        delta = max(-max_delta, min(max_delta, delta))
        return current + delta

    def _watchdog_cb(self):
        """Si se pierde /cmd_vel, frena INMEDIATO (bypassa la rampa por
        completo) — esto es una situación de seguridad, no una parada
        programada, así que aquí se prioriza detener el robot lo antes
        posible sobre proteger los motores de un pico de corriente."""
        with self._lock:
            stale = (time.time() - self._last_cmd_time) > WATCHDOG_TIMEOUT
        if stale:
            with self._ramp_lock:
                # Se resetea TANTO el objetivo como el valor ya rampado,
                # para que _ramp_cb() no intente "reanudar" hacia el último
                # target la próxima vez que llegue un /cmd_vel real — sin
                # este reset, el primer ciclo de rampa post-watchdog partiría
                # de un _cmd_vx distinto de cero de forma inconsistente.
                self._target_vx = 0.0
                self._target_vy = 0.0
                self._target_wz = 0.0
                self._cmd_vx = 0.0
                self._cmd_vy = 0.0
                self._cmd_wz = 0.0
            self._bot.set_car_motion(0.0, 0.0, 0.0)

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
            self._last_arm_rad = positions          # <-- FIX: ahora sí se actualiza
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
        for p in params:
            if p.name == 'kp': kp = p.value
            elif p.name == 'ki': ki = p.value
            elif p.name == 'kd': kd = p.value
        self._bot.set_pid_param(kp, ki, kd, forever=False)
        self.get_logger().info(f'PID actualizado — Kp={kp:.4f} Ki={ki:.4f} Kd={kd:.4f}')
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