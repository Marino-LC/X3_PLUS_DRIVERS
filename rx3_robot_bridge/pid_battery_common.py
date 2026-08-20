#!/usr/bin/env python3
"""
pid_battery_common.py
======================
Constantes, coreografía del brazo y estructuras de datos COMPARTIDAS
entre la evaluación en simulación (ag_motion_tests.py, zn_tuner.py) y la
validación en hardware físico (hw_pid_battery.py).

Este módulo NO depende de Gazebo, DEAP ni de ningún paquete de
simulación — solo de la biblioteca estándar de Python — precisamente
para poder copiarse de forma aislada a la Jetson Orin (que ejecuta el
bridge de hardware, sin el resto del stack de simulación) sin arrastrar
dependencias innecesarias.

SINCRONIZACIÓN: este es el único archivo que debe copiarse/sincronizarse
entre el workspace de simulación y la Orin cada vez que cambie algo de
la batería de pruebas o la coreografía del brazo. Si cambias algo aquí
en un lado y no en el otro, sim y hardware dejan de ser comparables —
exactamente el mismo tipo de problema que ya se corrigió una vez entre
zn_tuner_openloop.py y el AG.
"""

import math
from dataclasses import dataclass, field
from typing import List

# ── Distancias / ángulos de prueba ────────────────────────────────────────
DIST_X      = 0.50
DIST_Y      = 0.20 
DIST_RETURN = 0.10
ROT_ANGLE   = math.pi / 2

# ── Velocidades de referencia ─────────────────────────────────────────────
VX_REF = 0.20
VY_REF = 0.20
WZ_REF = 0.20

# ── Lazo de control / tolerancias ─────────────────────────────────────────
CTRL_DT      = 0.05
SETTLE_TIME  = 0.30
TIMEOUT_MOVE = 2
TIMEOUT_ROT  = 2
POS_TOL      = 0.04
YAW_TOL      = 0.05
MAX_POS_ERROR_ABORT = 1.0

# ── Pesos de la función de costo ──────────────────────────────────────────
W1, W2, W3 = 0.35, 0.30, 0.35
PENALTY_TO = 50.0

# ── Brazo Dofbot — coreografía pick & place lado a lado ───────────────────
ARM_JOINTS  = ["arm_joint_01","arm_joint_02","arm_joint_03",
               "arm_joint_04","arm_joint_05"]
GRIP_JOINTS = ["grip_joint"]
ARM_TOPIC   = "/dofbot_trajectory_controller/joint_trajectory"
GRIP_TOPIC  = "/dofbot_gripper_controller/joint_trajectory"
ARM_MOVE_DUR = 0.55
ARM_HOLD_DUR = 0.55

ARM_HOME       = [ 0.00,  0.00,  0.00,  0.00, 1.57]
ARM_PICK_LEFT  = [-1.20, -1.25, -0.7, -0.3, 1.57]
ARM_PICK_RIGHT = [ 1.20, -1.25, -0.7, -0.3, 1.57]
ARM_CHOREOGRAPHY = [(1,4), (3,2), (5,1)]
GRIP_OPEN   = -0.95
GRIP_CLOSED = 0.00


@dataclass
class SegmentLog:
    name:    str
    t:       List[float] = field(default_factory=list)
    vx_ref:  List[float] = field(default_factory=list)
    vy_ref:  List[float] = field(default_factory=list)
    wz_ref:  List[float] = field(default_factory=list)
    vx_real: List[float] = field(default_factory=list)
    vy_real: List[float] = field(default_factory=list)
    wz_real: List[float] = field(default_factory=list)
    pos_err: List[float] = field(default_factory=list)
    x_ref:    List[float] = field(default_factory=list)
    y_ref:    List[float] = field(default_factory=list)
    yaw_ref:  List[float] = field(default_factory=list)
    x_real:   List[float] = field(default_factory=list)
    y_real:   List[float] = field(default_factory=list)
    yaw_real: List[float] = field(default_factory=list)


class Pose2D:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x, self.y, self.yaw = x, y, yaw
    def copy(self): return Pose2D(self.x, self.y, self.yaw)
    def dist(self, o): return math.hypot(self.x-o.x, self.y-o.y)