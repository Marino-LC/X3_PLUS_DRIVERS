#!/usr/bin/env python3
"""
encoder_odometry_math.py
=========================
Cinemática directa + integración odométrica de la base mecanum a partir
de conteos ABSOLUTOS de encoder (FUNC_REPORT_ENCODER / get_motor_encoder()),
implementando directamente las ecuaciones de la tesis (sección "modelo
diferencial de las cuatro ruedas" -> incrementos -> proyección al marco
inercial).

DELIBERADAMENTE sin ninguna dependencia de ROS 2 ni de pyserial: esta
clase solo transforma números. Esto permite:
  1. Testearla de forma aislada (unittest) sin necesitar rclpy ni el
     robot físico conectado.
  2. Reutilizarla desde cualquier nodo que posea el acceso al puerto
     serie, sin duplicar la lógica de integración. En este proyecto la
     instancia el nodo que YA es dueño del objeto RosBoardDrv
     (rosmaster_bridge_node.py), porque el puerto serie
     (/dev/ttyCH341USB0) solo puede tener un lector/escritor seguro a
     la vez -- ver nota de diseño en el bridge.

CONVENCIÓN DE RUEDAS Y SIGNO:
  Igual que WHEEL_JOINTS / wheel_polarity en mecanum_kinematic_node.py
  y WHEEL_SIGN en mecanum_odometry_node.py (simulación):
    índice 0 -> FL (front_left)
    índice 1 -> FR (front_right)
    índice 2 -> BR (back_right)
    índice 3 -> BL (back_left)
  get_motor_encoder() de RosBoardDrv devuelve (m1, m2, m3, m4) en ese
  mismo orden [FL, FR, BR, BL] según la convención ya documentada en
  memoria del proyecto.

ECUACIONES (equivalentes discretas de las de la tesis, sección de
incrementos longitudinal/lateral/angular):

  dx1 = (r/4) * ( dphi_FL + dphi_FR + dphi_BR + dphi_BL)
  dy1 = (r/4) * (-dphi_FL + dphi_FR - dphi_BR + dphi_BL)
  dth = (r / (4*(b+d))) * (-dphi_FL - dphi_FR + dphi_BR + dphi_BL)

donde (b+d) es la misma suma de semi-distancias que en simulación se
nombra k = lx + ly (mecanum_odometry_node.py). Se usa esa notación aquí
para mantener consistencia de nombres entre sim y hardware.

Proyección al marco inercial (rotación por theta previo, ecuación 8 de
la tesis):
  dx = dx1*cos(theta) - dy1*sin(theta)
  dy = dx1*sin(theta) + dy1*cos(theta)

NOTA SOBRE COUNTS_PER_WHEEL_REV:
  2464 = 11 PPR x 56 (reductora) x 4 (cuadratura) -- valor ya validado
  en memoria del proyecto para el ROSMASTER X3 PLUS.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# ── Constantes del encoder ──────────────────────────────────────────────────
COUNTS_PER_WHEEL_REV = 2464          # 11 PPR x 56 gear ratio x 4 quadrature
WHEEL_ORDER = ["FL", "FR", "BR", "BL"]

# Signo por rueda -- misma convención que wheel_polarity en
# mecanum_kinematic_node.py / WHEEL_SIGN en mecanum_odometry_node.py.
# Ajustar SOLO si, tras la prueba de concurrencia del puerto serie
# (paso previo obligatorio), se observa que la odometría por encoders
# sale invertida respecto al movimiento físico real.
DEFAULT_WHEEL_SIGN = [-1.0, -1.0, -1.0, -1.0]


@dataclass
class EncoderOdometryConfig:
    wheel_radius: float = 0.040     # m
    lx: float = 0.110               # m -- semilongitud eje X (b)
    ly: float = 0.102               # m -- semiancho eje Y (d)
    counts_per_rev: int = COUNTS_PER_WHEEL_REV
    wheel_sign: List[float] = field(default_factory=lambda: list(DEFAULT_WHEEL_SIGN))

    @property
    def k(self) -> float:
        """b + d de la tesis == lx + ly aquí, mismo nombre que en sim."""
        return self.lx + self.ly


class MecanumEncoderOdometry:
    """
    Integrador de odometría por encoders absolutos. Sin estado de ROS:
    solo recibe conteos crudos y devuelve/acumula pose.

    Uso típico (dentro de un nodo que sí tiene acceso al hardware):

        odom = MecanumEncoderOdometry(EncoderOdometryConfig(wheel_radius=0.040,
                                                              lx=0.110, ly=0.102))
        ...
        c1, c2, c3, c4 = bot.get_motor_encoder()   # FL, FR, BR, BL
        result = odom.update(c1, c2, c3, c4)
        if result is not None:
            x, y, theta, vx, vy, wz = result
    """

    def __init__(self, config: Optional[EncoderOdometryConfig] = None):
        self.cfg = config or EncoderOdometryConfig()
        self._prev_counts: Optional[Tuple[int, int, int, int]] = None
        self._prev_time: Optional[float] = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    # ── Reset ────────────────────────────────────────────────────────────
    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """Reinicia la pose acumulada. NO reinicia _prev_counts: el
        siguiente update() sigue calculando el delta correctamente desde
        el último conteo real leído, evitando un salto espurio de
        integración si el reset ocurre entre dos lecturas."""
        self.x, self.y, self.theta = x, y, theta

    def hard_reset_counts(self):
        """Fuerza reinicio también de la referencia de conteos: el
        próximo update() se toma como el primer ciclo (sin delta).
        Usar solo si se sospecha que _prev_counts quedó desincronizado
        (p. ej. tras una reconexión serie)."""
        self._prev_counts = None
        self._prev_time = None

    # ── Integración principal ───────────────────────────────────────────
    def update(self, c_fl: int, c_fr: int, c_br: int, c_bl: int,
               now: Optional[float] = None
               ) -> Optional[Tuple[float, float, float, float, float, float]]:
        """
        Args:
            c_fl, c_fr, c_br, c_bl: conteos ABSOLUTOS acumulados de cada
                encoder, en el orden que devuelve get_motor_encoder()
                (FL, FR, BR, BL).
            now: timestamp opcional (segundos, monotónico) para calcular
                velocidades vx/vy/wz. Si se omite, esos campos devueltos
                serán 0.0 (solo se actualiza la pose).

        Returns:
            None en el primer ciclo (no hay conteo previo con qué
            calcular el delta). En ciclos siguientes:
            (x, y, theta, vx, vy, wz)
        """
        counts = (c_fl, c_fr, c_br, c_bl)

        if self._prev_counts is None:
            self._prev_counts = counts
            self._prev_time = now
            return None

        d_counts = [c - p for c, p in zip(counts, self._prev_counts)]
        d_counts = [dc * s for dc, s in zip(d_counts, self.cfg.wheel_sign)]
        self._prev_counts = counts

        dt = None
        if now is not None and self._prev_time is not None:
            dt = now - self._prev_time
        self._prev_time = now

        # counts -> incremento angular de cada rueda (rad)
        d_phi = [2.0 * math.pi * dc / self.cfg.counts_per_rev for dc in d_counts]
        dphi_fl, dphi_fr, dphi_br, dphi_bl = d_phi

        r = self.cfg.wheel_radius
        k = self.cfg.k   # b + d

        # ── Incrementos en el marco propio de la plataforma (tesis §4-6) ──
        dx1 = (r / 4.0) * (dphi_fl + dphi_fr + dphi_br + dphi_bl)
        dy1 = (r / 4.0) * (-dphi_fl + dphi_fr - dphi_br + dphi_bl)
        dth = (r / (4.0 * k)) * (-dphi_fl - dphi_fr + dphi_br + dphi_bl)

        # ── Proyección al marco inercial, rotando por theta previo (§8) ──
        cos_th = math.cos(self.theta)
        sin_th = math.sin(self.theta)
        dx = dx1 * cos_th - dy1 * sin_th
        dy = dx1 * sin_th + dy1 * cos_th

        self.x += dx
        self.y += dy
        self.theta += dth
        # normalizar a (-pi, pi]
        self.theta = (self.theta + math.pi) % (2.0 * math.pi) - math.pi

        vx = vy = wz = 0.0
        if dt is not None and dt > 0.0:
            vx = dx1 / dt
            vy = dy1 / dt
            wz = dth / dt

        return self.x, self.y, self.theta, vx, vy, wz