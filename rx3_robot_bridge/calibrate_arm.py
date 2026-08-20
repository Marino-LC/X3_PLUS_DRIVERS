#!/usr/bin/env python3
"""
calibrate_arm.py
================
Script interactivo para calibrar el brazo Dofbot.
Permite mover cada servo individualmente, leer ángulos y activar/desactivar torque.
Útil para determinar servo_zero_deg y signo de cada joint.

Uso: ros2 run rx3_robot_bridge calibrate_arm [--com /dev/ttyUSB0] [--car_type 1]
"""

import sys
import time
import argparse
from rosboard_drv.rosboard_drv import RosBoardDrv

class ArmCalibrator:
    def __init__(self, com, car_type):
        self.bot = RosBoardDrv(com=com, car_type=car_type, baudrate=115200, debug=False)
        self.servo_ids = [1, 2, 3, 4, 5, 6]  # IDs de los servos (1-5 articulaciones, 6 gripper)
        self.current_angles = {sid: 90 for sid in self.servo_ids}  # guardar último ángulo enviado
        self.torque_enabled = True
        self.bot.set_uart_servo_torque(1)  # activar torque por defecto
        print("Brazo calibrador iniciado. Torque activado.")
        print("Comandos disponibles (escribe el número y pulsa Enter):")
        print("  <servo_id> <ángulo>   -> Mover servo a ángulo (grados)")
        print("  r <servo_id>          -> Leer ángulo actual del servo")
        print("  t                     -> Alternar torque (on/off)")
        print("  q                     -> Salir")
        print("Ejemplo: '1 90' mueve servo 1 a 90°")

    def move(self, sid, angle):
        """Mueve un servo a un ángulo (grados) con tiempo de movimiento 1s."""
        try:
            angle = int(angle)
            if sid not in self.servo_ids:
                print(f"ID de servo inválido: {sid}. Debe ser {self.servo_ids}")
                return
            # Limitar ángulo a rango razonable (0-180, excepto servo 5 que es 0-270)
            if sid == 5:
                angle = max(0, min(270, angle))
            else:
                angle = max(0, min(180, angle))
            print(f"Moviendo servo {sid} a {angle}° ...")
            self.bot.set_uart_servo_angle(sid, angle, 1000)  # 1 segundo
            time.sleep(1.2)  # esperar a que termine
            self.current_angles[sid] = angle
            # Leer y mostrar ángulo real
            real_angle = self.bot.get_uart_servo_angle(sid)
            print(f"Ángulo leído: {real_angle}°")
        except ValueError:
            print("El ángulo debe ser un número entero.")
        except Exception as e:
            print(f"Error: {e}")

    def read(self, sid):
        """Lee y muestra el ángulo actual de un servo."""
        try:
            sid = int(sid)
            if sid not in self.servo_ids:
                print(f"ID de servo inválido: {sid}")
                return
            angle = self.bot.get_uart_servo_angle(sid)
            print(f"Servo {sid} → {angle}°")
        except ValueError:
            print("El ID debe ser un número entero.")

    def toggle_torque(self):
        """Alterna el torque de todos los servos."""
        self.torque_enabled = not self.torque_enabled
        self.bot.set_uart_servo_torque(1 if self.torque_enabled else 0)
        print(f"Torque {'activado' if self.torque_enabled else 'desactivado'}")

    def run(self):
        """Bucle principal de comandos."""
        while True:
            try:
                cmd = input("> ").strip()
                if not cmd:
                    continue
                parts = cmd.split()
                if parts[0].lower() == 'q':
                    print("Saliendo...")
                    break
                elif parts[0].lower() == 't':
                    self.toggle_torque()
                elif parts[0].lower() == 'r':
                    if len(parts) < 2:
                        print("Uso: r <servo_id>")
                    else:
                        self.read(parts[1])
                else:
                    # Asumimos que es "servo_id angulo"
                    if len(parts) >= 2:
                        sid = int(parts[0])
                        angle = int(parts[1])
                        self.move(sid, angle)
                    else:
                        print("Comando no reconocido. Usa: <id> <ángulo> | r <id> | t | q")
            except KeyboardInterrupt:
                print("\nInterrupción. Saliendo...")
                break
            except Exception as e:
                print(f"Error: {e}")

def main(args=None):
    parser = argparse.ArgumentParser(description="Calibración interactiva del brazo Dofbot")
    parser.add_argument('--com', default='/dev/ttyCH341USB1', help='Puerto serie del RosBoard')
    parser.add_argument('--car_type', type=int, default=1, help='Tipo de coche (1=X3, 2=X3PLUS, 5=R2)')
    args = parser.parse_args()

    calibrator = ArmCalibrator(args.com, args.car_type)
    calibrator.run()

if __name__ == '__main__':
    main()