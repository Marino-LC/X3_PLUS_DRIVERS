#!/usr/bin/env python3
"""
test_concurrencia_serial.py
===========================
Prueba aislada para validar concurrencia de lectura/escritura en el bus serial.
"""

import time
import threading
from rosboard_drv.rosboard_drv import RosBoardDrv

PUERTO = "/dev/ttyCH341USB0" 

def hilo_escritura_intensiva(bot, stop_event):
    while not stop_event.is_set():
        bot.set_car_motion(0.0, 0.0, 0.0)
        bot.set_uart_servo_angle_array([90, 90, 90, 90, 90, 90])
        time.sleep(0.05) 

def main():
    print("=== INICIANDO PRUEBA DE ESTRÉS: CONCURRENCIA SERIAL ===")
    
    try:
        bot = RosBoardDrv(com=PUERTO, debug=False)
    except Exception as e:
        print(f"❌ Error al conectar con el puerto {PUERTO}: {e}")
        return

    # 1. Iniciar el hilo de lectura interna y ACTIVAR EL AUTO-REPORTE
    print("[1] Activando hilo de recepción (create_receive_threading)...")
    bot.create_receive_threading()
    
    # CRÍTICO en modo aislado: Forzamos a la placa a escupir datos
    try:
        bot.set_auto_report_state(True)
    except AttributeError:
        pass # Si tu versión del driver no requiere esto, lo ignoramos de forma segura

    # 2. Capturar línea base con impresión de Debug
    print("[2] Esperando primera trama válida de encoders (Timeout: 5s)...")
    ret_inicial = None
    
    for intento in range(50):
        bot.set_car_motion(0.0, 0.0, 0.0) 
        ret = bot.get_motor_encoder()
        
        # ¡AQUÍ ESTÁ LA MAGIA! Imprimimos lo que lee el driver
        print(f"   -> [Debug] Intento {intento}: Datos crudos recibidos = {ret}")
        
        # CORRECCIÓN: Yahboom devuelve 4 motores, no 5 elementos.
        if ret is not None and len(ret) == 4:
            ret_inicial = ret
            break 
            
        time.sleep(0.1)

    if ret_inicial is None:
        print("❌ Error Fatal: El hardware no reportó. Revisa que no haya otra terminal de ROS 2 abierta robando el puerto.")
        return
        
    enc_ini_1, enc_ini_2, enc_ini_3, enc_ini_4 = ret_inicial
    print(f"\n[2] ✅ ¡Conexión exitosa! Encoders iniciales: [{enc_ini_1}, {enc_ini_2}, {enc_ini_3}, {enc_ini_4}]")

    # 3. Desatar la concurrencia
    print("[3] Iniciando bombardeo de comandos de escritura concurrentes (20Hz)...")
    stop_event = threading.Event()
    writer_thread = threading.Thread(target=hilo_escritura_intensiva, args=(bot, stop_event))
    writer_thread.start()

    # 4. Evaluación y Monitoreo
    print("[4] Evaluando estabilidad de lectura concurrente durante 10 segundos...\n")
    fallos = 0
    lecturas_totales = 0
    inicio_test = time.time()

    try:
        while time.time() - inicio_test < 10.0:
            ret = bot.get_motor_encoder()
            lecturas_totales += 1
            
            # Ajustado a longitud de 4
            if ret is None or len(ret) != 4:
                print("⚠️ ALERTA [Trama Rota]: El driver devolvió datos incompletos o nulos.")
                fallos += 1
                continue
                
            e1, e2, e3, e4 = ret
            
            if abs(e1 - enc_ini_1) > 2 or abs(e2 - enc_ini_2) > 2:
                print(f"⚠️ ALERTA [Salto Espurio]: Lectura irreal detectada -> [{e1}, {e2}, {e3}, {e4}]")
                fallos += 1

            time.sleep(0.05) 

    except KeyboardInterrupt:
        print("\nPrueba interrumpida manualmente.")

    finally:
        stop_event.set()
        writer_thread.join()
        bot.set_car_motion(0.0, 0.0, 0.0)
        del bot 

        print("\n=== RESULTADOS DE LA PRUEBA ===")
        print(f"Lecturas totales completadas: {lecturas_totales}")
        print(f"Tramas corruptas / Errores de colisión: {fallos}")

        if fallos == 0:
            print("\nEl bus serial soporta Full-Duplex simulado.")
        else:
            porcentaje_error = (fallos / lecturas_totales) * 100
            print(f"\nFALLO DETECTADO (Tasa de error: {porcentaje_error:.1f}%): Necesitarás implementar un Lock (Mutex).")

if __name__ == '__main__':
    main()