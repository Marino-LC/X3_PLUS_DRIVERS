#!/usr/bin/env python3
# coding: utf-8

import time
import csv
from rosboard_drv.rosboard_drv import RosBoardDrv

def prueba_escalonada():
    print("Inicializando conexión con la tarjeta base...")
    bot = RosBoardDrv(com="/dev/ttyCH341USB0", debug=False, logpath="./logs")
    
    # Hilo indispensable para poder leer los encoders
    bot.create_receive_threading()
    time.sleep(1.0) # Dar tiempo a que los hilos y lecturas se estabilicen
    
    # Detener los motores por seguridad antes de empezar
    bot.set_motor(0, 0, 0, 0)
    time.sleep(0.5)

    # Secuencia de PWM a evaluar
    pasos_pwm = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
    tiempo_por_paso = 1.5      # Segundos que durará cada escalón
    frecuencia_muestreo = 0.01 # ~100 Hz (10 ms de pausa por ciclo)
    
    datos_recolectados = []
    
    print("--- INICIANDO PRUEBA ESCALONADA EN 3 SEGUNDOS ---")
    time.sleep(3)
    print("¡EJECUTANDO!")

    tiempo_inicio = time.time()
    
    # Bucle de control por cada escalón de PWM
    for pwm in pasos_pwm:
        print(f"-> Aplicando escalón de PWM: {pwm}")
        bot.set_motor(pwm, pwm, pwm, pwm) 
        
        tiempo_paso_inicio = time.time()
        
        # Bucle de recolección de datos para este escalón específico
        while (time.time() - tiempo_paso_inicio) < tiempo_por_paso:
            tiempo_actual = time.time() - tiempo_inicio
            
            # Leer encoders
            m1, m2, m3, m4 = bot.get_motor_encoder()
            
            # Guardar datos en memoria
            datos_recolectados.append([tiempo_actual, pwm, m1, m2, m3, m4])
            
            # Delay para no saturar el bus
            time.sleep(frecuencia_muestreo)

    # Detener motores al finalizar la secuencia completa
    print("Deteniendo motores por seguridad...")
    bot.set_motor(0, 0, 0, 0)
    
    # Guardar en CSV
    nombre_archivo = "datos_escalera.csv"
    with open(nombre_archivo, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Tiempo_s', 'Entrada_PWM', 'Enc_M1', 'Enc_M2', 'Enc_M3', 'Enc_M4'])
        writer.writerows(datos_recolectados)
        
    print(f"Prueba finalizada. Datos exportados exitosamente a '{nombre_archivo}'.")

if __name__ == "__main__":
    try:
        prueba_escalonada()
    except KeyboardInterrupt:
        print("\nPrueba interrumpida por el usuario. Deteniendo motores...")
        # Instanciar bot de rescate asegurándonos de usar el puerto correcto (CH341)
        bot_rescue = RosBoardDrv(com="/dev/ttyCH341USB0")
        bot_rescue.set_motor(0, 0, 0, 0)