#!/usr/bin/env python3
# coding: utf-8
"""
ROSBoard V3.3.9

"""


import struct
import time
import serial
import threading

import datetime
import logging
from .rosboard_errors import SerialConnectionError

class RosBoardDrv(object):
    # Constants Definition
    # __uart_state = 0        # Global variable???
    # Hidden/Reserved Constants
    __HEAD = 0xFF
    __DEVICE_ID = 0xFC
    __COMPLEMENT = 257 - __DEVICE_ID
    # __CAR_TYPE = car_type
    __CAR_ADJUST = 0x80

    FUNC_AUTO_REPORT = 0x01
    FUNC_BEEP = 0x02
    FUNC_PWM_SERVO = 0x03
    FUNC_PWM_SERVO_ALL = 0x04
    FUNC_RGB = 0x05
    FUNC_RGB_EFFECT = 0x06

    FUNC_REPORT_SPEED = 0x0A
    FUNC_REPORT_MPU_RAW = 0x0B
    FUNC_REPORT_IMU_ATT = 0x0C
    FUNC_REPORT_ENCODER = 0x0D
    FUNC_REPORT_ICM_RAW = 0x0E
    
    FUNC_RESET_STATE = 0x0F

    FUNC_MOTOR = 0x10
    FUNC_CAR_RUN = 0x11
    FUNC_MOTION = 0x12
    FUNC_SET_MOTOR_PID = 0x13
    FUNC_SET_YAW_PID = 0x14
    FUNC_SET_CAR_TYPE = 0x15

    FUNC_UART_SERVO = 0x20
    FUNC_UART_SERVO_ID = 0x21
    FUNC_UART_SERVO_TORQUE = 0x22
    FUNC_ARM_CTRL = 0x23
    FUNC_ARM_OFFSET = 0x24

    FUNC_AKM_DEF_ANGLE = 0x30
    FUNC_AKM_STEER_ANGLE = 0x31

    FUNC_REQUEST_DATA = 0x50
    FUNC_VERSION = 0x51

    FUNC_RESET_FLASH = 0xA0

    CARTYPE_X3 = 0x01
    CARTYPE_X3_PLUS = 0x02
    CARTYPE_X1 = 0x04
    CARTYPE_R2 = 0x05

    def __init__(self, car_type=1, com="/dev/ttyCH341USB0", baudrate=115200, delay=0.002, debug=False, logpath = ""):
        self.ser = serial.Serial(com, 115200)
        self.__uart_state = 0
        self.__delay_time = delay
        self.__debug = debug
        self.__logger = logging.getLogger("RosBoardDrv")
        self.config_logging(logpath, debug)

        self.__ax = self.__ay = self.__az = 0.0
        self.__gx = self.__gy = self.__gz = 0.0
        self.__mx = self.__my = self.__mz = 0.0
        self.__vx = self.__vy = self.__vz = 0.0

        self.__yaw = self.__roll = self.__pitch = 0.0

        self.__encoder_m1 = self.__encoder_m2 = 0
        self.__encoder_m3 = self.__encoder_m4 = 0

        self.__read_id = 0
        self.__read_val = 0

        self.__read_arm_ok = 0
        self.__read_arm = [-1, -1, -1, -1, -1, -1]

        self.__version_H = self.__version_L = 0
        self.__version = 0

        self.__pid_index = 0
        self.__kp1 = self.__ki1 = self.__kd1 = 0

        self.__arm_offset_state = 0
        self.__arm_offset_id = 0
        self.__arm_ctrl_enable = True

        self.__battery_voltage = 0

        self.__akm_def_angle = 100
        self.__akm_readed_angle = False
        self.__AKM_SERVO_ID = 0x01

        self.__car_type = car_type
        self.__read_car_type = 0

        if self.__debug:
            self.__logger.debug(f"commmand delay setting to {str(self.__delay_time)} seg.")

        try:
            self.ser = serial.Serial(com, baudrate, timeout=1)
            if self.__debug:
                self.__logger.debug(f"Opened serial port {com} at {baudrate} baud.")
        except serial.SerialException as e:
            self.__logger.error("Failed to open serial port.")
            raise SerialConnectionError("Could not establish serial connection") from e

        # ?? Turn on the torque of the mechanical arm to avoid not being able to read the angle when the No. 6 servo is plugged in for the first time.
        self.set_uart_servo_torque(1)
        time.sleep(0.002)

    def config_logging(self, logpath = "", debug = False):
        """Sets up logging configuration."""
        format_style = '[%(levelname)s] [%(asctime)s]: %(message)s'
        logging.basicConfig(
            filename=f"{'./logs' if len(logpath)== 0 else logpath}/rosboard_drv_{datetime.datetime.now():%d%m%y}.log",
            level= logging.DEBUG if debug else logging.INFO,
            format=format_style
        )

    def __del__(self):
        self.ser.close()
        self.__uart_state = 0
        if self.__debug:
            self.__logger.debug("Destructor called, serial connection closed.")


    # According to the type of data frame to make the corresponding parsing
    def __parse_data(self, ext_type, ext_data):
        """
        Parses incoming data based on type of data frame.
        
        :param ext_type: Type of the incoming data packet.
        :param ext_data: Data payload in byte format.
        """

        if self.__debug:
            self.__logger.debug(f"Called __parse_data(0x{ext_type:02x}, {ext_data})")
        if ext_type == self.FUNC_REPORT_SPEED:
            # print(ext_data)
            self.__vx = int(struct.unpack('h', bytearray(ext_data[0:2]))[0]) / 1000.0
            self.__vy = int(struct.unpack('h', bytearray(ext_data[2:4]))[0]) / 1000.0
            self.__vz = int(struct.unpack('h', bytearray(ext_data[4:6]))[0]) / 1000.0
            self.__battery_voltage = struct.unpack('B', bytearray(ext_data[6:7]))[0]
        # (MPU9250)the original gyroscope, accelerometer, magnetometer data
        elif ext_type == self.FUNC_REPORT_MPU_RAW:
            gyro_ratio = 1 / 3754.9
            self.__gx = struct.unpack('h', bytearray(ext_data[0:2]))[0]*gyro_ratio
            self.__gy = struct.unpack('h', bytearray(ext_data[2:4]))[0]*-gyro_ratio
            self.__gz = struct.unpack('h', bytearray(ext_data[4:6]))[0]*-gyro_ratio

            accel_ratio = 1 / 1671.84
            self.__ax = struct.unpack('h', bytearray(ext_data[6:8]))[0]*accel_ratio
            self.__ay = struct.unpack('h', bytearray(ext_data[8:10]))[0]*accel_ratio
            self.__az = struct.unpack('h', bytearray(ext_data[10:12]))[0]*accel_ratio

            mag_ratio = 1.0
            self.__mx = struct.unpack('h', bytearray(ext_data[12:14]))[0]*mag_ratio
            self.__my = struct.unpack('h', bytearray(ext_data[14:16]))[0]*mag_ratio
            self.__mz = struct.unpack('h', bytearray(ext_data[16:18]))[0]*mag_ratio
        # (ICM20948)the original gyroscope, accelerometer, magnetometer data
        elif ext_type == self.FUNC_REPORT_ICM_RAW:
            gyro_ratio = 1 / 1000.0
            self.__gx = struct.unpack('h', bytearray(ext_data[0:2]))[0]*gyro_ratio
            self.__gy = struct.unpack('h', bytearray(ext_data[2:4]))[0]*gyro_ratio
            self.__gz = struct.unpack('h', bytearray(ext_data[4:6]))[0]*gyro_ratio

            accel_ratio = 1 / 1000.0
            self.__ax = struct.unpack('h', bytearray(ext_data[6:8]))[0]*accel_ratio
            self.__ay = struct.unpack('h', bytearray(ext_data[8:10]))[0]*accel_ratio
            self.__az = struct.unpack('h', bytearray(ext_data[10:12]))[0]*accel_ratio

            mag_ratio = 1 / 1000.0
            self.__mx = struct.unpack('h', bytearray(ext_data[12:14]))[0]*mag_ratio
            self.__my = struct.unpack('h', bytearray(ext_data[14:16]))[0]*mag_ratio
            self.__mz = struct.unpack('h', bytearray(ext_data[16:18]))[0]*mag_ratio
        # the attitude Angle of the board
        elif ext_type == self.FUNC_REPORT_IMU_ATT:
            self.__roll = struct.unpack('h', bytearray(ext_data[0:2]))[0] / 10000.0
            self.__pitch = struct.unpack('h', bytearray(ext_data[2:4]))[0] / 10000.0
            self.__yaw = struct.unpack('h', bytearray(ext_data[4:6]))[0] / 10000.0
        # Encoder data on all four wheels
        elif ext_type == self.FUNC_REPORT_ENCODER:
            self.__encoder_m1 = struct.unpack('i', bytearray(ext_data[0:4]))[0]
            self.__encoder_m2 = struct.unpack('i', bytearray(ext_data[4:8]))[0]
            self.__encoder_m3 = struct.unpack('i', bytearray(ext_data[8:12]))[0]
            self.__encoder_m4 = struct.unpack('i', bytearray(ext_data[12:16]))[0]
        ## Why???
        ### else:
        elif ext_type == self.FUNC_UART_SERVO:
            self.__read_id = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            self.__read_val = struct.unpack('h', bytearray(ext_data[1:3]))[0]
            if self.__debug:
                self.__logger.debug(f"FUNC_UART_SERVO: {self.__read_id}, {self.__read_val}")

        elif ext_type == self.FUNC_ARM_CTRL:
            self.__read_arm[0] = struct.unpack('h', bytearray(ext_data[0:2]))[0]
            self.__read_arm[1] = struct.unpack('h', bytearray(ext_data[2:4]))[0]
            self.__read_arm[2] = struct.unpack('h', bytearray(ext_data[4:6]))[0]
            self.__read_arm[3] = struct.unpack('h', bytearray(ext_data[6:8]))[0]
            self.__read_arm[4] = struct.unpack('h', bytearray(ext_data[8:10]))[0]
            self.__read_arm[5] = struct.unpack('h', bytearray(ext_data[10:12]))[0]
            self.__read_arm_ok = 1
            if self.__debug:
                self.__logger.debug(f"FUNC_ARM_CTRL: {self.__read_arm}")

        elif ext_type == self.FUNC_VERSION:
            self.__version_H = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            self.__version_L = struct.unpack('B', bytearray(ext_data[1:2]))[0]
            if self.__debug:
                self.__logger.debug(f"FUNC_VERSION: {self.__version_H}, {self.__version_L}")

        elif ext_type == self.FUNC_SET_MOTOR_PID:
            self.__pid_index = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            self.__kp1 = struct.unpack('h', bytearray(ext_data[1:3]))[0]
            self.__ki1 = struct.unpack('h', bytearray(ext_data[3:5]))[0]
            self.__kd1 = struct.unpack('h', bytearray(ext_data[5:7]))[0]
            if self.__debug:
                self.__logger.debug(f"FUNC_SET_MOTOR_PID: {self.__pid_index}, [{self.__kp1}, {self.__ki1}, {self.__kd1}]")

        elif ext_type == self.FUNC_SET_YAW_PID:
            self.__pid_index = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            self.__kp1 = struct.unpack('h', bytearray(ext_data[1:3]))[0]
            self.__ki1 = struct.unpack('h', bytearray(ext_data[3:5]))[0]
            self.__kd1 = struct.unpack('h', bytearray(ext_data[5:7]))[0]
            if self.__debug:
                self.__logger.debug(f"FUNC_SET_YAW_PID: {self.__pid_index}, [{self.__kp1}, {self.__ki1}, {self.__kd1}]")

        elif ext_type == self.FUNC_ARM_OFFSET:
            self.__arm_offset_id = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            self.__arm_offset_state = struct.unpack('B', bytearray(ext_data[1:2]))[0]
            if self.__debug:
                self.__logger.debug(f"FUNC_ARM_OFFSET: {self.__arm_offset_id}, {self.__arm_offset_state}")

        elif ext_type == self.FUNC_AKM_DEF_ANGLE:
            id = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            self.__akm_def_angle = struct.unpack('B', bytearray(ext_data[1:2]))[0]
            self.__akm_readed_angle = True
            if self.__debug:
                self.__logger.debug(f"FUNC_AKM_DEF_ANGLE: {id}, {self.__akm_def_angle}")
        
        elif ext_type == self.FUNC_SET_CAR_TYPE:
            car_type = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            self.__read_car_type = car_type
            if self.__debug:
                self.__logger.debug(f"FUNC_SET_CAR_TYPE: {self.__read_car_type}")
        ## Add by me        
        else:
            self.__logger.warning(f"Unrecognized function: ext_type 0x{ext_type:02x} ext_data {ext_data}")

    def __receive_data(self):
        """receive data"""
        # Clear buffer
        self.ser.flushInput()
        while True:
            head1 = bytearray(self.ser.read())[0]
            if head1 == self.__HEAD:
                head2 = bytearray(self.ser.read())[0]
                check_sum = 0
                rx_check_num = 0
                if head2 == self.__DEVICE_ID - 1:
                    ext_len = bytearray(self.ser.read())[0]
                    ext_type = bytearray(self.ser.read())[0]
                    ext_data = []
                    check_sum = ext_len + ext_type
                    data_len = ext_len - 2
                    while len(ext_data) < data_len:
                        value = bytearray(self.ser.read())[0]
                        ext_data.append(value)
                        if len(ext_data) == data_len:
                            rx_check_num = value
                        else:
                            check_sum = check_sum + value
                    if check_sum % 256 == rx_check_num:
                        self.__parse_data(ext_type, ext_data)
                    else:
                        if self.__debug:
                            self.__logger.debug(f"Check Sum Error: {ext_len}, {ext_type}, {ext_data}")

    def __request_data(self, function, param=0):
        """
        Request data 

        Args:
            function: corresponding function word to return data 
            parm: parameter passed in
        """
        cmd = [self.__HEAD, self.__DEVICE_ID, 0x05, self.FUNC_REQUEST_DATA, int(function) & 0xff, int(param) & 0xff]
        checksum = sum(cmd, self.__COMPLEMENT) & 0xff
        cmd.append(checksum)
        self.ser.write(cmd)
        if self.__debug:
            self.__logger.debug(f"Requesting data: {cmd}")
        time.sleep(0.002)

    def __arm_convert_value(self, s_id, s_angle):
        """Arm converts Angle to position pulse"""
        value = -1
        if s_id == 1:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 2:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 3:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 4:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 5:
            value = int((3700 - 380) * (s_angle - 0) / (270 - 0) + 380)
        elif s_id == 6:
            value = int((3100 - 900) * (s_angle - 0) / (180 - 0) + 900)
        return value

    def __arm_convert_angle(self, s_id, s_value):
        """Arm converts position pulses into angles"""
        s_angle = -1
        if s_id == 1:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 2:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 3:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 4:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 5:
            s_angle = int((270 - 0) * (s_value - 380) / (3700 - 380) + 0 + 0.5)
        elif s_id == 6:
            s_angle = int((180 - 0) * (s_value - 900) / (3100 - 900) + 0 + 0.5)
        return s_angle

    def __limit_motor_value(self, value):
        """
        Limit the PWM duty ratio value of motor input. 
        
        Args:
            value: 127, keep the original data, do not modify the current motor speed  
        """
        if value == 127:
            return 127
        elif value > 100:
           return 100
        elif value < -100:
            return -100
        else:
            return int(value)

    def create_receive_threading(self):
        """Start the thread that receives and processes data"""
        try:
            if self.__uart_state == 0:
                name1 = "task_serial_receive"
                task_receive = threading.Thread(target=self.__receive_data, name=name1)
                task_receive.setDaemon(True)
                task_receive.start()
                self.__logger.info("Started data reception thread.")
                self.__uart_state = 1
                time.sleep(.05)
        except Exception as e:
            self.__logger.error(f"ON create_receive_threading: {e}")
            pass
    
    def set_auto_report_state(self, enable, forever=False):
        """
        The MCU automatically returns the data status bit, which is enabled by default. 
        If the switch is closed, the data reading function will be affected.

        Args: 
            enable: If is true, The underlying expansion board sends four different packets of data 
            every 10 milliseconds, so each packet is refreshed every 40 milliseconds. If enable is false, 
            the report is not sent.
            forever: True for permanent, False for temporary
        """
        try:
            state1 = 0
            state2 = 0
            if enable:
                state1 = 1
            if forever:
                state2 = 0x5F
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x05, self.FUNC_AUTO_REPORT, state1, state2]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.__logger.debug(f"ON set_auto_report_state: executed {cmd}")
            time.sleep(self.__delay_time)
        except Exception as e:
            self.__logger.error(f'ON set_auto_report_state: {e}')
            pass

    def set_beep(self, on_time):
        """
        Buzzer switch. 
        
        Args:
            on_time: 0 the buzzer is off. 1, the buzzer keeps ringing. 
                If on_time >=10 automatically closes after xx milliseconds (on_time is a multiple of 10)
        """
        try:
            if on_time < 0:
                self.__logger.warning(f'Invalid beep input: {on_time}')
                return
            value = bytearray(struct.pack('h', int(on_time)))

            cmd = [self.__HEAD, self.__DEVICE_ID, 0x05, self.FUNC_BEEP, value[0], value[1]]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.__logger.debug(f"ON set_beep: executed {cmd}")
            time.sleep(self.__delay_time)
        except Exception as e:
            self.__logger.error(f'ON set_beep: {e}')
            pass

    def set_pwm_servo(self, servo_id, angle):
        """
        PWD Servo control

        Args:
            servo_id: Corresponding PWD servo output [1, 4] 
            angle: Corresponding servo Angle value [0, 180]
        """
        try:
            if servo_id < 1 or servo_id > 4:
                self.__logger.error(f"ON set_pwm_servo: servo_id ({servo_id}) invalid.")
                return
            if angle > 180:
                angle = 180
            elif angle < 0:
                angle = 0
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_PWM_SERVO, int(servo_id), int(angle)]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.__logger.debug(f"ON set_pwm_servo: executed {cmd}")
            time.sleep(self.__delay_time)
        except Exception as e:
            self.__logger.error(f'ON set_pwm_servo: {e}')
            pass

    def set_pwm_servo_all(self, angle_s1, angle_s2, angle_s3, angle_s4):
        """
        Simultaneously control the angles of all four PWM channels

        Args:
            angle_s1: PWM Servo angle 1 value (0 - 180)
            angle_s2: PWM Servo angle 2 value (0 - 180)
            angle_s3: PWM Servo angle 3 value (0 - 180)
            angle_s4: PWM Servo angle 4 value (0 - 180)
        """
        try:
            if angle_s1 < 0 or angle_s1 > 180:
                angle_s1 = 255
            if angle_s2 < 0 or angle_s2 > 180:
                angle_s2 = 255
            if angle_s3 < 0 or angle_s3 > 180:
                angle_s3 = 255
            if angle_s4 < 0 or angle_s4 > 180:
                angle_s4 = 255
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_PWM_SERVO_ALL, \
                   int(angle_s1), int(angle_s2), int(angle_s3), int(angle_s4)]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.__logger.debug(f"ON all set_pwm_servo_all: executed {cmd}")
            time.sleep(self.__delay_time)
        except Exception as e:
            self.__logger.error(f'ON set_pwm_servo_all: {e}')
            pass
    
    def set_colorful_lamps(self, led_id, red, green, blue):
        """"
        RGB programmable light belt control, can be controlled individually or collectively, before control need to stop THE RGB light effect.
        
        Args:
            led_id: Control the CORRESPONDING numbered RGB lights (0, 13); if led_id=0xFF, controls all lights.
            red: indicating R value (0-255) of the color for the RGB led indicated for led_id.
            green: indicating G value (0-255) of the color for the RGB led indicated for led_id.
            blue: indicating B value (0-255) of the color for the RGB led indicated for led_id.
        """
        try:
            id = int(led_id) & 0xff
            r = int(red) & 0xff
            g = int(green) & 0xff
            b = int(blue) & 0xff
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_RGB, id, r, g, b]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.__logger.debug(f"ON set_colorful_lamps: executed {cmd}")
            time.sleep(self.__delay_time)
        except Exception as e:
            self.__logger.error(f'ON set_colorful_lamps: {e}')
            pass

    def set_colorful_effect(self, effect, speed=255, parm=255):
        """
        RGB programmable light band special effects display.

        Args:
            effect: Accepted values (0, 6) 
                0: stop light effect 
                1: running light 
                2: running horse light 
                3: breathing light 
                4: gradient light 
                5: starlight 
                6: power display 
            speed: values (1, 10), the smaller the value, the faster the speed changes
            parm: left blank, as an additional argument.  
                Usage 1: The color of breathing lamp can be modified by the effect of breathing lamp [0, 6]
        """
        try:
            eff = int(effect) & 0xff
            spe = int(speed) & 0xff
            par = int(parm) & 0xff
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_RGB_EFFECT, eff, spe, par]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.__logger.debug(f"ON set_colorful_effect: executed {cmd}")
            time.sleep(self.__delay_time)
        except Exception as e:
            self.__logger.error(f'ON set_colorful_effect: {e}')
            pass

    def set_motor(self, speed_1, speed_2, speed_3, speed_4):
        """
        Control PWM pulse of motor to control speed (speed measurement without encoder).

        Args:
            speed_1: Accepted range value (-100, 100) Set velocity for dc motor 1, sign indicates motor direction, if value is set to 0 motor stops
            speed_2: Accepted range value (-100, 100) Set velocity for dc motor 2, sign indicates motor direction, if value is set to 0 motor stops
            speed_3: Accepted range value (-100, 100) Set velocity for dc motor 3, sign indicates motor direction, if value is set to 0 motor stops
            speed_4: Accepted range value (-100, 100) Set velocity for dc motor 4, sign indicates motor direction, if value is set to 0 motor stops
        """
        if self.__debug:
                self.__logger.debug(f'Setting motors to: {speed_1},{speed_2},{speed_3},{speed_4}')
        try:
            t_speed_a = bytearray(struct.pack('b', self.__limit_motor_value(speed_1)))
            t_speed_b = bytearray(struct.pack('b', self.__limit_motor_value(speed_2)))
            t_speed_c = bytearray(struct.pack('b', self.__limit_motor_value(speed_3)))
            t_speed_d = bytearray(struct.pack('b', self.__limit_motor_value(speed_4)))
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_MOTOR,
                   t_speed_a[0], t_speed_b[0], t_speed_c[0], t_speed_d[0]]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.__logger.debug(f"ON set_motor: executed {cmd}")
            time.sleep(self.__delay_time)
        except Exception as e:
            self.__logger.error(f'ON set_motor: {e}')
            pass


    def set_car_run(self, state, speed, adjust=False):
        """
        Control the car forward, backward, left, right and other movements.

        Args:
            state: Accepted values 
                0 - stop 
                1 - forward
                2 - backward
                3 - left
                4 - right
                5 - spin left
                6 - spin right
            speed:  Accepted range value (-100, 100) Set velocity for car speed, sign indicates motor direction, if value is set to 0 car stops
            adjust: True activate the gyroscope auxiliary motion direction. False, the function is disabled. This function is not enabled
        """
        try:
            car_type = self.__car_type      # self.__CAR_TYPE
            if adjust:
                car_type = car_type | self.__CAR_ADJUST
            t_speed = bytearray(struct.pack('h', int(speed)))
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_CAR_RUN, \
                car_type, int(state&0xff), t_speed[0], t_speed[1]]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.__logger.debug(f"ON set_car_run: executed {cmd}")
            time.sleep(self.__delay_time)
        except Exception as e:
            self.__logger.error(f'ON set_car_run: {e}')
            pass

    def set_car_motion(self, v_x, v_y, v_z):
        """
        Car movement control
        Input ranges for each car type: 
            X3: v_x=[-1.0, 1.0], v_y=[-1.0, 1.0], v_z=[-5, 5]
            X3PLUS: v_x=[-0.7, 0.7], v_y=[-0.7, 0.7], v_z=[-3.2, 3.2]
            R2/R2L: v_x=[-1.8, 1.8], v_y=[-0.045, 0.045], v_z=[-3, 3]

        Args:
            v_x: Velocity value for car X axis
            v_y: Velocity value for car Y axis
            v_z: Velocity value for car Z axis
        """
        try:
            vx_parms = bytearray(struct.pack('h', int(v_x*1000)))
            vy_parms = bytearray(struct.pack('h', int(v_y*1000)))
            vz_parms = bytearray(struct.pack('h', int(v_z*1000)))
            # cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_MOTION, self.__CAR_TYPE, \
            #     vx_parms[0], vx_parms[1], vy_parms[0], vy_parms[1], vz_parms[0], vz_parms[1]]
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_MOTION, self.__car_type, \
                vx_parms[0], vx_parms[1], vy_parms[0], vy_parms[1], vz_parms[0], vz_parms[1]]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.__logger.debug(f"ON set_car_motion: executed {cmd}")
            time.sleep(self.__delay_time)
        except Exception as e:
            self.__logger.error(f'ON set_car_motion error: {e}')
            pass

    def set_pid_param(self, kp, ki, kd, forever=False):
        """
        Set PID parameter, PID parameter control will affect the set_car_motion function to control the speed change of the car.
        This parameter is optional by default.  
        Since permanent storage needs to be written into the chip flash, which takes a long time to operate, delay is added to avoid packet loss caused by MCU.
        Temporary effect fast response, single effective, data will not be maintained after restarting the single chip.

        Args:
            kp: Proportional constant, value range accepted (0, 10.0)
            ki: Integral constant, value range accepted (0, 10.0)
            kd: Derivative constant, value range accepted (0, 10.0)
            forever: True for permanent, False for temporary.  
        """
        try:
            state = 0
            if forever:
                state = 0x5F
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x0A, self.FUNC_SET_MOTOR_PID]
            if kp > 10 or ki > 10 or kd > 10 or kp < 0 or ki < 0 or kd < 0:
                self.__logger.warning(f"PID value must be:[0, 10.0]")
                return
            kp_params = bytearray(struct.pack('h', int(kp * 1000)))
            ki_params = bytearray(struct.pack('h', int(ki * 1000)))
            kd_params = bytearray(struct.pack('h', int(kd * 1000)))
            cmd.append(kp_params[0])  # low
            cmd.append(kp_params[1])  # high
            cmd.append(ki_params[0])  # low
            cmd.append(ki_params[1])  # high
            cmd.append(kd_params[0])  # low
            cmd.append(kd_params[1])  # high
            cmd.append(state)
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.__logger.debug(f"ON set_pid_param: executed {cmd}")
            time.sleep(self.__delay_time)
            if forever:
                time.sleep(0.1)
        except Exception as e:
            self.__logger.error(f'ON set_pid_param: {e}')
            pass
    
    def set_car_type(self, car_type):
        """
        Set car Type

        Args:
            car_type: Car type
        """
        if str(car_type).isdigit():
            # self.__CAR_TYPE = int(car_type) & 0xff
            self.__car_type = int(car_type) & 0xff
            # cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_SET_CAR_TYPE, self.__CAR_TYPE, 0x5F]
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_SET_CAR_TYPE, self.__car_type, 0x5F]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.__logger.debug(f"ON set_car_type: executed {cmd}")
            time.sleep(0.1)
        else:
            self.__logger.warning(f"ON set_car_type: input invalid")

    def set_uart_servo(self, servo_id, pulse_value, run_time=500):
        """
        Control bus steering gear.  
        
        Args:
            Servo_id: [1-255], indicating the ID of the steering gear to be controlled. 
            If ID =254, control all connected steering gear.  
            pulse_value: [96,4000] indicates the position to which the steering gear will run.  
            run_time: indicates the running time (ms). 
            The shorter the time, the faster the steering gear rotates.  
            The minimum value is 0 and the maximum value is 2000
        """
        try:
            if not self.__arm_ctrl_enable:
                return
            if servo_id < 1 or pulse_value < 96 or pulse_value > 4000 or run_time < 0:
                self.__logger.error(f"ON set_uart_servo({servo_id}, {pulse_value}, {run_time}): Input error.")
                return
            if run_time > 2000:
                run_time = 2000
            if run_time < 0:
                run_time = 0
            s_id = int(servo_id) & 0xff
            value = bytearray(struct.pack('h', int(pulse_value)))
            r_time = bytearray(struct.pack('h', int(run_time)))

            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_UART_SERVO, \
                s_id, value[0], value[1], r_time[0], r_time[1]]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.__logger.debug(f"ON set_uart_servo: ID({servo_id}), PULSE({int(pulse_value)}), executed {cmd}")
            time.sleep(self.__delay_time)
        except Exception as e:
            self.__logger.error(f'ON set_uart_servo: {e}')
            pass

    def set_uart_servo_angle(self, s_id, s_angle, run_time=500):
        """
        Set bus steering gear Angle interface 
        
        Args:
            s_id:[1,6] 
            s_angle: 1-4:[0, 180], 5:[0, 270], 6:[0, 180], 
                set steering gear to move to the Angle.  
            run_time: indicates the running time (ms). 
                The shorter the time, the faster the steering gear rotates.  
                The minimum value is 0 and the maximum value is 2000
        """
        try:
            if s_id == 1:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    self.__logger.warning(f"angle_1 set error: {s_angle} -> {value} out of limits (0, 180).")
            elif s_id == 2:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    self.__logger.warning(f"angle_2 set error: {s_angle} -> {value} out of limits (0, 180).")
            elif s_id == 3:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    self.__logger.warning(f"angle_3 set error: {s_angle} -> {value} out of limits (0, 180).")
            elif s_id == 4:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    self.__logger.warning(f"angle_4 set error: {s_angle} -> {value} out of limits (0, 180).")
            elif s_id == 5:
                if 0 <= s_angle <= 270:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    self.__logger.warning(f"angle_5 set error: {s_angle} -> {value} out of limits (0, 270).")
            elif s_id == 6:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    self.__logger.warning(f"angle_6 set error: {s_angle} -> {value} out of limits (0, 180).")
        except Exception as e:
            self.__logger.error(f'ON set_uart_servo_angle: ID({s_id}). {e}')
            pass

    def set_uart_servo_id(self, servo_id):
        """
        Set the bus servo ID (Use with caution).
        Before running this function, please confirm that only one bus actuator is connected. 
        Otherwise, all connected bus actuators will be set to the same ID, resulting in confusion 
        of control.

        Args:
            servo_id: Accepted value range (1-250).
        """
        try:
            if servo_id < 1 or servo_id > 250:
                self.__logger.warning(f"ON set_uart_servo_id: servo_id ({servo_id}) out of range, must be (1-250)")
                return
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_UART_SERVO_ID, int(servo_id)]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.__logger.debug(f"UART Servo_id: executed {cmd}")
            time.sleep(self.__delay_time)
        except Exception as e:
            self.__logger.error(f'ON set_uart_servo_id: {e}')
            pass

    def set_uart_servo_torque(self, enable):
        """
        Turn off/on the bus steering gear torque force.

        Args:  
            enable: If 0 or False, turn off the torque force of the steering gear, the steering gear can be turned by hand, but the command cannot control the rotation.
                If 1or True, turn on torque force, command can control rotation, can not turn steering gear by hand
        """
        try:
            if enable > 0:
                on = 1
            else:
                on = 0
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_UART_SERVO_TORQUE, on]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.__logger.debug(f"ON set_uart_servo_torque: executed {cmd}")
            time.sleep(self.__delay_time)
        except Exception as e:
            self.__logger.error(f'ON set_uart_servo_torque: {e}')
            pass

    def set_uart_servo_ctrl_enable(self, enable):
        """
        Set the control switch of the manipulator. 
        
        Args:
            enable: True Indicates that the control protocol is normally sent; 
            False indicates that the control protocol is not sent
        """
        if enable:
            self.__arm_ctrl_enable = True
        else:
            self.__arm_ctrl_enable = False

    def set_uart_servo_angle_array(self, angle_s=[90, 90, 90, 90, 90, 180], run_time=500):
        """"Angle of all steering gear of the manipulator is controlled"""
        try:
            if not self.__arm_ctrl_enable:
                return
            if 0 <= angle_s[0] <= 180 and 0 <= angle_s[1] <= 180 and 0 <= angle_s[2] <= 180 and \
                0 <= angle_s[3] <= 180 and 0 <= angle_s[4] <= 270 and 0 <= angle_s[5] <= 180:
                if run_time > 2000:
                    run_time = 2000
                if run_time < 0:
                    run_time = 0
                temp_val = [0, 0, 0, 0, 0, 0]
                for i in range(6):
                    temp_val[i] = self.__arm_convert_value(i+1, angle_s[i])
                    
                value_s1 = bytearray(struct.pack('h', int(temp_val[0])))
                value_s2 = bytearray(struct.pack('h', int(temp_val[1])))
                value_s3 = bytearray(struct.pack('h', int(temp_val[2])))
                value_s4 = bytearray(struct.pack('h', int(temp_val[3])))
                value_s5 = bytearray(struct.pack('h', int(temp_val[4])))
                value_s6 = bytearray(struct.pack('h', int(temp_val[5])))

                r_time = bytearray(struct.pack('h', int(run_time)))
                cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_ARM_CTRL, \
                       value_s1[0], value_s1[1], value_s2[0], value_s2[1], value_s3[0], value_s3[1], \
                       value_s4[0], value_s4[1], value_s5[0], value_s5[1], value_s6[0], value_s6[1], \
                       r_time[0], r_time[1]]
                cmd[2] = len(cmd) - 1
                checksum = sum(cmd, self.__COMPLEMENT) & 0xff
                cmd.append(checksum)
                self.ser.write(cmd)
                if self.__debug:
                    self.__logger.debug(f"ON set_uart_servo_arm_angle_array:")
                    self.__logger.debug(f"   Executed arm: {cmd}")
                    self.__logger.debug(f"   Executed value: {temp_val}")
                time.sleep(self.__delay_time)
            else:
                self.__logger.warning(f"angle_s input error: {angle_s}")
        except Exception as e:
            self.__logger.error(f'ON set_uart_servo_angle_array: {e}')
            pass

    def set_uart_servo_offset(self, servo_id):
        """
        Set the mid-bit deviation of the manipulator
        
        Args: 
            servo_id: 0 to 6, 0 Restore the factory default values
        """
        try:
            self.__arm_offset_id = 0xff
            self.__arm_offset_state = 0
            s_id = int(servo_id) & 0xff
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_ARM_OFFSET, s_id]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.__logger.debug(f"ON set_uart_servo_offset: executed {cmd}")
            time.sleep(self.__delay_time)
            for i in range(200):
                if self.__arm_offset_id == servo_id:
                    if self.__debug:
                        if self.__arm_offset_id == 0:
                            self.__logger.debug("ON set_uart_servo_offset: Arm reset offset value")
                        else:
                            self.__logger.debug(f"ON set_uart_servo_offset: Arm offset state {self.__arm_offset_id}, {self.__arm_offset_state}, {i}")
                    return self.__arm_offset_state
                time.sleep(.001)
            return self.__arm_offset_state
        except Exception as e:
            self.__logger.error(f'ON set_uart_servo_offset: {e}')
            pass

    def set_akm_default_angle(self, angle, forever=False):
        """
        Set the default Angle of akerman - car type (R2) front wheel. 
        Since permanent storage needs to be written into the chip flash, which takes a long time to operate, delay is added to avoid packet loss caused by MCU.
        Temporary effect fast response, single effective, data will not be maintained after restarting the single chip.
        
        Args:
            angle: Accepted values (60, 120)
            forever: True for permanent, False for temporary.
        """
        try:
            if int(angle) > 120 or int(angle) < 60:
                return
            id = self.__AKM_SERVO_ID
            state = 0
            if forever:
                state = 0x5F
                self.__akm_def_angle = angle
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_AKM_DEF_ANGLE, id, int(angle), state]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.__logger.debug(f"ON set_akm_default_angle: executed {cmd}")
            time.sleep(self.__delay_time)
            if forever:
                time.sleep(.1)
        except Exception as e:
            self.__logger.error(f'ON set_akm_default_angle: {e}')
            pass

    def set_akm_steering_angle(self, angle, ctrl_car=False):
        """
        Control the steering Angle of ackman type (R2) car relative to the default 
        
        Args:
            angle: Accepted values Angle (-45, 45); negative for left and positive for right.
            ctrl_car: False, only control the steering gear Angle, True control the steering gear Angle and modify the speed of the left and right motors.
        """
        try:
            if int(angle) > 45 or int(angle) < -45:
                return
            id = self.__AKM_SERVO_ID
            if ctrl_car:
                id = self.__AKM_SERVO_ID + 0x80
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_AKM_STEER_ANGLE, id, int(angle)&0xFF]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.__logger.debug(f"ON set_akm_steering_angle: executed {cmd}")
            time.sleep(self.__delay_time)
        except Exception as e:
            self.__logger.error(f'ON set_akm_steering_angle: {e}')
            pass

    def reset_flash_value(self):
        """ Reset the car flash saved data, restore the factory default value"""
        try:
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_RESET_FLASH, 0x5F]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.__logger.debug(f"ON reset_flash_value: executed{cmd}")
            time.sleep(self.__delay_time)
            time.sleep(.1)
        except Exception as e:
            self.__logger.error(f'ON reset_flash_value: {e}')
            pass
    
    def reset_car_state(self):
        """Reset car status, including parking, lights off, buzzer off"""
        try:
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_RESET_STATE, 0x5F]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                self.__logger.debug(f"ON reset_car_state: executed {cmd}")
            time.sleep(self.__delay_time)
        except Exception as e:
            self.__logger.error(f'ON reset_car_state: {e}')
            pass

    def clear_auto_report_data(self):
        """Clear the cache data automatically sent by the MCU"""
        self.__ax = self.__ay = self.__az = 0.0
        self.__gx = self.__gy = self.__gz = 0.0
        self.__mx = self.__my = self.__mz = 0.0
        self.__vx = self.__vy = self.__vz = 0.0
        self.__yaw = self.__roll = self.__pitch = 0.0
        self.__battery_voltage = 0

    def get_akm_default_angle(self):
        """Read the default angle of the front wheel servo of the Ackerman type (R2) car."""
        if not self.__akm_readed_angle:
            self.__request_data(self.FUNC_AKM_DEF_ANGLE, self.__AKM_SERVO_ID)
            akm_count = 0
            while True:
                if self.__akm_readed_angle:
                    break
                akm_count = akm_count + 1
                if akm_count > 100:
                    return -1
                time.sleep(.01)
        return self.__akm_def_angle

    def get_uart_servo_value(self, servo_id):
        """
        Read bus servo position parameters. 
        
        Args:
            servo_id: Desired servo id, accepted values (1-250) 
            return: read ID, current position parameters
        """
        try:
            if servo_id < 1 or servo_id > 250:
                self.__logger.warning("ON set_uart_servo_id: servo_id out of range, must be [1-250]")
                return
            self.__read_id = 0
            self.__read_val = 0
            self.__request_data(self.FUNC_UART_SERVO, int(servo_id) & 0xff)
            time.sleep(self.__delay_time)   # Me
            timeout = 15        # Me, old val 30
            while timeout > 0:
                if self.__read_id > 0:
                    return self.__read_id, self.__read_val
                timeout = timeout - 1
                #time.sleep(self.__delay_time)    #Me, old val time.sleep(.001)    
                time.sleep(.001)    
            return -1, -1
        except Exception as e:
            self.__logger.error(f'ON get_uart_servo_value: {e}')
            return -2, -2

    def get_uart_servo_angle(self, s_id):
        """
        Read the Angle of the bus steering gear.
         
        Args:
            s_id: indicates the ID number of the steering gear to be read, s_id values [1-6]
        """
        try:
            angle = -1
            read_id, value = self.get_uart_servo_value(s_id)
            if s_id == 1 and read_id == 1:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        self.__logger.warning(f"ON get_uart_servo_angle: read servo ({s_id}) out of range {angle} ")
                    angle = -1
            elif s_id == 2 and read_id == 2:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        self.__logger.warning(f"ON get_uart_servo_angle: read servo ({s_id}) out of range {angle} ")
                    angle = -1
            elif s_id == 3 and read_id == 3:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        self.__logger.warning(f"ON get_uart_servo_angle: read servo ({s_id}) out of range {angle} ")
                    angle = -1
            elif s_id == 4 and read_id == 4:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        self.__logger.warning(f"ON get_uart_servo_angle: read servo ({s_id}) out of range {angle} ")
                    angle = -1
            elif s_id == 5 and read_id == 5:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 270 or angle < 0:
                    if self.__debug:
                        self.__logger.warning(f"ON get_uart_servo_angle: read servo ({s_id}) out of range {angle} ")
                    angle = -1
            elif s_id == 6 and read_id == 6:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        self.__logger.warning(f"ON get_uart_servo_angle: read servo ({s_id}) out of range {angle} ")
                    angle = -1
            else:
                if self.__debug:
                    self.__logger.error(f"ON get_uart_servo_angle: read servo ({s_id}) error")
            if self.__debug:
                self.__logger.debug(f"ON get_uart_servo_angle: for ID({s_id}) request angle, Read ID ({read_id}), Value({value})")
            return angle
        except Exception as e:
            self.__logger.error(f'ON get_uart_servo_angle: {e}')
            return -2

    def get_uart_servo_angle_array(self):
        """
        Read the angles of three steering gear [xx, xx, xx, xx, xx, xx] at one time. 
        If one steering gear is wrong, that one is -1
        """
        try:
            angle = [-1, -1, -1, -1, -1, -1]
            self.__read_arm = [-1, -1, -1, -1, -1, -1]
            self.__read_arm_ok = 0
            self.__request_data(self.FUNC_ARM_CTRL, 1)
            timeout = 30
            while timeout > 0:
                if self.__read_arm_ok == 1:
                    for i in range(6):
                        if self.__read_arm[i] > 0:
                            angle[i] = self.__arm_convert_angle(i+1, self.__read_arm[i])
                    if self.__debug:
                        self.__logger.debug(f"ON get_uart_servo_angle_array: {30-timeout}, angles {angle}")
                    break
                timeout = timeout - 1
                time.sleep(.001)
            return angle
        except Exception as e:
            self.__logger.error(f'ON get_uart_servo_angle_array: {e}')
            return [-2, -2, -2, -2, -2, -2]

    def get_accelerometer_data(self):
        """
        Get accelerometer triaxial data, return a_x, a_y, a_z
        """
        # Why???
        a_x, a_y, a_z = self.__ax, self.__ay, self.__az
        return a_x, a_y, a_z

    def get_gyroscope_data(self):
        """Get the gyro triaxial data, return g_x, g_y, g_z"""
        # Why???
        g_x, g_y, g_z = self.__gx, self.__gy, self.__gz
        return g_x, g_y, g_z

    def get_magnetometer_data(self):
        """Get the magnetometer three-axis data and return m_x, m_y, m_z"""
        m_x, m_y, m_z = self.__mx, self.__my, self.__mz
        return m_x, m_y, m_z

    def get_imu_attitude_data(self, ToAngle=True):
        """
        Get the board attitude angle and return yaw, roll, pitch
        Args:
            ToAngle: True returns mesure in degrees, False returns radians.
        """
        if ToAngle:
            RtA = 57.2957795
            roll = self.__roll * RtA
            pitch = self.__pitch * RtA
            yaw = self.__yaw * RtA
        else:
            roll, pitch, yaw = self.__roll, self.__pitch, self.__yaw
        # self.__roll, self.__pitch, self.__yaw = 0.0, 0.0, 0.0
        return roll, pitch, yaw

    def get_motion_data(self):
        """Get the car speed, val_vx, val_vy, val_vz"""
        val_vx = self.__vx
        val_vy = self.__vy
        val_vz = self.__vz
        # self.__vx, self.__vy, self.__vz = 0.0, 0.0, 0.0
        return val_vx, val_vy, val_vz

    def get_battery_voltage(self):
        """Get the battery voltage"""
        vol = self.__battery_voltage / 10.0
        # self.__battery_voltage = 0
        return vol

    def get_motor_encoder(self):
        """Obtain data of four-channel motor encoder"""
        m1, m2, m3, m4 = self.__encoder_m1, self.__encoder_m2, self.__encoder_m3, self.__encoder_m4
        # self.__encoder_m1, self.__encoder_m2, self.__encoder_m3, self.__encoder_m4 = 0, 0, 0, 0
        return m1, m2, m3, m4

    def get_motion_pid(self):
        """Get the motion PID parameters of the dolly and return [kp, ki, kd]"""
        self.__kp1 = 0
        self.__ki1 = 0
        self.__kd1 = 0
        self.__pid_index = 0
        self.__request_data(self.FUNC_SET_MOTOR_PID, int(1))
        for i in range(20):
            if self.__pid_index > 0:
                kp = float(self.__kp1 / 1000.0)
                ki = float(self.__ki1 / 1000.0)
                kd = float(self.__kd1 / 1000.0)
                if self.__debug:
                    self.__logger.debug(f"ON get_motion_pid: {self.__pid_index}, {[kp, ki, kd]}, {i}")
                return [kp, ki, kd]
            time.sleep(.001)
        return [-1, -1, -1]


    def get_car_type_from_machine(self):
        """Gets the current car type from machine"""
        self.__request_data(self.FUNC_SET_CAR_TYPE)
        for i in range(0, 20):
            if self.__read_car_type != 0:
                car_type = self.__read_car_type
                self.__read_car_type = 0
                return car_type
            time.sleep(.001)
        return -1


    def get_version(self):
        """Get the underlying microcontroller version number, such as 1.1"""
        if self.__version_H == 0:
            self.__request_data(self.FUNC_VERSION)
            for i in range(0, 20):
                if self.__version_H != 0:
                    val = self.__version_H * 1.0
                    self.__version = val + self.__version_L / 10.0
                    if self.__debug:
                        self.__logger.debug(f"ON get_version: V{self.__version}, i:{i}")
                    return self.__version
                time.sleep(.001)
        else:
            return self.__version
        return -1
