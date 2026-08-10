#!/usr/bin/env python3
"""
hw_bringup.launch.py
=====================
Levanta TODO lo necesario antes de correr hw_pid_battery.py:
  - robot_state_publisher (TF completo, parseando el xacro a URDF en tiempo real).
  - rosmaster_bridge_node (ruedas + brazo, con sus parámetros PID y puerto).
  - ydlidar_ros2_driver (sensor real, publicando en best_effort).
  - rf2o_laser_odometry (localización, forzada a escuchar en best_effort).
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 1. Ruta y parseo del Xacro para el robot_state_publisher
    xacro_path = os.path.join(
        get_package_share_directory('omni_dofbot_description'),
        'urdf', 'omni_dofbot_no_control.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}]
    )

    # 2. Nodo Puente con el STM32 (Motores)
    bridge_node = Node(
        package='rx3_robot_bridge',
        executable='rosmaster_bridge',
        parameters=[{
            'com': LaunchConfiguration('serial_port'),
            'car_type': 1,
            'kp': 1.5, 'ki': 0.08, 'kd': 0.5,
        }],
        output='screen'
    )

    # 3. Driver del LiDAR
    ydlidar_params = os.path.join(
        get_package_share_directory('rx3_robot_bridge'),
        'config', 'ydlidar_x4.yaml')

    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[ydlidar_params],
    )

    # 4. Nodo de Odometría (Con el parche de QoS inyectado)
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0
        }],
    )

    # Construcción final del Launch
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyCH341USB1'),
        robot_state_publisher_node,
        bridge_node,
        ydlidar_node,
        rf2o_node,
    ])