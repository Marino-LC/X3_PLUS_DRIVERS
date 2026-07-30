#!/usr/bin/env python3
"""
hw_bringup.launch.py
=====================
Levanta TODO lo necesario antes de correr hw_pid_battery.py:
  - robot_state_publisher (TF estático base_footprint -> laser_link, etc.).
  - rosmaster_bridge_node (ruedas + brazo, Ruta B).
  - ydlidar_ros2_driver (sensor real).
  - rf2o_laser_odometry (localización, remapeada a /odom).
  - rviz2 (Visualización en tiempo real).
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

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

    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ydlidar_ros2_driver'),
                        'launch', 'ydlidar_launch.py')),
        launch_arguments={'lidar_type': '0'}.items()
    )


    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/odom',
            'publish_tf' : True,
            'base_frame_id' : 'base_footprint',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 20.0
        }],
        # --- LA MAGIA ESTÁ AQUÍ ---
        ros_arguments=[
            '--remap', '/scan:=/scan',  # (Opcional, pero buena práctica)
            '--qos-profile-overrides', '/scan:best_effort' # Forzamos a rf2o a aceptar BEST_EFFORT
        ]
    )

    # NUEVO NODO: RViz2 para visualización
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyCH341USB0'),
        robot_state_publisher_node,
        bridge_node,
        ydlidar_launch,
        rf2o_node,
        rviz_node, # <-- Añadido al lanzamiento
    ])