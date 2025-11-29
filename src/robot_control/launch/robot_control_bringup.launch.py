#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Paths
    rplidar_launch = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_a1_launch.py'
    )
    

    laser_tf_launch = os.path.join(
        get_package_share_directory('robot_control'),
        'launch',
        'laser_tf.launch.py'
    )

    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    # Launch Description
    return LaunchDescription([

        # Motor bridge node
        Node(
            package="robot_control",
            executable="motor_bridge",
            name="motor_bridge",
            output="screen"
        ),

        # Encoder Odom node
        Node(
            package="robot_control",
            executable="encoder_odom",
            name="encoder_odom",
            output="screen"
        ),

        # RPLIDAR launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch)
        ),

        # Laser TF launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_tf_launch)
        ),

        # Navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch)
        ),

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch)
        )
    ])
