#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_robot = get_package_share_directory('robot_control')
    pkg_rplidar = get_package_share_directory('rplidar_ros')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # Correct SLAM params path
    default_slam_yaml = os.path.join(
        pkg_robot,
        'config',
        'slam_params.yaml'
    )

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value=default_slam_yaml,
            description='Full path to SLAM parameters.'
        ),

        Node(
            package="robot_control",
            executable="motor_bridge",
            name="motor_bridge",
            output="screen"
        ),

        Node(
            package="robot_control",
            executable="encoder_odom",
            name="encoder_odom",
            output="screen"
        ),

      IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(pkg_rplidar, "launch", "rplidar_a1_launch.py")
    ),
        launch_arguments={
        'serial_port': '/dev/ttyUSB1',
        'serial_baudrate': '115200'
    }.items()
    ),


        # TF between base & laser REQUIRED for SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_robot, 'launch', 'laser_tf.launch.py')
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
            )
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file, {'use_sim_time': False}]
        )
    ])
