from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
import os

def generate_launch_description():
    default_yaml = os.path.join(
        os.getenv('HOME', '/home/pg'),
        'ros2_ws/src/robot_control/config/slam_params.yaml'
    )

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_yaml,
            description='Full path to the SLAM parameters file.'
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': False}
            ]
        )
    ])
