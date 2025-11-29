from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', '/home/pg/ros2_ws/src/robot_control/config',
            '-configuration_basename', 'cartographer.lua'
        ],
        remappings=[
            ('/scan', '/scan')  # change if your lidar topic is different
        ]
    )

   
    return LaunchDescription([cartographer])
