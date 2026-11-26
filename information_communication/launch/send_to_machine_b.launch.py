"""Run the information relay on the shared ROS 2 network."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('discovery_server', description='Fast DDS server address and UDP port'),
        DeclareLaunchArgument('domain_id', default_value='30', description='ROS domain shared with machine B'),
        Node(
            package='information_communication',
            executable='information_receiver_node.py',
            output='screen',
            additional_env={
                'ROS_DOMAIN_ID': LaunchConfiguration('domain_id'),
                'ROS_LOCALHOST_ONLY': '0',
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                'ROS_DISCOVERY_SERVER': LaunchConfiguration('discovery_server'),
                'ROS_SUPER_CLIENT': 'TRUE',
            },
        ),
    ])
