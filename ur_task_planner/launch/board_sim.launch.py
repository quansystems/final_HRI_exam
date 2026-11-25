"""Start the board, camera, RViz and local robot command nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare('ur_task_planner')
    return LaunchDescription([
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('random_seed', default_value=''),
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ur_simulation_gz'), 'launch', 'ur3e_rg2_table_depth.launch.py'])),
            launch_arguments={
                'launch_rviz': 'false',
                'gazebo_gui': LaunchConfiguration('gazebo_gui'),
                'random_seed': LaunchConfiguration('random_seed'),
            }.items())]),
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                package, 'launch', 'task_planner.launch.py'])),
            launch_arguments={'launch_rviz': 'false', 'launch_observer': 'false'}.items())]),
        Node(package='camera_detection', executable='camera_detection_node.py', output='screen',
             parameters=[PathJoinSubstitution([package, 'config', 'scene_depth.yaml']),
                         {'use_sim_time': True}]),
        Node(package='infomation_communication', executable='information_receiver_node.py',
             output='screen', parameters=[{'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', name='board_rviz',
             condition=IfCondition(LaunchConfiguration('launch_rviz')),
             arguments=['-d', PathJoinSubstitution([package, 'rviz', 'board.rviz'])],
             parameters=[{'use_sim_time': True}], additional_env={'LC_ALL': 'C.UTF-8'}),
    ])
