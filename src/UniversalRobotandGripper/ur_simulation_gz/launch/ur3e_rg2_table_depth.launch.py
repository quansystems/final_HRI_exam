"""UR3e/RG2, a 3x3 board and a registered overhead RGB-D camera."""
import tempfile
from uuid import uuid4
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PIECE_NAMES = tuple(f'{color}_{i}' for color in ('red', 'yellow') for i in range(1, 5))


def randomize_board(world, seed):
    """Place eight pieces along the Y axis outside the board."""
    centers = [(-.17, .1925 - i*.055) for i in range(8)]
    for name, (x, y) in zip(PIECE_NAMES, centers):
        world.find(f"model[@name='{name}']/pose").text = f'{x} {y} .826 0 0 0'


def static_transform(name, parent, child, pose):
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name=name,
        arguments=[item for flag, value in zip(
            ('--x', '--y', '--z', '--roll', '--pitch', '--yaw'), pose)
                   for item in (flag, str(value))] +
                  ['--frame-id', parent, '--child-frame-id', child],
        parameters=[{'use_sim_time': True}],
    )


def launch_simulation(context):
    package = FindPackageShare('ur_simulation_gz')
    source = PathJoinSubstitution([package, 'world', 'ur_table_depth.sdf']).perform(context)
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    # Camera TF comes from the sensor model, so pose changes cannot drift.
    camera_pose = world.find("model[@name='depth_camera']/pose").text.split()
    seed_text = LaunchConfiguration('random_seed').perform(context).strip()
    if seed_text:
        randomize_board(world, int(seed_text))
    temporary_world = tempfile.TemporaryDirectory(prefix='ur3e_board_')
    world_file = temporary_world.name + '/board.sdf'
    try:
        tree.write(world_file, encoding='utf-8', xml_declaration=True)
    except Exception:
        temporary_world.cleanup()
        raise
    return [
        RegisterEventHandler(OnShutdown(
            on_shutdown=lambda event, context: temporary_world.cleanup())),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                package, 'launch', 'ur3e_rg2_table.launch.py'])),
            launch_arguments={
                'world_file': world_file,
                'launch_rviz': LaunchConfiguration('launch_rviz'),
                'gazebo_gui': LaunchConfiguration('gazebo_gui'),
                'controller_spawner_timeout': LaunchConfiguration('controller_spawner_timeout'),
                'gazebo_clock_topic': f'/world/{world.attrib["name"]}/clock',
            }.items()),
        static_transform('depth_camera_pose', 'gazebo_world', 'depth_camera_link', camera_pose),
    ]


def generate_launch_description():
    package = FindPackageShare('ur_simulation_gz')
    gazebo_partition = LaunchConfiguration('gazebo_partition')
    return LaunchDescription([
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('controller_spawner_timeout', default_value='120'),
        DeclareLaunchArgument('random_seed', default_value='',
                              description='Fixed staging row D1-D8 (seed retained for compatibility).'),
        DeclareLaunchArgument(
            'gazebo_partition', default_value=f'ur3e_board_{uuid4().hex}',
            description='Gazebo Transport partition shared by this simulator and its bridges.',
        ),
        # A bridge left from an older launch must not subscribe to this world's camera or clock.
        SetEnvironmentVariable('IGN_PARTITION', gazebo_partition),
        SetEnvironmentVariable('GZ_PARTITION', gazebo_partition),
        LogInfo(msg=['Board Gazebo Transport partition: ', gazebo_partition]),
        OpaqueFunction(function=launch_simulation),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='depth_camera_bridge',
             parameters=[{'use_sim_time': True, 'config_file': PathJoinSubstitution([
                 package, 'config', 'depth_camera_bridge.yaml'])}], output='screen'),
        # Fortress point cloud XYZ is +X forward; image headers remain optical.
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='depth_camera_points_bridge',
             arguments=['/depth_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'],
             parameters=[{'use_sim_time': True, 'override_frame_id': 'depth_camera_link'}], output='screen'),
        static_transform('gazebo_to_robot_world', 'gazebo_world', 'world', [-.35, 0, .81, 0, 0, 0]),
        static_transform('depth_camera_optical_pose', 'depth_camera_link', 'depth_camera_optical_frame',
                         [0, 0, 0, -1.5707963267948966, 0, -1.5707963267948966]),
    ])
