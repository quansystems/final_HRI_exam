#!/usr/bin/env python3
"""Publish measured cube poses; the executor owns the manipulation state."""
import math
import json
import time

import cv2
from cv_bridge import CvBridge
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformException, TransformListener
try:
    from ur_task_planner.msg import ObservedScene
except ImportError:
    ObservedScene = None

from .cube_perception import detect_cubes, rotation_matrix
from .board_perception import BoardPerception


class SceneObserverNode(Node):
    def __init__(self):
        super().__init__('camera_detection', automatically_declare_parameters_from_overrides=True)
        # Keep per-frame image processing from competing with Gazebo rendering for worker threads.
        cv2.setNumThreads(1)
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.sizes = {name: self.get_parameter(f'objects.{name}.size').value
                      for name in self.get_parameter('object_names').value}
        self.height = float(self.get_parameter('table_surface_z').value)
        self.board = None
        if self.has_parameter('board_enabled') and self.get_parameter('board_enabled').value:
            self.board = BoardPerception(
                center=self.get_parameter('board_center').value,
                cell_size=float(self.get_parameter('board_cell_size').value),
                surface_z=float(self.get_parameter('board_surface_z').value),
                piece_size=float(self.get_parameter('board_piece_size').value),
                staging_first=(self.get_parameter('staging_first_position').value
                               if self.has_parameter('staging_enabled') and self.get_parameter('staging_enabled').value else None),
                staging_spacing=float(self.get_parameter('staging_spacing').value)
                                if self.has_parameter('staging_spacing') else .055,
                staging_direction=self.get_parameter('staging_direction').value
                                  if self.has_parameter('staging_direction') else (1., 0.))
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.board_publisher = self.create_publisher(String, 'board_state', latched)
            self.position_publisher = self.create_publisher(String, 'position_state', latched)
            self.d_publisher = self.create_publisher(Int32MultiArray, 'd_occupancy', latched)
            self.marker_publisher = self.create_publisher(MarkerArray, 'board_markers', latched)
            self.last_frame_time = 0.
            self.stale_timer = self.create_timer(.5, self.check_stale_board)
        self.info = None
        if not self.has_parameter('reliable_camera'):
            self.declare_parameter('reliable_camera', False)
        image_qos = (QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
                     if self.get_parameter('reliable_camera').value else qos_profile_sensor_data)
        self.info_sub = self.create_subscription(
            CameraInfo, '/depth_camera/camera_info', self.set_info, image_qos)
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/depth_camera/image', qos_profile=image_qos)
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/depth_camera/depth_image', qos_profile=image_qos)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=5, slop=0.04)
        self.sync.registerCallback(self.observe)
        self.publisher = self.create_publisher(ObservedScene, 'observed_scene', 1) if ObservedScene else None
        self.last_warning = 0
        if self.board:
            self.get_logger().info('Camera detection ready: /position_state, /d_occupancy, /board_markers.')

    def set_info(self, msg):
        self.info = msg

    def observe(self, rgb_msg, depth_msg):
        if self.info is None:
            return
        try:
            if (rgb_msg.header.frame_id != depth_msg.header.frame_id
                    or self.info.header.frame_id != rgb_msg.header.frame_id):
                raise ValueError('RGB/depth/CameraInfo must share the registered optical frame')
            tf = self.buffer.lookup_transform(
                'gazebo_world', rgb_msg.header.frame_id, Time.from_msg(rgb_msg.header.stamp))
            q, t = tf.transform.rotation, tf.transform.translation
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'rgb8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.float64)
            if depth_msg.encoding == '16UC1':
                depth *= 0.001
            elif depth_msg.encoding != '32FC1':
                raise ValueError(f'unsupported depth encoding: {depth_msg.encoding}')
            rotation = rotation_matrix([q.x, q.y, q.z, q.w])
            translation = np.array([t.x, t.y, t.z])
            if self.board:
                state, poses = self.board.observe(rgb, depth, self.info.k, rotation, translation)
                self.last_frame_time = time.monotonic()
                self.publish_board(state, rgb_msg.header.stamp)
            else:
                poses = detect_cubes(rgb, depth, self.info.k, rotation,
                                     translation, self.sizes, self.height)
        except (TransformException, ValueError, RuntimeError) as error:
            now = self.get_clock().now().nanoseconds
            if now - self.last_warning > 5_000_000_000:
                self.get_logger().warn(f'Observation unavailable: {error}')
                self.last_warning = now
            return
        if self.publisher is None:
            return
        msg = ObservedScene()
        msg.header.stamp = rgb_msg.header.stamp
        msg.header.frame_id = 'gazebo_world'
        for name, (centre, yaw) in poses.items():
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = centre
            pose.orientation.z = math.sin(yaw/2)
            pose.orientation.w = math.cos(yaw/2)
            msg.object_names.append(name)
            msg.poses.append(pose)
        self.publisher.publish(msg)

    def check_stale_board(self):
        if self.board and time.monotonic() - self.last_frame_time > 1.5:
            self.publish_board({
                'cells': {cell: 'unknown' for cell in self.board.centers},
                'occupants': {cell: 'unknown' for cell in self.board.centers},
                'complete': False,
                'counts': {'red': 0, 'yellow': 0, 'empty': 0, 'unknown': len(self.board.centers), 'invalid': 0},
                'position_order': self.board.position_order,
                'd_occupancy': list(self.board.d_occupancy), 'd_valid': [False]*8,
                'piece_locations': self.board.piece_locations,
                'piece_valid': {name:False for name in self.board.piece_names},
            }, self.get_clock().now().to_msg())

    def publish_board(self, state, stamp):
        state.update(frame_id='gazebo_world', stamp={'sec': stamp.sec, 'nanosec': stamp.nanosec})
        message = String(data=json.dumps(state))
        self.board_publisher.publish(message)
        self.position_publisher.publish(message)
        self.d_publisher.publish(Int32MultiArray(data=state["d_occupancy"]))
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        def add(kind, namespace, position, scale, color, text=''):
            marker = Marker()
            marker.header.frame_id = 'gazebo_world'
            marker.header.stamp = stamp
            marker.ns = namespace
            marker.id = len(markers.markers)
            marker.type, marker.action = kind, Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
            marker.pose.orientation.w = 1.
            marker.scale.x, marker.scale.y, marker.scale.z = scale
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            marker.text = text
            markers.markers.append(marker)

        side, z = self.board.cell_size, self.board.surface_z
        for index, (cell, (x, y)) in enumerate(self.board.centers.items()):
            gray = .78 if index % 2 == 0 else .4
            add(Marker.CUBE, 'board', (x, y, z-.001), (side-.001, side-.001, .002),
                (gray, gray, gray, 1.))
            value = state['cells'][cell]
            if value in ('red', 'yellow'):
                size = self.board.piece_size
                color = (1., 0., 0., 1.) if value == 'red' else (1., 1., 0., 1.)
                add(Marker.CUBE, 'pieces', (x, y, z+size/2), (size, size, size), color)
            label = cell + (' ?' if value in ('unknown', 'invalid') else '')
            add(Marker.TEXT_VIEW_FACING, 'cells', (x, y, z+self.board.piece_size+.012),
                (0., 0., .011), (.1, .1, .1, 1.), label)
        cx, cy = self.board.center
        for i, letter in enumerate('ABC'):
            add(Marker.TEXT_VIEW_FACING, 'rows', (cx-1.8*side, cy+(1-i)*side, z+.005),
                (0., 0., .016), (.05, .05, .05, 1.), letter)
        for i in range(3):
            add(Marker.TEXT_VIEW_FACING, 'columns', (cx+(i-1)*side, cy+1.8*side, z+.005),
                (0., 0., .016), (.05, .05, .05, 1.), str(i+1))
        self.marker_publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SceneObserverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
