"""Publish webcam detections without sending the webcam image."""

import json
from time import monotonic

import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String


class BehaviourPublisher:
    def __enter__(self):
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('detection_behaviour')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.node.create_publisher(String, 'behaviour_state', qos)
        self.last_output = None
        self.last_publish = 0.0
        return self

    def update(self, expression, gesture):
        now = monotonic()
        output = (expression, gesture)
        if output != self.last_output or now - self.last_publish >= 1.0:
            self.publish(True, expression, gesture)
            self.last_output = output
            self.last_publish = now

    def publish(self, available, expression, gesture):
        state = {'available': available, 'two_open_palms': gesture,
                 'happy': expression == 'happy', 'angry': expression == 'angry'}
        self.publisher.publish(String(data=json.dumps(state, separators=(',', ':'))))

    def __exit__(self, *exc):
        try:
            if rclpy.ok():
                self.publish(False, None, False)
                self.publisher.wait_for_all_acked(Duration(seconds=0.25))
        finally:
            self.node.destroy_node()
            rclpy.try_shutdown()
