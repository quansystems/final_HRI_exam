#!/usr/bin/env python3
"""Display task events and expose them to other ROS consumers."""
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class Notification(Node):
    def __init__(self):
        super().__init__('notification')
        self.publisher = self.create_publisher(
            String, 'notification',
            QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.subscription = self.create_subscription(String, 'task_status', self.receive, 20)

    def receive(self, message):
        try:
            event = json.loads(message.data)
            status, command_id = event['status'], event['id']
        except (ValueError, TypeError, KeyError):
            self.get_logger().warning('Invalid task status message.')
            return
        detail = event.get('message') or event.get('step', '')
        text = f'Lệnh {command_id} [{status}] {detail}'
        if status in ('WAITING', 'EXECUTING', 'DONE'):
            self.get_logger().info(text)
        else:
            self.get_logger().warning(text)
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = Notification()
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
