#!/usr/bin/env python3
"""Receive camera state, retain validity flags, and publish a validated snapshot."""
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

ORDER = [f'{r}{c}' for r in 'ABC' for c in range(1,4)] + [f'D{i}' for i in range(1,9)]
PIECES = [f'{color}_{i}' for color in ('red','yellow') for i in range(1,5)]


def validate_state(state):
    if state.get('position_order') != ORDER:
        raise ValueError('position_order must contain A1-C3, D1-D8')
    occupancy = state.get('d_occupancy')
    if not isinstance(occupancy,list) or len(occupancy)!=8 or any(type(v) is not int or v not in (0,1) for v in occupancy):
        raise ValueError('d_occupancy must contain eight binary integers')
    if set(state.get('cells',{})) != set(ORDER):
        raise ValueError('cells must describe all 17 positions')
    for name in PIECES:
        values=state.get('piece_locations',{}).get(name)
        if not isinstance(values,list) or len(values)!=17 or any(type(v) is not int or v not in (0,1) for v in values) or sum(values)>1:
            raise ValueError(f'invalid location array for {name}')
    if len(state.get('d_valid',[]))!=8 or any(type(v) is not bool for v in state['d_valid']):
        raise ValueError('d_valid must contain eight boolean flags')
    if set(state.get('piece_valid',{}))!=set(PIECES) or any(type(v) is not bool for v in state['piece_valid'].values()):
        raise ValueError('piece_valid must describe all eight pieces')
    return state


class InformationReceiver(Node):
    def __init__(self):
        super().__init__('information_receiver')
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state=None
        self.last_summary=None
        self.publisher=self.create_publisher(String,'information_state',qos)
        self.subscription=self.create_subscription(String,'position_state',self.receive,qos)
        self.get_logger().info('Waiting for /position_state from camera_detection.')

    def receive(self,msg):
        try:
            self.state=validate_state(json.loads(msg.data))
        except (ValueError,TypeError,KeyError,AttributeError) as error:
            self.get_logger().warning(f'Rejected camera state: {error}')
            return
        self.publisher.publish(String(data=json.dumps(self.state)))
        summary=(tuple(self.state['d_occupancy']),tuple(self.state['d_valid']),self.state.get('complete'))
        if summary!=self.last_summary:
            self.get_logger().info(f'Received camera state: D={self.state["d_occupancy"]}, complete={self.state.get("complete")}')
            self.last_summary=summary


def main():
    rclpy.init()
    node=InformationReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    main()
