#!/usr/bin/env python3
"""Validate and relay board and webcam state."""
import json
from time import monotonic

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

ORDER = [f'{r}{c}' for r in 'ABC' for c in range(1,4)] + [f'D{i}' for i in range(1,9)]
PIECES = [f'{color}_{i}' for color in ('red','yellow') for i in range(1,5)]
BEHAVIOUR_OFFLINE = {'available': False, 'two_open_palms': False, 'happy': False, 'angry': False}
BEHAVIOUR_TIMEOUT = 3.0


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


def validate_behaviour(state):
    if not isinstance(state, dict) or any(type(state.get(key)) is not bool for key in BEHAVIOUR_OFFLINE):
        raise ValueError('available, two_open_palms, happy and angry must be boolean')
    if state['happy'] and state['angry']:
        raise ValueError('happy and angry cannot both be true')
    if not state['available'] and any(state[key] for key in ('two_open_palms', 'happy', 'angry')):
        raise ValueError('unavailable webcam cannot report a detection')
    return {key: state[key] for key in BEHAVIOUR_OFFLINE}


class InformationReceiver(Node):
    def __init__(self):
        super().__init__('information_receiver')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state = None
        self.last_summary = None
        self.behaviour = None
        self.last_behaviour_time = 0.0
        self.publisher = self.create_publisher(String, 'information_state', qos)
        self.behaviour_publisher = self.create_publisher(String, 'information_behaviour', qos)
        self.mutual_publisher = self.create_publisher(String, 'mutual_communication', qos)
        self.subscription = self.create_subscription(String, 'position_state', self.receive, qos)
        self.behaviour_subscription = self.create_subscription(String, 'behaviour_state', self.receive_behaviour, qos)
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.behaviour_timer = self.create_timer(0.25, self.check_behaviour_timeout,
                                                 clock=steady_clock)
        self.set_behaviour(BEHAVIOUR_OFFLINE)
        self.behaviour_heartbeat = self.create_timer(1.0, self.publish_behaviour, clock=steady_clock)
        self.get_logger().info('Waiting for /position_state and /behaviour_state.')

    def receive(self,msg):
        try:
            self.state=validate_state(json.loads(msg.data))
        except (ValueError,TypeError,KeyError,AttributeError) as error:
            self.get_logger().warning(f'Rejected camera state: {error}')
            return
        self.publish_snapshot()
        summary=(tuple(self.state['d_occupancy']),tuple(self.state['d_valid']),self.state.get('complete'))
        if summary!=self.last_summary:
            self.get_logger().info(f'Received camera state: D={self.state["d_occupancy"]}, complete={self.state.get("complete")}')
            self.last_summary=summary

    def receive_behaviour(self, msg):
        try:
            state = validate_behaviour(json.loads(msg.data))
        except ValueError as error:
            self.get_logger().warning(f'Rejected behaviour state: {error}')
            return
        self.last_behaviour_time = monotonic()
        self.set_behaviour(state)

    def check_behaviour_timeout(self):
        if self.behaviour['available'] and monotonic() - self.last_behaviour_time >= BEHAVIOUR_TIMEOUT:
            self.set_behaviour(BEHAVIOUR_OFFLINE)

    def set_behaviour(self, state):
        if self.behaviour != state:
            self.behaviour = state
            self.publish_behaviour()
            self.publish_snapshot()

    def publish_behaviour(self):
        message = String(data=json.dumps(self.behaviour))
        self.mutual_publisher.publish(message)
        self.behaviour_publisher.publish(message)

    def publish_snapshot(self):
        if self.state is not None:
            snapshot = {**self.state, 'behaviour': self.behaviour}
            self.publisher.publish(String(data=json.dumps(snapshot)))


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
