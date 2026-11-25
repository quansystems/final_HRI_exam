#!/usr/bin/env python3
"""Validate a camera-based move and run its skills one at a time."""
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from ur_task_planner.srv import ExecuteSkill, MovePiece


class PieceCommandNode(Node):
    def __init__(self):
        super().__init__('piece_command', automatically_declare_parameters_from_overrides=True)
        self.positions = set(self.get_parameter('zone_names').value)
        self.objects = set(self.get_parameter('object_names').value)
        self.camera = None
        self.active = False
        self.sequence = 0
        self.camera_subscription = self.create_subscription(
            String, 'position_state', self.update_camera,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status_publisher = self.create_publisher(String, 'task_status', 20)
        self.skill_client = self.create_client(ExecuteSkill, 'execute_skill')
        self.move_service = self.create_service(MovePiece, 'move_piece', self.move_piece)

    def update_camera(self, message):
        try:
            state = json.loads(message.data)
            stamp = state['stamp']
            self.camera = (stamp['sec'] * 10**9 + stamp['nanosec'], state)
        except (ValueError, TypeError, KeyError):
            self.camera = None

    def publish_status(self, command, status, message='', **details):
        event = {**command, 'status': status, 'message': message, **details}
        self.status_publisher.publish(String(data=json.dumps(event, ensure_ascii=False)))

    def select_piece(self, command):
        color, source, destination = (command[key] for key in ('color', 'source', 'destination'))
        if source == destination:
            return None, 'SAME_POSITION', 'Ô nguồn và ô đích phải khác nhau.'
        if source not in self.positions or destination not in self.positions:
            return None, 'INVALID_POSITION', 'Chọn vị trí từ A1–C3 hoặc D1–D8.'
        if color not in ('red', 'yellow'):
            return None, 'INVALID_COLOR', 'Màu hợp lệ là red/đỏ hoặc yellow/vàng.'
        if self.camera is None:
            return None, 'CAMERA_UNAVAILABLE', 'Chưa nhận được trạng thái camera.'
        stamp, state = self.camera
        age = self.get_clock().now().nanoseconds - stamp
        if stamp <= 0 or not 0 <= age <= 2 * 10**9:
            return None, 'CAMERA_UNAVAILABLE', 'Dữ liệu camera chưa sẵn sàng hoặc đã cũ.'
        cells = state.get('cells', {})
        source_color = cells.get(source)
        if source_color == 'empty':
            return None, 'SOURCE_EMPTY', f'Ô {source} đang trống.'
        if source_color not in ('red', 'yellow'):
            return None, 'CAMERA_UNAVAILABLE', f'Camera chưa xác nhận ô {source}.'
        if source_color != color:
            return None, 'COLOR_MISMATCH', f'Camera thấy màu {source_color} tại {source}.'
        destination_color = cells.get(destination)
        if destination_color in ('red', 'yellow'):
            return None, 'DESTINATION_OCCUPIED', f'Ô {destination} đã có quân.'
        if destination_color != 'empty':
            return None, 'CAMERA_UNAVAILABLE', f'Camera chưa xác nhận ô {destination} trống.'
        piece = state.get('occupants', {}).get(source)
        if (piece not in self.objects or not piece.startswith(color + '_')
                or state.get('piece_valid', {}).get(piece) is not True):
            return None, 'CAMERA_UNAVAILABLE', f'Camera chưa xác nhận ID quân tại {source}.'
        return piece, '', ''

    def move_piece(self, request, response):
        self.sequence += 1
        color = request.color.strip().lower()
        command = {
            'id': self.sequence,
            'color': {'đỏ': 'red', 'vàng': 'yellow'}.get(color, color),
            'source': request.source.strip().upper(),
            'destination': request.destination.strip().upper(),
        }
        if self.active:
            response.status = 'BUSY'
            response.message = 'Robot đang xử lý lệnh trước.'
        else:
            piece, response.status, response.message = self.select_piece(command)
            if piece and not self.skill_client.service_is_ready():
                response.status = 'EXECUTOR_UNAVAILABLE'
                response.message = 'Dịch vụ thực thi chưa sẵn sàng.'
            elif piece:
                self.active = True
                response.accepted = True
                response.status = 'WAITING'
                response.message = 'Đã nhận lệnh.'
                # The executor re-observes immediately before pick and checks the source.
                steps = [('pick', piece, command['source']),
                         ('place', piece, command['destination']),
                         ('home', '', ''), ('observe', '', '')]
        self.publish_status(command, response.status, response.message)
        if response.accepted:
            self.run_step(command, piece, steps)
        return response

    def run_step(self, command, piece, steps):
        skill, object_name, zone = steps[0]
        self.publish_status(command, 'EXECUTING', step=skill, piece=piece)
        request = ExecuteSkill.Request(skill=skill, object_name=object_name, zone=zone)
        try:
            future = self.skill_client.call_async(request)
        except Exception as error:
            self.finish(command, 'FAILED', str(error), step=skill)
            return
        future.add_done_callback(
            lambda result: self.step_finished(command, piece, steps, result))

    def step_finished(self, command, piece, steps, future):
        skill = steps[0][0]
        try:
            result = future.result()
            if result.status != 'SUCCESS':
                self.finish(command, result.status, result.message, step=skill)
                return
            if len(steps) == 1:
                state = json.loads(result.scene_state)
                zones = state.get('zones', {})
                destination = zones.get(command['destination'], [])
                # IDs assigned by color tracking can change when the arm covers the board.
                # The requested move is confirmed by the color at the destination and an empty source.
                if (state.get('observed') and zones.get(command['source']) == []
                        and len(destination) == 1
                        and destination[0].startswith(command['color'] + '_')):
                    self.finish(command, 'DONE', 'Camera đã xác nhận quân ở ô đích.', piece=piece)
                else:
                    self.finish(command, 'VERIFICATION_FAILED',
                                'Camera chưa xác nhận quân ở ô đích.', piece=piece)
                return
        except (RuntimeError, ValueError, TypeError, AttributeError) as error:
            self.finish(command, 'FAILED', str(error), step=skill)
            return
        self.run_step(command, piece, steps[1:])

    def finish(self, command, status, message, **details):
        self.active = False
        self.publish_status(command, status, message, **details)


def main():
    rclpy.init()
    node = PieceCommandNode()
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
