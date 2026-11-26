"""Run expression and simultaneous open-palm detection on a webcam."""

import argparse
from contextlib import nullcontext
import json
from pathlib import Path
from time import monotonic, sleep

import cv2

from detector import Confirmation, Detector, Settings, resize_frame
from models import DEFAULT_CACHE, ensure_models


TITLE = 'Detection Behaviour'
FACE_TEXT = {'happy': 'Vui ve', 'angry': 'Tuc gian'}


def show(frame, box, expression, gesture):
    if box is not None and expression is not None:
        x, y, width, height = box
        color = (0, 185, 0) if expression == 'happy' else (20, 75, 255)
        cv2.rectangle(frame, (x, y), (x + width, y + height), color, 2)
        cv2.putText(frame, FACE_TEXT[expression], (max(0, x), max(25, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, .68, color, 2)
    if gesture:
        cv2.putText(frame, 'Hai tay xoe', (12, frame.shape[0] - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, .75, (0, 210, 255), 2)
    cv2.imshow(TITLE, frame)
    return (cv2.waitKey(1) & 0xFF) in (27, ord('q')) or cv2.getWindowProperty(TITLE, cv2.WND_PROP_VISIBLE) < 1


def run_camera(args, detector, publisher=None):
    capture = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    try:
        if not capture.isOpened():
            raise RuntimeError(f'Không mở được webcam {args.camera}. Kiểm tra /dev/video{args.camera}.')
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, round(args.width * .75))
        capture.set(cv2.CAP_PROP_FPS, args.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        settings = Settings()
        confirmation = Confirmation(settings)
        last_output = (None, False)
        next_frame = start = monotonic()
        while args.seconds is None or monotonic() - start < args.seconds:
            sleep(max(0., next_frame - monotonic()))
            now = monotonic()
            next_frame = now + 1. / args.fps
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError(f'Webcam {args.camera} ngừng gửi khung hình.')
            frame = resize_frame(frame, args.width)
            evidence = detector.inspect(frame, round((now - start) * 1000), refine=settings.refine_hands)
            output = confirmation.update(evidence)
            expression, gesture = output
            if publisher is not None:
                publisher.update(expression, gesture)
            if output != last_output:
                print(json.dumps({'expression': expression, 'two_open_palms': gesture}), flush=True)
                last_output = output
            if not args.headless and show(frame, evidence.face_box, expression, gesture):
                break
    finally:
        capture.release()
        if not args.headless:
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--fps', type=float, default=5.)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--seconds', type=float, help='Thời gian chạy thử')
    parser.add_argument('--headless', action='store_true', help='Chỉ in kết quả JSON')
    parser.add_argument('--ros', action='store_true', help='Phát trạng thái lên /behaviour_state qua ROS 2')
    parser.add_argument('--model-dir', type=Path, default=DEFAULT_CACHE)
    args = parser.parse_args()
    if args.camera < 0 or not 1. <= args.fps <= 15. or not 320 <= args.width <= 1280:
        parser.error('--camera phải >= 0, --fps từ 1 đến 15 và --width từ 320 đến 1280')
    if args.seconds is not None and not 0. < args.seconds < float('inf'):
        parser.error('--seconds phải là số dương hữu hạn')

    cv2.setNumThreads(2)
    try:
        ros_context = nullcontext()
        if args.ros:
            try:
                from ros_publisher import BehaviourPublisher
            except ImportError as error:
                parser.exit(1, f'--ros cần ROS 2 Humble trong terminal hiện tại. Source /opt/ros/humble/setup.bash.\n{error}\n')
            ros_context = BehaviourPublisher()
        ensure_models(args.model_dir)
        with Detector(args.model_dir) as detector, ros_context as publisher:
            run_camera(args, detector, publisher)
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, ValueError) as error:
        parser.exit(1, f'{error}\n')


if __name__ == '__main__':
    main()
