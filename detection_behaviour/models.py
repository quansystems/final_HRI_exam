"""Fetch the published model weights into a local cache."""

import hashlib
import os
from pathlib import Path
from tempfile import NamedTemporaryFile
from urllib.request import urlopen


ZOO = 'https://media.githubusercontent.com/media/opencv/opencv_zoo/main/models/'
YUNET = 'face_detection_yunet_2023mar.onnx'
EXPRESSION = 'enet_b0_8_va_mtl.onnx'
BASELINE_EXPRESSION = 'facial_expression_recognition_mobilefacenet_2022july.onnx'
GESTURE = 'gesture_recognizer.task'
MODELS = {
    YUNET: (f'{ZOO}face_detection_yunet/{YUNET}',
            '8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4'),
    BASELINE_EXPRESSION: (f'{ZOO}facial_expression_recognition/{BASELINE_EXPRESSION}',
                 '4f61307602fc089ce20488a31d4e4614e3c9753a7d6c41578c854858b183e1a9'),
    EXPRESSION: ('https://raw.githubusercontent.com/av-savchenko/face-emotion-recognition/main/'
                 f'models/affectnet_emotions/onnx/{EXPRESSION}',
                 'c43e056ad388d4a8dc911832b8291435b2af537f967e5870ebd731574ec7e812'),
    GESTURE: ('https://storage.googleapis.com/mediapipe-models/gesture_recognizer/'
              'gesture_recognizer/float16/1/gesture_recognizer.task',
              '97952348cf6a6a4915c2ea1496b4b37ebabc50cbbf80571435643c455f2b0482'),
}
DEFAULT_CACHE = Path(os.environ.get('XDG_CACHE_HOME') or Path.home() / '.cache') / 'detection_behaviour'


def ensure_models(directory, *, include_baseline=False):
    directory.mkdir(parents=True, exist_ok=True)
    names = (YUNET, EXPRESSION, GESTURE)
    if include_baseline:
        names += (BASELINE_EXPRESSION,)
    for name in names:
        url, expected_hash = MODELS[name]
        path = directory / name
        if path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == expected_hash:
            continue
        with urlopen(url, timeout=45) as response:
            data = response.read()
        if hashlib.sha256(data).hexdigest() != expected_hash:
            raise ValueError(f'SHA-256 của {name} khác phiên bản đã kiểm thử.')
        with NamedTemporaryFile(dir=directory, delete=False) as temporary:
            temporary.write(data)
        Path(temporary.name).replace(path)
