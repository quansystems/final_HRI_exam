"""CPU inference for the two facial expressions and the simultaneous hand gesture."""

from collections import deque
from dataclasses import dataclass

import cv2
import mediapipe as mp
import numpy as np

from models import BASELINE_EXPRESSION, EXPRESSION, GESTURE, YUNET


# Five eye/nose/mouth points expected by the OpenCV Zoo MobileFaceNet model.
FACE_TEMPLATE = np.array([[38.2946, 51.6963], [73.5318, 51.5014], [56.0252, 71.7366],
                          [41.5493, 92.3655], [70.7299, 92.2041]], dtype=np.float32)
EXPRESSIONS = {0: 'angry', 3: 'happy'}
HSE_ORDER = [0, 2, 3, 4, 5, 6, 7, 1]  # Put anger and happiness at the same indices as the baseline.
RGB_MEAN = np.array([.485, .456, .406], dtype=np.float32)
RGB_STD = np.array([.229, .224, .225], dtype=np.float32)
FINGERS = np.array([[5, 6, 8], [9, 10, 12], [13, 14, 16], [17, 18, 20]])


@dataclass(frozen=True)
class Settings:
    angry_score: float = 0.
    happy_score: float = 0.
    expression_margin: float = 0.
    angry_gain: float = 4.
    happy_gain: float = 1.
    face_frames: int = 5
    palm_score: float = .01
    finger_angle: float = 0.
    finger_spread: float = 0.
    thumb_spread: float = 0.
    support_score: float = 1.
    hand_frames: int = 2
    refine_hands: bool = True


@dataclass
class Evidence:
    expression: np.ndarray | None = None
    face_box: tuple[int, int, int, int] | None = None
    palms: tuple[tuple[float, float, float, float], ...] = ()


def expression_label(scores, settings):
    if scores is None:
        return None
    scores = scores.copy()
    scores[0] *= settings.angry_gain
    scores[3] *= settings.happy_gain
    scores /= scores.sum()
    winner = int(np.argmax(scores))
    label = EXPRESSIONS.get(winner)
    if label is None:
        return None
    threshold = settings.angry_score if label == 'angry' else settings.happy_score
    margin = scores[winner] - np.partition(scores, -2)[-2]
    return label if scores[winner] >= threshold and margin >= settings.expression_margin else None


def both_palms_open(palms, settings):
    recognized = [palm for palm in palms if palm[0] >= settings.palm_score]
    if len(recognized) == 2:
        return True
    if len(recognized) != 1 or recognized[0][0] < settings.support_score or settings.finger_angle <= 0.:
        return False
    # Geometry can recover one open hand, and must be supported by a strong
    # Open_Palm classification of the other hand in this same frame.
    return sum(score < settings.palm_score and angle >= settings.finger_angle
               and spread >= settings.finger_spread and thumb >= settings.thumb_spread
               for score, angle, spread, thumb in palms) == 1


class Confirmation:
    """Smooth scores from the same face and confirm simultaneous hands across frames."""

    def __init__(self, settings):
        self.settings = settings
        self.faces = deque(maxlen=settings.face_frames)
        self.previous_box = None
        self.gesture_count = 0

    def update(self, evidence):
        if evidence.expression is None or face_changed(self.previous_box, evidence.face_box):
            self.faces.clear()
        self.previous_box = evidence.face_box
        if evidence.expression is not None:
            self.faces.append(evidence.expression)
        face_scores = np.mean(self.faces, axis=0) if self.faces else None
        expression = expression_label(face_scores, self.settings)
        self.gesture_count = (self.gesture_count + 1
                              if both_palms_open(evidence.palms, self.settings) else 0)
        return expression, self.gesture_count >= self.settings.hand_frames


def face_changed(previous, current):
    if previous is None or current is None:
        return False
    x1, y1, w1, h1 = previous
    x2, y2, w2, h2 = current
    overlap = (max(0, min(x1 + w1, x2 + w2) - max(x1, x2)) *
               max(0, min(y1 + h1, y2 + h2) - max(y1, y2)))
    return overlap < .3 * min(w1 * h1, w2 * h2)


def hand_geometry(landmarks):
    points = np.array([(point.x, point.y, point.z) for point in landmarks])
    start, middle, end = (points[FINGERS[:, i]] for i in range(3))
    a, b = start - middle, end - middle
    norms = np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1)
    palm_width = np.linalg.norm(points[5] - points[17])
    if np.any(norms < 1e-8) or palm_width < 1e-8:
        return 0., 0., 0.
    cosines = (a * b).sum(axis=1) / norms
    angle = np.degrees(np.arccos(np.clip(cosines, -1., 1.))).min()
    spread = np.linalg.norm(points[8] - points[20]) / palm_width
    thumb = np.linalg.norm(points[4] - points[5]) / palm_width
    return float(angle), float(spread), float(thumb)


class Detector:
    def __init__(self, directory, *, video=True, expression_model=EXPRESSION):
        self.face = cv2.FaceDetectorYN.create(str(directory / YUNET), '', (320, 240),
                                              score_threshold=.75, top_k=1000)
        self.expression = cv2.dnn.readNetFromONNX(str(directory / expression_model))
        self.use_baseline_expression = expression_model == BASELINE_EXPRESSION
        self.gesture_model = str(directory / GESTURE)
        self.video = video
        self.hands = self.create_recognizer(video)
        self.hand_crops = None

    def create_recognizer(self, video):
        vision = mp.tasks.vision
        mode = vision.RunningMode.VIDEO if video else vision.RunningMode.IMAGE
        return vision.GestureRecognizer.create_from_options(vision.GestureRecognizerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=self.gesture_model),
            running_mode=mode, num_hands=2, min_hand_detection_confidence=.40,
            min_hand_presence_confidence=.45, min_tracking_confidence=.50))

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.hands.close()
        if self.hand_crops is not None:
            self.hand_crops.close()

    def face_evidence(self, frame):
        height, width = frame.shape[:2]
        self.face.setInputSize((width, height))
        _, faces = self.face.detect(frame)
        if faces is None:
            return None, None
        # Keep the closest visible face as the focus of the webcam demo.
        face = max(faces, key=lambda row: row[2] * row[3])
        if min(face[2:4]) < 40:
            return None, None
        box = tuple(map(int, face[:4]))
        blob = self.expression_input(frame, face, box)
        if blob is None:
            return None, None
        self.expression.setInput(blob)
        logits = self.expression.forward().reshape(-1)
        if not self.use_baseline_expression:
            # The final two model outputs are continuous valence and arousal.
            logits = logits[HSE_ORDER]
        weights = np.exp(logits - logits.max())
        return weights / weights.sum(), box

    def expression_input(self, frame, face, box):
        if not self.use_baseline_expression:
            x, y, width, height = box
            crop = frame[max(0, y):max(0, y + height), max(0, x):max(0, x + width)]
            if not crop.size:
                return None
            resized = cv2.resize(crop, (224, 224))
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.
            return ((rgb - RGB_MEAN) / RGB_STD).transpose(2, 0, 1)[None]

        points = face[4:14].reshape(5, 2)
        transform, _ = cv2.estimateAffinePartial2D(points, FACE_TEMPLATE, method=cv2.LMEDS)
        if transform is None:
            return None
        aligned = cv2.warpAffine(frame, transform, (112, 112))
        return cv2.dnn.blobFromImage(aligned, 1 / 127.5, (112, 112),
                                     (127.5, 127.5, 127.5), swapRB=True)

    @staticmethod
    def hand_candidates(frame, recognizer, timestamp_ms=None, left=0):
        image = mp.Image(image_format=mp.ImageFormat.SRGB,
                         data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        result = (recognizer.recognize_for_video(image, timestamp_ms) if timestamp_ms is not None
                  else recognizer.recognize(image))
        candidates = []
        height, width = frame.shape[:2]
        for categories, coords, landmarks in zip(result.gestures, result.hand_landmarks,
                                                 result.hand_world_landmarks):
            score = next((c.score for c in categories if c.category_name == 'Open_Palm'), 0.)
            center = np.array([left + coords[9].x * width, coords[9].y * height])
            size = np.hypot((coords[5].x - coords[17].x) * width, (coords[5].y - coords[17].y) * height)
            candidates.append((center, size, (score, *hand_geometry(landmarks))))
        return candidates

    def hand_evidence(self, frame, timestamp_ms=0, *, refine=True):
        timestamp = timestamp_ms if self.video else None
        candidates = self.hand_candidates(frame, self.hands, timestamp)
        base = tuple(candidate[2] for candidate in candidates)
        if not refine or sum(palm[0] >= .45 for palm in base) == 2:
            return base, base

        if self.video and self.hand_crops is None:
            self.hand_crops = self.create_recognizer(video=False)
        recognizer = self.hand_crops if self.video else self.hands
        width = frame.shape[1]
        for left, right in [(0, round(.62 * width)), (round(.38 * width), width)]:
            candidates.extend(self.hand_candidates(frame[:, left:right], recognizer, left=left))
        distinct = []
        for center, size, palm in sorted(candidates, key=lambda candidate: candidate[2][0], reverse=True):
            if all(np.linalg.norm(center - known) > max(12., .9 * max(size, known_size))
                   for known, known_size, _ in distinct):
                distinct.append((center, size, palm))
        return base, tuple(candidate[2] for candidate in distinct)

    def inspect(self, frame, timestamp_ms=0, *, refine=True):
        expression, box = self.face_evidence(frame)
        _, palms = self.hand_evidence(frame, timestamp_ms, refine=refine)
        return Evidence(expression, box, palms)


def resize_frame(frame, width):
    height, actual_width = frame.shape[:2]
    if actual_width <= width:
        return frame
    return cv2.resize(frame, (width, round(height * width / actual_width)),
                      interpolation=cv2.INTER_AREA)
