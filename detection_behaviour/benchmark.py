"""Tune on annotated examples and evaluate the frozen settings on a separate split."""

import argparse
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
import csv
from dataclasses import asdict, replace
from itertools import product
import json
from pathlib import Path
from random import Random
from statistics import median, quantiles
from time import perf_counter
from urllib.parse import urlencode
from urllib.request import urlopen

import cv2
import numpy as np

from detector import Confirmation, Detector, Evidence, Settings, resize_frame
from models import BASELINE_EXPRESSION, DEFAULT_CACHE, EXPRESSION, ensure_models


CREMA = 'https://raw.githubusercontent.com/CheyneyComputerScience/CREMA-D/master/'
VIDEOS = 'https://media.githubusercontent.com/media/CheyneyComputerScience/CREMA-D/master/VideoFlash/'
HAGRID = 'https://datasets-server.huggingface.co/filter?'
LABELS = ('angry', 'happy', None)


def download(url, path):
    if not path.exists():
        for attempt in range(3):
            try:
                with urlopen(url, timeout=30) as response:
                    path.write_bytes(response.read())
                break
            except OSError:
                if attempt == 2:
                    raise
    return path


def face_cases(cache):
    votes = download(f'{CREMA}processedResults/tabulatedVotes.csv', cache / 'crema_votes.csv')
    clips = defaultdict(lambda: defaultdict(list))
    with votes.open(newline='') as file:
        for row in csv.DictReader(file):
            if not row[''].startswith('2') or float(row['agreement']) < .55:
                continue
            name = row['fileName']
            label = {'A': 'angry', 'H': 'happy'}.get(row['emoVote'])
            if ':' not in row['emoVote']:
                clips[name.split('_')[0]][label].append(name)

    rng = Random(63)
    actors = [actor for actor in sorted(clips) if all(len(clips[actor][label]) >= 2 for label in LABELS)]
    rng.shuffle(actors)
    cases = [(split, label, rng.sample(sorted(clips[actor][label]), 2))
             for split, group in [('tune', actors[:24]), ('test', actors[24:36])]
             for actor in group for label in LABELS]
    directory = cache / 'crema'
    directory.mkdir(exist_ok=True)
    jobs = [(f'{VIDEOS}{name}.flv', directory / f'{name}.flv')
            for _, _, names in cases for name in names]
    with ThreadPoolExecutor(8) as pool:
        list(pool.map(lambda job: download(*job), jobs))
    return [(split, label, directory / f'{name}.flv') for split, label, names in cases for name in names]


def sample_clip(path):
    capture = cv2.VideoCapture(str(path))
    frames = []
    try:
        fps = capture.get(cv2.CAP_PROP_FPS)
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            frames.append(frame)
    finally:
        capture.release()
    if len(frames) < 6 or not fps > 0:
        raise RuntimeError(f'Cannot sample video {path}')
    interval = round(fps / 5)
    start = max(0, (len(frames) - 4 * interval) // 2)
    return frames[start:start + 5 * interval:interval]


def hand_cases(cache):
    """Pair real one-hand photos in one frame. Keep source photos out of both splits."""
    rng = Random(37)
    groups = {}
    directory = cache / 'hagrid'
    directory.mkdir(exist_ok=True)
    for kind, label in [('palm', 8), ('fist', 2), ('peace', 9), ('four', 3)]:
        params = {'dataset': 'cj-mills/hagrid-sample-30k-384p', 'config': 'default',
                  'split': 'train', 'where': f'"label"={label}', 'offset': 71, 'length': 64}
        with urlopen(HAGRID + urlencode(params), timeout=30) as response:
            rows = json.load(response)['rows']
        paths = [directory / f'{row["row_idx"]}.jpg' for row in rows]
        jobs = [(row['row']['image']['src'], path) for row, path in zip(rows, paths)]
        with ThreadPoolExecutor(8) as pool:
            list(pool.map(lambda job: download(*job), jobs))
        rng.shuffle(paths)
        groups[kind] = (paths[:32], paths[32:])

    def picture(kind, split_index):
        path = rng.choice(groups[kind][split_index])
        image = cv2.imread(str(path))
        height, width = image.shape[:2]
        scale = min(290 / width, 340 / height)
        image = cv2.resize(image, (round(width * scale), round(height * scale)), interpolation=cv2.INTER_AREA)
        tile = np.full((360, 320, 3), 110, np.uint8)
        y, x = (360 - image.shape[0]) // 2, (320 - image.shape[1]) // 2
        tile[y:y + image.shape[0], x:x + image.shape[1]] = image
        return tile

    for split_index, split in enumerate(('tune', 'test')):
        for index in range(36):
            left = picture('palm', split_index)
            right = cv2.flip(picture('palm', split_index), 1)
            negative = picture(('fist', 'peace', 'four')[index % 3], split_index)
            background = np.full_like(left, 110)
            frames = ((True, [(left, right)] * 3),
                      (False, [(left, negative)] * 3),
                      (False, [(left, background), (background, right), (left, background)]),
                      (False, [(negative, background)] * 3))
            for target, sequence in frames:
                pair = [np.hstack((a, b)) for a, b in sequence]
                gain = (1., .65, .84)[index % 3]
                yield split, target, [cv2.convertScaleAbs(image, alpha=gain * factor)
                                      for image, factor in zip(pair, (.94, 1., .97))]


def collect(cache, model_dir, expression_model, *, include_hands):
    filename = 'evidence_v3.json' if include_hands else f'{Path(expression_model).stem}_evidence.json'
    path = cache / filename
    if path.exists():
        return json.loads(path.read_text())
    clips = face_cases(cache)
    print(f'{len(clips)} face clips; extracting model scores', flush=True)
    data = {'face': [], 'hand': []}
    with Detector(model_dir, video=False, expression_model=expression_model) as detector:
        for split, label, clip in clips:
            observations = []
            for frame in sample_clip(clip):
                scores, _ = detector.face_evidence(frame)
                observations.append(scores.tolist() if scores is not None else None)
            data['face'].append((split, label, observations))

        if include_hands:
            print('Extracting gesture scores from HaGRID sequences', flush=True)
            for split, label, sequence in hand_cases(cache):
                data['hand'].append((split, label, [detector.hand_evidence(frame) for frame in sequence]))

    path.write_text(json.dumps(data))
    return data


def observations(data, split):
    face = [(label, [Evidence(expression=np.array(frame) if frame is not None else None)
                     for frame in sequence]) for group, label, sequence in data['face'] if group == split]
    hand = [(label, [[Evidence(palms=frame[index]) for frame in sequence] for index in (0, 1)])
            for group, label, sequence in data['hand'] if group == split]
    return face, hand


def scores(sequences, settings, task, labels):
    pairs = []
    for expected, frames in sequences:
        confirmation = Confirmation(settings)
        if task == 1:
            frames = frames[int(settings.refine_hands)]
        for frame in frames:
            output = confirmation.update(frame)
        pairs.append((expected, output[task]))
    accuracy = sum(ground == prediction for ground, prediction in pairs) / len(pairs)
    by_label = {}
    for label in labels:
        true = sum(ground == prediction == label for ground, prediction in pairs)
        positives = sum((ground == label) + (prediction == label) for ground, prediction in pairs)
        by_label[str(label)] = {'correct': true, 'support': sum(ground == label for ground, _ in pairs),
                                'f1': 2 * true / positives if positives else 0.}
    return {'n': len(pairs), 'accuracy': accuracy,
            'macro_f1': sum(item['f1'] for item in by_label.values()) / len(labels), 'classes': by_label}


def five_rounds(tune_face, tune_hand):
    def fit(candidates, samples, task, labels):
        def macro_f1(config):
            return scores(samples, config, task, labels)['macro_f1']
        return max(candidates, key=macro_f1)

    base = Settings(angry_score=0., happy_score=0., expression_margin=0., angry_gain=1., happy_gain=1.,
                    face_frames=1, palm_score=.01, finger_angle=0., finger_spread=0.,
                    thumb_spread=0., hand_frames=1, refine_hands=False)
    calibration = fit((replace(base, face_frames=n, angry_gain=g, angry_score=a, happy_score=h)
                       for n, g, a, h in product([1, 3, 5], [1., 1.5, 2., 3., 4.],
                                                 [0., .40, .52], [0., .50])),
                      tune_face, 0, LABELS)
    hand_crops = fit((replace(calibration, refine_hands=refine, palm_score=p)
                     for refine, p in product([False, True], [.01, .40, .50, .60])),
                    tune_hand, 1, (False, True))
    stable_base = replace(hand_crops, hand_frames=2)
    stability = fit((replace(stable_base, support_score=p, finger_angle=a, finger_spread=s, thumb_spread=t)
                     for p, a, s, t in product([.60, .72, .82, .90], [0., 138., 145., 152., 159.],
                                               [1.05, 1.15, 1.25, 1.35], [.65, .8, .95])),
                    tune_hand, 1, (False, True))
    return [('baseline', base, BASELINE_EXPRESSION), ('expression_model', base, EXPRESSION),
            ('expression_calibration', calibration, EXPRESSION), ('hand_regions', hand_crops, EXPRESSION),
            ('hand_spread_and_stability', stability, EXPRESSION)]


def latency(model_dir, image, settings):
    with Detector(model_dir) as detector:
        for i in range(5):
            detector.inspect(image, i * 200, refine=settings.refine_hands)
        times = []
        for i in range(5, 45):
            start = perf_counter()
            detector.inspect(image, i * 200, refine=settings.refine_hands)
            times.append((perf_counter() - start) * 1000)
    return {'median_ms': median(times), 'p95_ms': quantiles(times, n=20)[18]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, default=DEFAULT_CACHE / 'benchmark')
    parser.add_argument('--model-dir', type=Path, default=DEFAULT_CACHE)
    args = parser.parse_args()
    args.cache.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(2)
    ensure_models(args.model_dir, include_baseline=True)
    baseline = collect(args.cache, args.model_dir, BASELINE_EXPRESSION, include_hands=True)
    alternate = collect(args.cache, args.model_dir, EXPRESSION, include_hands=False)
    baseline_tune, tune_hand = observations(baseline, 'tune')
    baseline_test, test_hand = observations(baseline, 'test')
    tune_face, _ = observations(alternate, 'tune')
    test_face, _ = observations(alternate, 'test')
    report = {'rounds': []}
    profiles = five_rounds(tune_face, tune_hand)
    for name, settings, model in profiles:
        result = {'name': name, 'face_model': model, 'settings': asdict(settings),
                  'face_tune': scores(baseline_tune if model == BASELINE_EXPRESSION else tune_face, settings, 0, LABELS),
                  'hand_tune': scores(tune_hand, settings, 1, (False, True)),
                  'face_test': scores(baseline_test if model == BASELINE_EXPRESSION else test_face, settings, 0, LABELS),
                  'hand_test': scores(test_hand, settings, 1, (False, True))}
        report['rounds'].append(result)
        print(json.dumps(result), flush=True)

    deployed = Settings()
    report['deployed'] = {'settings': asdict(deployed),
                          'face_test': scores(test_face, deployed, 0, LABELS),
                          'hand_test': scores(test_hand, deployed, 1, (False, True))}
    print('Deployed:', json.dumps(report['deployed']), flush=True)
    # Use a fixed face frame for the CPU timing with both webcam models enabled.
    clip = sorted((args.cache / 'crema').glob('*.flv'))[0]
    report['latency'] = latency(args.model_dir, resize_frame(sample_clip(clip)[0], 640), deployed)
    print('Latency:', report['latency'], flush=True)
    (args.cache / 'report.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
