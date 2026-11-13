"""Collect stationary 5x sharpened corners for the 50 mm ID 4 trial."""

import argparse
from collections import Counter, deque
import hashlib
import json
from pathlib import Path
import signal
import sys
import time

import cv2
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from probe_core import Detector


def save_json(path, row):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(row, allow_nan=False))
    temp.replace(path)


def corners_at_scale(detector, gray, scale, offset):
    image = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_LANCZOS4)
    image = cv2.addWeighted(image, 1.6, cv2.GaussianBlur(image, (0, 0), scale * .55), -.6, 0)
    tags = [tag for tag in detector.detect(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)) if tag['hamming'] == 0]
    if len(tags) != 1 or tags[0]['id'] != 4:
        raise ValueError(f'expected exactly one ID 4 at scale {scale}')
    corners = np.asarray(tags[0]['corners_lb_rb_rt_lt_px'])
    return (corners + .5) / scale - .5 + offset


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--latest-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--duration', type=float, required=True)
    parser.add_argument('--frame-interval-s', type=float, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--serial', required=True)
    parser.add_argument('--roi', type=int, nargs=4, required=True, metavar=('X', 'Y', 'W', 'H'))
    args = parser.parse_args()
    x, y, width, height = args.roi
    if min(x, y) < 0 or min(width, height) < 32 or x + width > 1280 or y + height > 720:
        parser.error('ROI is outside the 1280x720 image')
    manifest = json.loads(args.manifest.read_text())
    pins = manifest['source_sha256']
    for path, digest in pins.items():
        if hashlib.sha256((ROOT / path).read_bytes()).hexdigest() != digest:
            raise ValueError(f'changed source: {path}')
    cfg = yaml.safe_load((ROOT / 'config/155_chest.yaml').read_text())
    cfg['tag_ids'] = [4]
    cv2.setNumThreads(1)
    detector = Detector(cfg)
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, stop)

    out = args.output_dir
    window = deque(maxlen=30)
    counts = Counter()
    last = 0
    deadline = time.monotonic() + args.duration
    saved = False
    # Match the region used for the independent static capture.
    offset = np.array([x, y])
    with (out / 'visual_measurements.jsonl').open('x') as log:
        while not stopped and time.monotonic() < deadline:
            with np.load(args.latest_dir / 'latest.npz', allow_pickle=False) as frame:
                stamp = int(frame['stamp_ns'])
                if stamp == last:
                    time.sleep(.006)
                    continue
                last = stamp
                image = frame['image']
                serial = str(frame['serial'])
                if (image.shape != (720, 1280) or serial != args.serial
                        or str(frame['frame_id']) != 'd455_color_optical_frame'):
                    raise ValueError('wrong camera or image mode')
            x, y = offset
            roi = image[y:y + height, x:x + width]
            try:
                corners = corners_at_scale(detector, roi, 5, offset)
            except ValueError as exc:
                if window and stamp - window[-1][0] >= 800_000_000:
                    window.clear()
                counts['rejected'] += 1
                log.write(json.dumps({'state': 'rejected', 'image_stamp_ns': stamp, 'error': str(exc)}) + '\n')
                continue
            save_json(out / 'tag_live.json', {'state': 'valid', 'image_stamp_ns': stamp, 'serial': serial,
                                             'corners': corners.tolist(), 'tag_id': 4})
            while window and stamp - window[0][0] > 3_500_000_000:
                window.popleft()
            window.append((stamp, corners))
            if len(window) < window.maxlen:
                counts['collecting'] += 1
                continue
            medians = np.median([row[1] for row in window], axis=0)
            row = {'state': 'valid', 'image_stamp_ns': stamp, 'window_start_stamp_ns': window[0][0],
                   'source_sha256': pins, 'base_heading_used': False, 'tag_id': 4,
                   'corners_by_scale': {'5': medians.tolist()}}
            save_json(out / 'visual_live.json', row)
            counts['valid'] += 1
            log.write(json.dumps(row, allow_nan=False) + '\n')
            log.flush()
            if not saved:
                if not cv2.imwrite(str(out / 'initial_raw.png'), image):
                    raise RuntimeError('could not archive the initial tag image')
                saved = True
    save_json(out / 'worker_summary.json', dict(counts))


if __name__ == '__main__':
    main()
