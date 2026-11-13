#!/usr/bin/env python3
"""Host camera helper. Writes only one atomic latest-frame file; no ROS/control."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import time
import cv2
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--device', required=True)
    p.add_argument('--serial', required=True)
    p.add_argument('--frame-id', default='d455_color_optical_frame')
    p.add_argument('--output-dir', required=True)
    p.add_argument('--duration', type=float, default=0)
    a = p.parse_args()
    directory = Path(a.output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    device = Path(a.device).resolve()
    chain = Path('/sys/class/video4linux', device.name, 'device').resolve()
    serials = [q.read_text().strip() for parent in [chain, *chain.parents]
               if (q := parent/'serial').is_file()]
    if a.serial not in serials:
        raise ValueError('Selected camera USB serial mismatch')
    lock = (directory/'capture.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    stop = False
    def halt(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, halt)
    signal.signal(signal.SIGTERM, halt)
    cap = cv2.VideoCapture(str(device), cv2.CAP_V4L2)
    started = time.monotonic()
    frames = 0
    missed = 0
    try:
        if not cap.isOpened():
            raise RuntimeError('Cannot open camera')
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 15)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        actual = [cap.get(x) for x in [cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT,
                                       cv2.CAP_PROP_FPS, cv2.CAP_PROP_FOURCC]]
        if actual[:2] != [1280., 720.] or abs(actual[2]-15) > .1:
            raise RuntimeError(f'Unexpected camera mode: {actual}')
        print(json.dumps({'device': str(device), 'usb_serial': a.serial, 'actual_mode': actual,
                          'timestamp_basis': 'host_read_completion_not_hardware_exposure'}), flush=True)
        while not stop and (a.duration <= 0 or time.monotonic()-started < a.duration):
            ok, bgr = cap.read()
            stamp_ns = time.time_ns()
            if not ok:
                missed += 1
                if missed >= 3:
                    raise RuntimeError('Camera frame read failed three times')
                time.sleep(.05)
                continue
            missed = 0
            if bgr.shape[:2] != (720, 1280):
                raise RuntimeError('Camera resolution changed')
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            temp = directory/'latest.tmp'
            with temp.open('wb') as f:
                np.savez(f, image=gray, stamp_ns=np.int64(stamp_ns), seq=np.int64(frames),
                         frame_id=np.array(a.frame_id), serial=np.array(a.serial))
            os.replace(temp, directory/'latest.npz')
            frames += 1
    finally:
        cap.release()
        (directory/'capture_summary.json').write_text(json.dumps(
            {'frames': frames, 'elapsed_s': time.monotonic()-started, 'released': True}), encoding='utf-8')
        lock.close()


if __name__ == '__main__':
    main()
