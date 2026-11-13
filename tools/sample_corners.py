"""Read 5x Lanczos ID 4 corners from a capture_latest.py frame directory."""

import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from probe_core import Detector


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--latest-dir", type=Path, required=True)
    parser.add_argument("--serial", required=True)
    parser.add_argument("--roi", type=int, nargs=4, required=True, metavar=("X", "Y", "W", "H"))
    parser.add_argument("--duration", type=float, default=10.)
    parser.add_argument("--max-hamming", type=int, choices=(0, 1, 2, 3), default=1)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    x, y, width, height = args.roi
    if min(x, y) < 0 or min(width, height) < 32 or x + width > 1280 or y + height > 720:
        parser.error("ROI is outside the 1280x720 image")
    if not 0 < args.duration <= 60:
        parser.error("duration must be within 60 seconds")

    detector = Detector(dict(family="tag36h11", tag_ids=[4], threads=2,
                             maxhamming=args.max_hamming, decimate=1.0))
    cv2.setNumThreads(1)
    source = args.latest_dir / "latest.npz"
    last_stamp = -1
    deadline = time.monotonic() + args.duration
    with args.output.open("x", encoding="utf-8") as out:
        while time.monotonic() < deadline:
            if not source.exists():
                time.sleep(.02)
                continue
            with np.load(source, allow_pickle=False) as frame:
                stamp = int(frame["stamp_ns"])
                if stamp == last_stamp or not 0 <= time.time_ns() - stamp < 350_000_000:
                    time.sleep(.01)
                    continue
                image = frame["image"]
                if (str(frame["serial"]) != args.serial or image.shape != (720, 1280)
                        or str(frame["frame_id"]) != "d455_color_optical_frame"):
                    raise ValueError("Unexpected camera or image mode")
            last_stamp = stamp
            up = cv2.resize(image[y:y + height, x:x + width], None, fx=5, fy=5, interpolation=cv2.INTER_LANCZOS4)
            up = cv2.addWeighted(up, 1.6, cv2.GaussianBlur(up, (0, 0), 2.75), -.6, 0)
            tags = detector.detect(cv2.cvtColor(up, cv2.COLOR_GRAY2BGR))
            for tag in tags:
                points = np.asarray(tag["corners_lb_rb_rt_lt_px"])
                tag["corners_original_px"] = ((points + .5) / 5 - .5 + [x, y]).tolist()
            out.write(json.dumps({"stamp_ns": stamp, "tags": tags}, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
