"""Calibrate and estimate a parked pose from AprilTag corner observations."""

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "motion_trial_20261109"))
from tag_corner_servo_v2 import Geometry, rz
from tag_pixel_v4_single_tag50_pitch_anchor import Tag50PitchStart


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def save(value, path):
    data = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)
    if path:
        with path.open("x", encoding="utf-8") as stream:
            stream.write(data + "\n")
    print(data)


def stationary_corners(path, method, hamming):
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    corners = []
    for row in rows:
        tags = row["tests"][method] if "tests" in row else row["tags"]
        hits = [tag["corners_original_px"] for tag in tags if tag["id"] == 4 and tag["hamming"] <= hamming]
        if len(hits) == 1:
            corners.append(hits[0])
    if len(corners) < 8:
        raise ValueError(f"{path}: only {len(corners)} usable frames for ID 4")
    values = np.asarray(corners, dtype=float)
    if values.shape != (len(corners), 4, 2) or not np.isfinite(values).all():
        raise ValueError("Expected four finite corners per observation")
    return values, len(rows)


def geometry_for(lens):
    return Geometry(*(np.asarray(lens[key]) for key in (
        "camera_matrix", "distortion_coefficients", "R_base_camera", "normalized_mount_xy"
    )))


def calibrate(lens, corners, frames, native_odom):
    geometry = geometry_for(lens)
    median = np.median(corners, axis=0)
    fits = geometry.hypotheses(median)
    return {
        "marker": {"family": "tag36h11", "id": 4, "black_border_side_m": 0.05},
        "lens_geometry": lens,
        "frames": frames,
        "used_frames": len(corners),
        "median_original_corners_px": median.tolist(),
        "native_odom_while_parked": native_odom,
        "reference_branches": [
            {
                "R": fit["R"].tolist(),
                "t": fit["t"].tolist(),
                "corner_rms_original_px": fit["corner_rms_px"],
                "camera_to_center_m": float(np.linalg.norm(fit["t"]) * (.05 / .16)),
            }
            for fit in fits
        ],
    }


def estimate(reference, corners, frames, yaw_deg):
    lens = reference["lens_geometry"]
    geometry = geometry_for(lens)
    yaw = math.radians(yaw_deg)
    median = np.median(corners, axis=0)
    current = geometry.hypotheses(median)
    mount = np.r_[geometry.txy * .25, 0.]
    branches = []
    for index, ref in enumerate(reference["reference_branches"]):
        candidates = []
        for fit in current:
            rotation = geometry.Rbc @ fit["R"] @ np.asarray(ref["R"]).T @ geometry.Rbc.T
            candidate_yaw = -math.atan2(rotation[1, 0], rotation[0, 0])
            xy = (geometry.Rbc @ (np.asarray(ref["t"]) * .3125) + mount
                  - rz(candidate_yaw) @ (geometry.Rbc @ (fit["t"] * .3125) + mount))[:2]
            candidates.append({
                "xy_m": xy.tolist(),
                "distance_m": float(np.linalg.norm(xy)),
                "yaw_deg": math.degrees(candidate_yaw),
                "corner_rms_original_px": fit["corner_rms_px"],
            })
        branch = {"reference_branch": index, "image_only_candidates": candidates}
        model = Tag50PitchStart(lens, ref, yaw)
        model.reference_points *= .05 / .04
        try:
            pose, rms, pitch = model.fit_with_pitch(median)
            body_return = -(rz(-yaw)[:2, :2] @ pose[:2])
            branch.update(
                offset_xy_m=pose[:2].tolist(),
                distance_m=float(np.linalg.norm(pose[:2])),
                apparent_pitch_deg=math.degrees(pitch),
                corner_rms_original_px=rms,
                body_return_xy_m=body_return.tolist(),
                body_return_yaw_deg=-yaw_deg,
            )
        except ValueError as error:
            branch["fit_error"] = str(error)
        branches.append(branch)
    return {"frames": frames, "used_frames": len(corners), "heading_deg": yaw_deg, "branches": branches}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    origin = commands.add_parser("calibrate", help="Save the reference branches from parked observations")
    origin.add_argument("--geometry", type=Path, required=True)
    origin.add_argument("--native-odom", nargs=3, type=float, metavar=("X", "Y", "YAW_RAD"))
    fit = commands.add_parser("estimate", help="Report tag-only and heading-constrained poses")
    fit.add_argument("--reference", type=Path, required=True)
    fit.add_argument("--heading-deg", type=float, required=True)
    for command in (origin, fit):
        command.add_argument("--corners", type=Path, required=True)
        command.add_argument("--method", default="5x_0.6")
        command.add_argument("--max-hamming", type=int, choices=(0, 1, 2, 3), default=1)
        command.add_argument("--output", type=Path)
    args = parser.parse_args()
    corners, frames = stationary_corners(args.corners, args.method, args.max_hamming)
    if args.command == "calibrate":
        value = calibrate(read_json(args.geometry), corners, frames, args.native_odom)
    else:
        if not math.isfinite(args.heading_deg):
            parser.error("expected a finite relative heading")
        value = estimate(read_json(args.reference), corners, frames, args.heading_deg)
    save(value, args.output)


if __name__ == "__main__":
    main()
