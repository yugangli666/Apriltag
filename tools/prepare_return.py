"""Prepare the visual anchor for a reviewed, stationary hardware run."""

import argparse
import math
from pathlib import Path

import numpy as np

from estimate import Tag50PitchStart, estimate, read_json, rz, save, stationary_corners


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--corners", type=Path, required=True)
    parser.add_argument("--start-odom", nargs=3, type=float, required=True, metavar=("X", "Y", "YAW_RAD"))
    parser.add_argument("--comparison-xy", nargs=2, type=float, required=True, metavar=("X_M", "Y_M"),
                        help="Independent start offset in origin axes for comparison, in meters")
    parser.add_argument("--original-reference-id", required=True, help="ID of the local chassis direction reference")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    reference = read_json(args.reference)
    origin = np.asarray(reference["native_odom_while_parked"], dtype=float)
    start = np.asarray(args.start_odom, dtype=float)
    comparison = np.asarray(args.comparison_xy, dtype=float)
    if origin.shape != (3,) or not np.isfinite(np.r_[origin, start, comparison]).all():
        parser.error("origin, start and comparison must contain finite values")
    yaw = math.remainder(start[2] - origin[2], 2 * math.pi)
    if abs(yaw) > math.radians(30):
        parser.error("start heading exceeds the controller's 30 degree limit")
    corners, frames = stationary_corners(args.corners, "5x_0.6", 0)
    if len(corners) < 90:
        parser.error("need at least 90 zero-correction frames for the recorded start")
    candidates = estimate(reference, corners, frames, math.degrees(yaw))["branches"]
    candidates = [row for row in candidates if "offset_xy_m" in row
                  and row["corner_rms_original_px"] <= .35 and row["distance_m"] <= .75
                  and np.linalg.norm(np.asarray(row["offset_xy_m"]) - comparison) <= .05]
    if not candidates:
        parser.error("no image estimate agrees with the independent comparison within 5 cm")
    if len(candidates) == 2 and np.linalg.norm(
        np.subtract(candidates[0]["offset_xy_m"], candidates[1]["offset_xy_m"])
    ) > .05:
        parser.error("candidate returns differ by more than 5 cm")
    selected = min(candidates, key=lambda row: row["corner_rms_original_px"])
    lens = reference["lens_geometry"]
    ref = reference["reference_branches"][selected["reference_branch"]]
    model = Tag50PitchStart(lens, ref, yaw)
    model.reference_points *= .05 / .04
    median = np.median(corners, axis=0)
    pose, rms, pitch = model.fit_with_pitch(median)
    windows = [model.fit(np.median(part, axis=0)) for part in np.array_split(corners, 3)]
    if any(error > .35 or np.linalg.norm(p[:2] - pose[:2]) > .02 for p, error in windows):
        parser.error("the stationary image fit does not repeat within the checked limits")
    save({
        "marker": reference["marker"], "original_reference_id": args.original_reference_id,
        "scale": 5, "geometry": lens, "reference_pose": {"R": ref["R"], "t": ref["t"]},
        "position_source": "5x ID 4 corner fit with referenced odometry heading",
        "odometry_position_comparison_source": "operator-provided independent parked-start comparison",
        "odometry_position_comparison_m": comparison.tolist(),
        "start_native_odom": start.tolist(), "start_yaw_rad": yaw,
        "primary_pose_xy_yaw_height": pose.tolist(), "start_corners_raw_px": median.tolist(),
        "apparent_pitch_offset_deg": math.degrees(pitch), "start_image_rms_px": rms,
        "return_body_xy_m": (-(rz(-yaw)[:2, :2] @ pose[:2])).tolist(),
    }, args.output)


if __name__ == "__main__":
    main()
