"""Recompute the fourth 50 mm trial from its published corner samples."""

import json
from pathlib import Path

from estimate import calibrate, estimate, read_json, stationary_corners


def main():
    examples = Path(__file__).resolve().parents[1] / "examples"
    trial = read_json(examples / "round4.json")
    geometry = read_json(examples / "camera_geometry.json")
    origin, count = stationary_corners(examples / "origin_corners.jsonl", "5x_0.6", 1)
    reference = calibrate(geometry, origin, count, None)
    branch = trial["selected_origin_branch"]
    result = {"camera_to_tag_center_m": reference["reference_branches"][branch]["camera_to_center_m"]}
    for label, max_hamming, yaw in (
        ("start", 0, trial["start_heading_deg"]),
        ("parked", 1, trial["parked_imu_heading_deg"]),
    ):
        samples, count = stationary_corners(examples / f"{label}_corners.jsonl", "5x_0.6", max_hamming)
        fitted = estimate(reference, samples, count, yaw)
        result[label] = {**fitted["branches"][branch], "used_frames": len(samples), "frames": count}
    result["field_record"] = {
        key: trial[key] for key in ("motion_seconds", "odometry_imu_residual_cm", "tape_measurement")
    }
    print(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False))


if __name__ == "__main__":
    main()
