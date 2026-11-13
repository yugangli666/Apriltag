"""Fit translation from the tag with a separately measured start heading."""

import numpy as np

from tag_corner_servo_v2 import Geometry, TAG_POINTS


class HeadingReferencedTagStart:
    def __init__(self, lens, reference_pose, yaw):
        self.geometry = Geometry(
            np.asarray(lens["camera_matrix"]),
            np.asarray(lens["distortion_coefficients"]),
            np.asarray(lens["R_base_camera"]),
            np.asarray(lens["normalized_mount_xy"]) * 0.25,
        )
        self.reference_points = (
            TAG_POINTS @ np.asarray(reference_pose["R"]).T
            + np.asarray(reference_pose["t"])
        ) * 0.25
        self.yaw = float(yaw)

    def fit(self, corners):
        observed = np.asarray(corners, dtype=float)
        if observed.shape != (4, 2) or not np.isfinite(observed).all():
            raise ValueError("Expected four finite ID 2 corners")

        def residual(state):
            x, y, height = state
            points = self.geometry.camera_after_body_move(self.reference_points, [x, y, self.yaw])
            points -= height * self.geometry.Rbc[2]
            return (self.geometry.project(points) - observed).ravel()

        state = np.zeros(3)
        steps = np.eye(3) * 1e-5
        for _ in range(8):
            jacobian = np.column_stack([
                (residual(state + step) - residual(state - step)) / 2e-5
                for step in steps
            ])
            change, _, rank, _ = np.linalg.lstsq(jacobian, -residual(state), rcond=None)
            if rank != 3 or not np.isfinite(change).all():
                raise ValueError("Tag corners do not constrain translation")
            state += change
            if np.linalg.norm(change) < 1e-8:
                break
        else:
            raise ValueError("The start fit did not converge")

        x, y, height = state
        if abs(height) > 0.04:
            raise ValueError("The fitted camera height offset exceeds the checked range")
        error = residual(state).reshape(4, 2)
        rms = float(np.sqrt(np.mean(np.sum(error**2, axis=1))))
        return np.array([x, y, self.yaw, height]), rms
