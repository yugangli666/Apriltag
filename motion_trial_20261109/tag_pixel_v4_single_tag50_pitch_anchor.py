"""Fit the 50 mm marker with referenced yaw and an apparent camera pitch offset."""

import math

import numpy as np

from tag_pixel_v4_single_tag40_heading_anchor import HeadingReferencedTagStart


class Tag50PitchStart(HeadingReferencedTagStart):
    def fit_with_pitch(self, corners):
        observed = np.asarray(corners, dtype=float)
        if observed.shape != (4, 2) or not np.isfinite(observed).all():
            raise ValueError("Expected four finite ID 4 corners")

        def residual(state):
            x, y, pitch = state
            points = self.geometry.camera_after_body_move(self.reference_points, [x, y, self.yaw])
            c, s = math.cos(pitch), math.sin(pitch)
            rotation = np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])
            return (self.geometry.project(points @ rotation.T) - observed).ravel()

        state = np.zeros(3)
        steps = np.eye(3) * 1e-5
        for _ in range(10):
            jacobian = np.column_stack([
                (residual(state + step) - residual(state - step)) / 2e-5
                for step in steps
            ])
            change, _, rank, _ = np.linalg.lstsq(jacobian, -residual(state), rcond=None)
            if rank != 3 or not np.isfinite(change).all():
                raise ValueError("The tag does not constrain the parked pose")
            state += change
            if np.linalg.norm(change) < 1e-8:
                break
        else:
            raise ValueError("The parked pose did not converge")
        x, y, pitch = state
        if abs(pitch) > math.radians(3):
            raise ValueError("The fitted camera pitch offset exceeds the trial range")
        error = residual(state).reshape(4, 2)
        rms = float(np.sqrt(np.mean(np.sum(error**2, axis=1))))
        return np.array([x, y, self.yaw, 0.]), rms, pitch

    def fit(self, corners):
        pose, rms, _ = self.fit_with_pitch(corners)
        return pose, rms
