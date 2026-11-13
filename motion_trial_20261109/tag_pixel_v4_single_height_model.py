"""Compute the constant-twist estimate used in preflight."""

import math
import numpy as np
from tag_corner_servo_v2 import rz


def continuous_command(pose, gain):
    """Constant body twist and its maximum planned duration."""
    heading = -float(pose[2])
    xy = -(rz(heading)[:2, :2] @ pose[:2])
    if abs(heading) > 1e-9:
        a = math.sin(heading) / heading
        b = (1 - math.cos(heading)) / heading
        integral_xy = np.linalg.solve([[a, -b], [b, a]], xy)
    else:
        integral_xy = xy
    integrated = np.r_[integral_xy, heading] / gain
    active = max(np.linalg.norm(integrated[:2]) / 0.01, abs(integrated[2]) / 0.01)
    if active < 1e-9:
        return np.zeros(3), 0.0, xy
    return integrated / active, active + 0.16, xy
