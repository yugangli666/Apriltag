"""Native chassis odometry displacement and IMU yaw in the start-tag frame."""
import math
from collections import deque
import numpy as np


def rotation2(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def calibration(samples):
    if len(samples) < 100 or samples[-1][0] - samples[0][0] < 1_000_000_000:
        raise ValueError("Insufficient stationary IMU samples")
    gyro = np.asarray([row[1] for row in samples], float)
    accel = np.asarray([row[2] for row in samples], float)
    up = accel.mean(axis=0)
    magnitude = np.linalg.norm(up)
    if not 8.0 < magnitude < 11.5 or not np.isfinite(gyro).all():
        raise ValueError("Invalid stationary IMU gravity or gyro data")
    up /= magnitude
    projected = gyro @ up
    if np.std(projected) > 0.005:
        raise ValueError("IMU did not settle for gyro-bias estimation")
    return up, float(np.mean(projected))


def native_pose(xy, encoded_yaw):
    xy = np.asarray(xy, float)
    yaw = np.asarray(encoded_yaw, float)
    if not np.isfinite(xy).all() or not np.isfinite(yaw).all():
        raise ValueError("Nonfinite native odometry pose")
    # controller_odom_bridge.py decodes the chassis-specific yaw in w.
    if np.max(np.abs(yaw[:3])) > 1e-6 or abs(yaw[3]) > math.pi + 1e-6:
        raise ValueError("Unexpected native odometry orientation encoding")
    return xy, float(yaw[3])


class PoseImuTrack:
    def __init__(self, pose, up, bias, imu_stamp, gyro, wheel_stamp, native):
        self.pose = np.asarray(pose[:3], float).copy()
        self.up = np.asarray(up, float)
        self.bias = bias
        self.imu_stamp = imu_stamp
        self.imu_rate = float(np.dot(gyro, self.up) - bias)
        self.wheel_stamp = wheel_stamp
        self.wheel_yaw = float(self.pose[2])
        self.wheel_xy, self.native_yaw = native
        self.distance = 0.
        self.travel_history = deque([(wheel_stamp, 0.)])

    @staticmethod
    def period(new, old):
        dt = (new - old) / 1e9
        if not 0 < dt <= .10:
            raise ValueError("Feedback timestamp gap or reset")
        return dt

    def imu(self, stamp, gyro):
        rate = float(np.dot(gyro, self.up) - self.bias)
        if not math.isfinite(rate) or abs(rate) > .1:
            raise ValueError("IMU yaw rate outside trial envelope")
        dt = self.period(stamp, self.imu_stamp)
        self.pose[2] += (self.imu_rate + rate) * .5 * dt
        self.imu_stamp, self.imu_rate = stamp, rate

    def wheel(self, stamp, native):
        dt = self.period(stamp, self.wheel_stamp)
        if abs(self.imu_stamp - stamp) > 60_000_000:
            raise ValueError("Wheel and IMU feedback clocks differ")
        xy, yaw = native
        step = xy - self.wheel_xy
        travel = float(np.linalg.norm(step))
        native_turn = math.remainder(yaw - self.native_yaw, 2 * math.pi)
        if travel > .003 + .06 * dt or abs(native_turn) > .03 + .20 * dt:
            raise ValueError(f"Native odometry pose discontinuity: {travel:.5f} m, {native_turn:.5f} rad")
        while len(self.travel_history) > 1 and stamp - self.travel_history[1][0] >= 400_000_000:
            self.travel_history.popleft()
        stamp0, travel0 = self.travel_history[0]
        span = (stamp - stamp0) / 1e9
        if span >= .4 and (self.distance + travel - travel0) / span > .06:
            raise ValueError("Sustained native odometry displacement above 0.06 m/s")
        world_yaw = (self.wheel_yaw + self.pose[2]) * .5
        native_yaw = self.native_yaw + native_turn * .5
        self.pose[:2] += rotation2(world_yaw - native_yaw) @ step
        self.distance += travel
        self.travel_history.append((stamp, self.distance))
        self.wheel_stamp, self.wheel_xy, self.native_yaw = stamp, xy, yaw
        self.wheel_yaw = float(self.pose[2])


def startup_command(command, elapsed):
    # Preserve the planned translation/yaw ratio while the steering starts moving.
    fraction = min(1., max(0., elapsed / 2.0))
    return np.asarray(command, float) * (fraction * fraction * (3. - 2. * fraction))


def control_to_origin(pose):
    body = -(rotation2(-pose[2]) @ pose[:2])
    distance = float(np.linalg.norm(body))
    heading = math.remainder(-pose[2], 2 * math.pi)
    if distance <= .015:
        linear = np.zeros(2)
    else:
        speed = min(.01, max(.004, distance * .35))
        linear = body * (speed / distance)
    yaw = 0. if abs(heading) <= .0175 else math.copysign(min(.008, max(.004, abs(heading)*.5)), heading)
    return np.r_[linear, yaw], distance, heading
