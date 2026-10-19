"""
attitude_complementary_filter.py

Author: Khanh Ngo

Nonlinear complementary filter for 3D attitude (Mahony filter). It follows the
structure of PX4 src/modules/attitude_estimator_q:

- The gyro is integrated to propagate the quaternion (fast, but drifts).
- The accelerometer tells where "down" is, which corrects roll and pitch.
- The magnetometer tells where North is, which corrects only the yaw.
- An integral of the correction estimates the gyro bias.

Assumption: the accelerometer measures mostly gravity. This is wrong during
strong accelerations; PX4 subtracts a GPS-based acceleration for that, and EKF2
solves it properly by estimating velocity at the same time.
"""

import numpy as np
from math import atan2

from dronegnc.common.rotation import (quat_multiply, quat_normalize, quat_to_dcm,
                                      quat_from_rotation_vector, dcm_to_quat, wrap_pi)


class AttitudeComplementaryFilter:
    """
    Attitude and gyro bias estimation from IMU + magnetometer
    """

    def __init__(self, accel_weight=0.2, mag_weight=0.1, gyro_bias_weight=0.1,
                 max_gyro_bias_rps=0.05, max_spin_rate_for_bias_rps=0.175, mag_declination_rad=0.0):
        """
        Constructor (the names match the PX4 parameters ATT_W_ACC, ATT_W_MAG, ATT_W_GYRO_BIAS)
        accel_weight: Gain of the roll/pitch correction from the accelerometer [1/s]
        mag_weight: Gain of the yaw correction from the magnetometer [1/s]
        gyro_bias_weight: Gain of the gyro bias integrator [1/s]
        max_gyro_bias_rps: Limit of the estimated gyro bias [rad/s]
        max_spin_rate_for_bias_rps: The bias is learned only below this angular speed [rad/s]
        mag_declination_rad: Angle between magnetic North and true North [rad]
        """

        self.accel_weight = accel_weight
        self.mag_weight = mag_weight
        self.gyro_bias_weight = gyro_bias_weight
        self.max_gyro_bias_rps = max_gyro_bias_rps
        self.max_spin_rate_for_bias_rps = max_spin_rate_for_bias_rps
        self.mag_declination_rad = mag_declination_rad
        self.quaternion = None  # unknown until the first update
        self.gyro_bias_rps = np.zeros(3)
        self.angular_velocity_rps = np.zeros(3)
        self.last_mag_body = None

    def update(self, imu, mag_body, dt_s):
        """
        Run one filter step
        imu: ImuSample
        mag_body: Magnetometer sample in the body frame [gauss], or None if no new sample
        dt_s: Time since the last update [s]
        """

        # The magnetometer is slower than the IMU: keep using its latest sample (like PX4)
        if mag_body is not None:
            self.last_mag_body = mag_body
        if self.last_mag_body is None:
            return
        if self.quaternion is None:
            self.quaternion = self._initial_attitude(imu.accel_body_mps2, self.last_mag_body)

        R = quat_to_dcm(self.quaternion)
        correction_rps = np.zeros(3)

        # Yaw correction from the magnetometer (only the horizontal part is used)
        mag_ned = R @ self.last_mag_body
        yaw_error_rad = wrap_pi(atan2(mag_ned[1], mag_ned[0]) - self.mag_declination_rad)
        correction_rps += R.T @ np.array([0.0, 0.0, -yaw_error_rad]) * self.mag_weight

        # Roll/pitch correction: the predicted "down" should line up with the measured one
        down_predicted_body = R.T @ np.array([0.0, 0.0, 1.0])
        down_measured_body = -imu.accel_body_mps2 / np.linalg.norm(imu.accel_body_mps2)
        correction_rps += np.cross(down_measured_body, down_predicted_body) * self.accel_weight

        # Gyro bias: integral of the correction. Like PX4, learn it only while rotating
        # slowly, because fast rotations make the corrections less trustworthy.
        if np.linalg.norm(imu.gyro_rps) < self.max_spin_rate_for_bias_rps:
            self.gyro_bias_rps -= correction_rps * self.gyro_bias_weight * dt_s
        self.gyro_bias_rps = np.clip(self.gyro_bias_rps, -self.max_gyro_bias_rps, self.max_gyro_bias_rps)

        # Propagate with the corrected angular velocity
        self.angular_velocity_rps = imu.gyro_rps - self.gyro_bias_rps
        delta_q = quat_from_rotation_vector((self.angular_velocity_rps + correction_rps) * dt_s)
        self.quaternion = quat_normalize(quat_multiply(self.quaternion, delta_q))

    @staticmethod
    def _initial_attitude(accel_body, mag_body):
        """
        Attitude from one accelerometer and one magnetometer sample, assuming no acceleration
        """

        down_body = -accel_body / np.linalg.norm(accel_body)
        east_body = np.cross(down_body, mag_body)
        east_body /= np.linalg.norm(east_body)
        north_body = np.cross(east_body, down_body)
        R_ned_to_body = np.column_stack([north_body, east_body, down_body])
        return dcm_to_quat(R_ned_to_body.T)
