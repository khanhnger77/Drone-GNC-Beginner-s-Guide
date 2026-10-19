"""
quadrotor_sensors.py

Author: Khanh Ngo

Sensors for the 3D quadrotor. Each sensor reads the TRUE vehicle and returns a
noisy measurement. Sensors other than the IMU return None when no new sample is due.
"""

import numpy as np
from dataclasses import dataclass

from dronegnc.common.rotation import quat_to_dcm
from dronegnc.sensors.sample_timer import SampleTimer


@dataclass
class ImuSample:
    """
    gyro_rps: Measured angular velocity in the body frame [rad/s]
    accel_body_mps2: Measured specific force in the body frame [m/s^2]
    """

    gyro_rps: np.ndarray
    accel_body_mps2: np.ndarray


@dataclass
class GpsSample:
    """
    position_ned_m: Measured position in NED [m]
    velocity_ned_mps: Measured velocity in NED [m/s]
    """

    position_ned_m: np.ndarray
    velocity_ned_mps: np.ndarray


class Imu:
    """
    Gyroscope + accelerometer. Runs every simulation step.
    """

    def __init__(self, gyro_noise_rps=0.01, accel_noise_mps2=0.3,
                 gyro_bias_rps=np.array([0.02, -0.01, 0.015]), rng=None):
        """
        Constructor
        gyro_noise_rps: Standard deviation of the gyro white noise [rad/s]
        accel_noise_mps2: Standard deviation of the accelerometer white noise [m/s^2]
        gyro_bias_rps: Constant gyro bias, unknown to the estimator [rad/s]
        rng: numpy random Generator (for reproducible runs)
        """

        self.gyro_noise_rps = gyro_noise_rps
        self.accel_noise_mps2 = accel_noise_mps2
        self.gyro_bias_rps = np.asarray(gyro_bias_rps, dtype=float)
        self.rng = rng if rng is not None else np.random.default_rng()

    def measure(self, vehicle):
        gyro_rps = (vehicle.state.angular_velocity_body_rps + self.gyro_bias_rps
                    + self.rng.normal(0.0, self.gyro_noise_rps, 3))
        accel_body_mps2 = (vehicle.specific_force_body_mps2()
                           + self.rng.normal(0.0, self.accel_noise_mps2, 3))
        return ImuSample(gyro_rps, accel_body_mps2)


class Gps:
    """
    GPS receiver: position and velocity in NED
    """

    def __init__(self, rate_hz=10.0, position_noise_m=0.5, velocity_noise_mps=0.1, rng=None):
        """
        Constructor
        rate_hz: Output rate [Hz]
        position_noise_m: Standard deviation of the position noise [m]
        velocity_noise_mps: Standard deviation of the velocity noise [m/s]
        rng: numpy random Generator (for reproducible runs)
        """

        self.timer = SampleTimer(rate_hz)
        self.position_noise_m = position_noise_m
        self.velocity_noise_mps = velocity_noise_mps
        self.rng = rng if rng is not None else np.random.default_rng()

    def measure(self, vehicle, time_s):
        if not self.timer.is_due(time_s):
            return None
        s = vehicle.state
        return GpsSample(s.position_ned_m + self.rng.normal(0.0, self.position_noise_m, 3),
                         s.velocity_ned_mps + self.rng.normal(0.0, self.velocity_noise_mps, 3))


class Barometer:
    """
    Barometric altimeter: altitude above the start point (= -z)
    """

    def __init__(self, rate_hz=50.0, altitude_noise_m=0.1, rng=None):
        """
        Constructor
        rate_hz: Output rate [Hz]
        altitude_noise_m: Standard deviation of the altitude noise [m]
        rng: numpy random Generator (for reproducible runs)
        """

        self.timer = SampleTimer(rate_hz)
        self.altitude_noise_m = altitude_noise_m
        self.rng = rng if rng is not None else np.random.default_rng()

    def measure(self, vehicle, time_s):
        if not self.timer.is_due(time_s):
            return None
        return -vehicle.state.position_ned_m[2] + self.rng.normal(0.0, self.altitude_noise_m)


class Magnetometer:
    """
    Magnetometer: Earth magnetic field seen in the body frame
    """

    def __init__(self, rate_hz=50.0, earth_field_ned_gauss=np.array([0.21, 0.0, 0.43]),
                 noise_gauss=0.01, rng=None):
        """
        Constructor
        rate_hz: Output rate [Hz]
        earth_field_ned_gauss: Earth magnetic field in NED (zero declination here) [gauss]
        noise_gauss: Standard deviation of the noise [gauss]
        rng: numpy random Generator (for reproducible runs)
        """

        self.timer = SampleTimer(rate_hz)
        self.earth_field_ned_gauss = np.asarray(earth_field_ned_gauss, dtype=float)
        self.noise_gauss = noise_gauss
        self.rng = rng if rng is not None else np.random.default_rng()

    def measure(self, vehicle, time_s):
        if not self.timer.is_due(time_s):
            return None
        R = quat_to_dcm(vehicle.state.quaternion)
        return R.T @ self.earth_field_ned_gauss + self.rng.normal(0.0, self.noise_gauss, 3)
