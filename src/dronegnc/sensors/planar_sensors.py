"""
planar_sensors.py

Author: Khanh Ngo

Sensors for the planar quadrotor. Each sensor reads the TRUE vehicle and returns a
noisy measurement. Sensors other than the IMU return None when no new sample is due.
"""

import numpy as np
from dataclasses import dataclass

from dronegnc.sensors.sample_timer import SampleTimer


@dataclass
class PlanarImuSample:
    """
    gyro_rps: Measured pitch rate [rad/s]
    accel_body_mps2: Measured specific force [x_body, z_body] [m/s^2]
    """

    gyro_rps: float
    accel_body_mps2: np.ndarray


@dataclass
class PlanarGpsSample:
    """
    position_m: Measured [x, z] position in NED [m]
    velocity_mps: Measured [vx, vz] velocity in NED [m/s]
    """

    position_m: np.ndarray
    velocity_mps: np.ndarray


class PlanarImu:
    """
    Gyroscope + accelerometer. Runs every simulation step.
    """

    def __init__(self, gyro_noise_rps=0.01, accel_noise_mps2=0.3, gyro_bias_rps=0.02, rng=None):
        """
        Constructor
        gyro_noise_rps: Standard deviation of the gyro white noise [rad/s]
        accel_noise_mps2: Standard deviation of the accelerometer white noise [m/s^2]
        gyro_bias_rps: Constant gyro bias, unknown to the estimator [rad/s]
        rng: numpy random Generator (for reproducible runs)
        """

        self.gyro_noise_rps = gyro_noise_rps
        self.accel_noise_mps2 = accel_noise_mps2
        self.gyro_bias_rps = gyro_bias_rps
        self.rng = rng if rng is not None else np.random.default_rng()

    def measure(self, vehicle):
        gyro_rps = (vehicle.state.pitch_rate_rps + self.gyro_bias_rps
                    + self.rng.normal(0.0, self.gyro_noise_rps))
        accel_body_mps2 = (vehicle.specific_force_body_mps2()
                           + self.rng.normal(0.0, self.accel_noise_mps2, 2))
        return PlanarImuSample(gyro_rps, accel_body_mps2)


class PlanarGps:
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
        position_m = np.array([s.x_m, s.z_m]) + self.rng.normal(0.0, self.position_noise_m, 2)
        velocity_mps = np.array([s.vx_mps, s.vz_mps]) + self.rng.normal(0.0, self.velocity_noise_mps, 2)
        return PlanarGpsSample(position_m, velocity_mps)


class PlanarBarometer:
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
        return vehicle.state.altitude_m() + self.rng.normal(0.0, self.altitude_noise_m)
