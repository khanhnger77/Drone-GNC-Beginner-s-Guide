"""
planar_ekf.py

Author: Khanh Ngo

Extended Kalman Filter for the planar quadrotor. It is a 2D miniature of PX4 EKF2
and uses the same three ideas:

1. The IMU is the INPUT of the prediction step, not a measurement.
   Gyro and accelerometer are integrated to predict attitude, velocity and position.
2. The gyro bias is part of the state, so the filter learns it from GPS and baro.
3. Measurements are fused one scalar at a time (sequential fusion) and each one
   passes an innovation consistency check first (PX4: "test ratio").

State x = [x_m, z_m, vx_mps, vz_mps, pitch_rad, gyro_bias_rps]
"""

import numpy as np
from math import sin, cos

from dronegnc.common.rotation import planar_dcm
from dronegnc.vehicle.planar_quadrotor import PlanarState

X, Z, VX, VZ, PITCH, GYRO_BIAS = range(6)
NUM_STATES = 6


class PlanarEkf:
    """
    Position, velocity, pitch and gyro bias estimation from IMU + GPS + barometer
    """

    def __init__(self, initial_state=PlanarState(), gravity_mps2=9.81,
                 gyro_noise_rps=0.01, accel_noise_mps2=0.3, gyro_bias_random_walk_rps2=1e-3,
                 innovation_gate_sigma=5.0):
        """
        Constructor
        initial_state: Initial guess of the state (PlanarState)
        gravity_mps2: Gravitational acceleration [m/s^2]
        gyro_noise_rps: Gyro noise used in the process noise [rad/s]
        accel_noise_mps2: Accelerometer noise used in the process noise [m/s^2]
        gyro_bias_random_walk_rps2: How fast the gyro bias may change [rad/s^2]
        innovation_gate_sigma: Measurements with |innovation| > gate * sigma are rejected
        """

        s = initial_state
        self.x = np.array([s.x_m, s.z_m, s.vx_mps, s.vz_mps, s.pitch_rad, 0.0])
        self.P = np.diag([1.0, 1.0, 0.5, 0.5, 0.1, 0.05]) ** 2
        self.gravity_mps2 = gravity_mps2
        self.IMU_NOISE_VAR = np.diag([gyro_noise_rps, accel_noise_mps2, accel_noise_mps2]) ** 2
        self.gyro_bias_random_walk_rps2 = gyro_bias_random_walk_rps2
        self.innovation_gate_sigma = innovation_gate_sigma
        self.pitch_rate_rps = 0.0
        self.num_rejected = 0

    def predict(self, imu, dt_s):
        """
        Prediction step: integrate the IMU sample
        imu: PlanarImuSample
        dt_s: Time since the last prediction [s]
        """

        x_m, z_m, vx, vz, pitch, bias = self.x
        fx, fz = imu.accel_body_mps2

        # Nonlinear motion model
        self.pitch_rate_rps = imu.gyro_rps - bias
        accel_ned = planar_dcm(pitch) @ imu.accel_body_mps2 + np.array([0.0, self.gravity_mps2])
        self.x = np.array([x_m + vx * dt_s,
                           z_m + vz * dt_s,
                           vx + accel_ned[0] * dt_s,
                           vz + accel_ned[1] * dt_s,
                           pitch + self.pitch_rate_rps * dt_s,
                           bias])

        # Jacobian of the motion model with respect to the state
        F = np.eye(NUM_STATES)
        F[X, VX] = dt_s
        F[Z, VZ] = dt_s
        F[VX, PITCH] = (-sin(pitch) * fx + cos(pitch) * fz) * dt_s
        F[VZ, PITCH] = (-cos(pitch) * fx - sin(pitch) * fz) * dt_s
        F[PITCH, GYRO_BIAS] = -dt_s

        # Jacobian with respect to the IMU input [gyro, accel_x, accel_z]
        G = np.zeros((NUM_STATES, 3))
        G[PITCH, 0] = dt_s
        G[VX:VZ + 1, 1:3] = planar_dcm(pitch) * dt_s

        Q = G @ self.IMU_NOISE_VAR @ G.T
        Q[GYRO_BIAS, GYRO_BIAS] += (self.gyro_bias_random_walk_rps2 * dt_s) ** 2
        self.P = F @ self.P @ F.T + Q

    def fuse_gps(self, gps):
        """
        Fuse a GPS sample: 4 scalar measurements (x, z, vx, vz)
        gps: PlanarGpsSample
        """

        measurements = [(X, gps.position_m[0], 0.5), (Z, gps.position_m[1], 0.5),
                        (VX, gps.velocity_mps[0], 0.1), (VZ, gps.velocity_mps[1], 0.1)]
        for index, value, noise_std in measurements:
            H = np.zeros(NUM_STATES)
            H[index] = 1.0
            self._fuse_scalar(H, value, self.x[index], noise_std ** 2)

    def fuse_baro(self, altitude_m, noise_std_m=0.1):
        """
        Fuse a barometer sample. The baro measures altitude = -z.
        altitude_m: Measured altitude [m]
        noise_std_m: Standard deviation of the baro noise [m]
        """

        H = np.zeros(NUM_STATES)
        H[Z] = -1.0
        self._fuse_scalar(H, altitude_m, -self.x[Z], noise_std_m ** 2)

    def _fuse_scalar(self, H, measured, predicted, noise_var):
        """
        Kalman update for one scalar measurement
        H: Row of the observation Jacobian (length NUM_STATES)
        measured: Measured value
        predicted: Value predicted from the current state
        noise_var: Measurement noise variance
        """

        innovation = measured - predicted
        innovation_var = H @ self.P @ H + noise_var  # scalar, so no matrix inverse needed

        # Innovation consistency check (PX4 calls innovation^2 / (gate^2 * var) the "test ratio")
        test_ratio = innovation ** 2 / (self.innovation_gate_sigma ** 2 * innovation_var)
        if test_ratio > 1.0:
            self.num_rejected += 1
            return

        K = self.P @ H / innovation_var
        self.x = self.x + K * innovation
        self.P = self.P - np.outer(K, H @ self.P)
        self.P = 0.5 * (self.P + self.P.T)  # keep it symmetric against rounding errors

    def estimate(self):
        """
        Current estimate as a PlanarState, so a controller can use it in place of the truth
        """

        x_m, z_m, vx, vz, pitch, _ = self.x
        return PlanarState(x_m, z_m, vx, vz, pitch, self.pitch_rate_rps)

    def std(self):
        """
        Standard deviation of each state (square root of the covariance diagonal)
        """

        return np.sqrt(np.diag(self.P))
