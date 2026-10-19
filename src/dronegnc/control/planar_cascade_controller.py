"""
planar_cascade_controller.py

Author: Khanh Ngo

Cascaded controller for the planar quadrotor, with the same loop structure as PX4:

  position --P--> velocity --PID--> acceleration --> thrust + pitch
           --P--> pitch rate --PID--> pitch moment --> rotor thrusts

Each inner loop must be faster than the loop around it. Here the bandwidths are
roughly 1 (position) < 3 (velocity) < 8 (attitude) < 20 (rate) rad/s.
"""

import numpy as np
from dataclasses import dataclass
from math import atan2, cos, radians

from dronegnc.common.rotation import wrap_pi
from dronegnc.control.pid import Pid


@dataclass
class PlanarSetpoints:
    """
    Intermediate outputs of every loop, for plotting and debugging
    """

    velocity_mps: np.ndarray
    accel_mps2: np.ndarray
    pitch_rad: float
    pitch_rate_rps: float
    collective_thrust_n: float
    pitch_moment_nm: float


class PlanarCascadeController:
    """
    Position controller of the planar quadrotor
    """

    def __init__(self, params, position_gain=1.0, velocity_gains=(3.0, 1.0, 0.0),
                 attitude_gain=8.0, rate_gains=(20.0, 2.0, 0.0),
                 max_speed_mps=4.0, max_tilt_rad=radians(35.0), max_pitch_rate_rps=radians(220.0)):
        """
        Constructor
        params: PlanarQuadrotorParams of the controlled vehicle
        position_gain: P gain from position error [m] to velocity setpoint [m/s] [1/s]
        velocity_gains: (kp, ki, kd) from velocity error [m/s] to acceleration [m/s^2]
        attitude_gain: P gain from pitch error [rad] to pitch rate setpoint [rad/s] [1/s]
        rate_gains: (kp, ki, kd) from pitch rate error [rad/s] to angular acceleration [rad/s^2]
        max_speed_mps: Limit of the velocity setpoint [m/s]
        max_tilt_rad: Limit of the pitch setpoint [rad]
        max_pitch_rate_rps: Limit of the pitch rate setpoint [rad/s]
        """

        self.params = params
        self.position_gain = position_gain
        self.velocity_pid = Pid(*velocity_gains, integral_limit=3.0)
        self.attitude_gain = attitude_gain
        self.rate_pid = Pid(*rate_gains, integral_limit=10.0)
        self.max_speed_mps = max_speed_mps
        self.max_tilt_rad = max_tilt_rad
        self.max_pitch_rate_rps = max_pitch_rate_rps
        self.setpoints = None

    def update(self, position_sp_m, state, dt_s):
        """
        Rotor thrusts [front, rear] [N] that move the vehicle to the position setpoint
        position_sp_m: Desired [x, z] in NED [m]
        state: PlanarState used for feedback (truth or estimate)
        dt_s: Time step [s]
        """

        p = self.params
        position_m = np.array([state.x_m, state.z_m])
        velocity_mps = np.array([state.vx_mps, state.vz_mps])

        # 1. Position loop (P)
        velocity_sp = self.position_gain * (np.asarray(position_sp_m) - position_m)
        speed = np.linalg.norm(velocity_sp)
        if speed > self.max_speed_mps:
            velocity_sp *= self.max_speed_mps / speed

        # 2. Velocity loop (PID) -> acceleration setpoint
        accel_sp = self.velocity_pid.update(velocity_sp, velocity_mps, dt_s)

        # 3. Acceleration -> thrust vector in NED. The thrust must produce the
        #    acceleration AND cancel gravity: F = m * (a - g)
        thrust_x, thrust_z = p.mass_kg * (accel_sp - np.array([0.0, p.gravity_mps2]))
        thrust_z = min(thrust_z, -0.1 * p.mass_kg * p.gravity_mps2)  # always push up a little

        # 4. Thrust vector -> pitch and collective thrust. Thrust is -T*[sin(pitch), cos(pitch)]
        pitch_sp = np.clip(atan2(-thrust_x, -thrust_z), -self.max_tilt_rad, self.max_tilt_rad)
        collective_thrust_n = -thrust_z / cos(pitch_sp)  # keep the vertical part (altitude first)
        collective_thrust_n = min(collective_thrust_n, 2.0 * p.max_rotor_thrust_n)

        # 5. Attitude loop (P) -> pitch rate setpoint
        pitch_rate_sp = np.clip(self.attitude_gain * wrap_pi(pitch_sp - state.pitch_rad),
                                -self.max_pitch_rate_rps, self.max_pitch_rate_rps)

        # 6. Rate loop (PID) -> pitch moment
        pitch_accel_sp = self.rate_pid.update(pitch_rate_sp, state.pitch_rate_rps, dt_s)
        pitch_moment_nm = p.inertia_kgm2 * pitch_accel_sp

        # 7. Mixer / control allocation: T = Tf + Tr, M = l * (Tf - Tr)
        thrust_front_n = collective_thrust_n / 2.0 + pitch_moment_nm / (2.0 * p.arm_length_m)
        thrust_rear_n = collective_thrust_n / 2.0 - pitch_moment_nm / (2.0 * p.arm_length_m)

        self.setpoints = PlanarSetpoints(velocity_sp, accel_sp, pitch_sp, pitch_rate_sp,
                                         collective_thrust_n, pitch_moment_nm)
        return np.clip([thrust_front_n, thrust_rear_n], 0.0, p.max_rotor_thrust_n)
