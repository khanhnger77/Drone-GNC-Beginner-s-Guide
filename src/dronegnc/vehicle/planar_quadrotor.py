"""
planar_quadrotor.py

Author: Khanh Ngo

2D quadrotor that moves in the North-Down (x-z) plane of the NED frame.
It has a front rotor and a rear rotor, and rotates only in pitch.
This is the smallest model that still has the key property of a multicopter:
to move horizontally, it must first tilt its thrust.

    front rotor (+x body)        rear rotor (-x body)
          Tf |                        | Tr
             o------------------------o
                      body (pitch)
"""

import numpy as np
from copy import copy
from dataclasses import dataclass

from dronegnc.common.rotation import planar_dcm


@dataclass
class PlanarQuadrotorParams:
    """
    Physical parameters of the planar quadrotor
    mass_kg: Mass [kg]
    inertia_kgm2: Moment of inertia around the pitch axis [kg m^2]
    arm_length_m: Distance from the center to each rotor [m]
    max_rotor_thrust_n: Maximum thrust of one rotor [N]
    gravity_mps2: Gravitational acceleration [m/s^2]
    """

    mass_kg: float = 1.0
    inertia_kgm2: float = 0.01
    arm_length_m: float = 0.2
    max_rotor_thrust_n: float = 10.0
    gravity_mps2: float = 9.81


@dataclass
class PlanarState:
    """
    State of the planar quadrotor in the NED frame (z is down, altitude = -z)
    x_m: North position [m]
    z_m: Down position [m]
    vx_mps: North velocity [m/s]
    vz_mps: Down velocity [m/s]
    pitch_rad: Pitch angle, positive = nose up [rad]
    pitch_rate_rps: Pitch rate [rad/s]
    """

    x_m: float = 0.0
    z_m: float = 0.0
    vx_mps: float = 0.0
    vz_mps: float = 0.0
    pitch_rad: float = 0.0
    pitch_rate_rps: float = 0.0

    def altitude_m(self):
        return -self.z_m


class PlanarQuadrotor:
    """
    True dynamics of the planar quadrotor. Only the simulator touches this object;
    the navigation stack sees it only through sensors.
    """

    def __init__(self, params=PlanarQuadrotorParams(), initial_state=PlanarState()):
        """
        Constructor
        params: PlanarQuadrotorParams
        initial_state: PlanarState at time 0
        """

        self.params = params
        self.state = copy(initial_state)
        self.accel_ned_mps2 = np.array([0.0, 0.0])  # last acceleration, needed by the IMU

    def step(self, rotor_thrusts_n, dt_s):
        """
        Advance the true state by one time step (semi-implicit Euler)
        rotor_thrusts_n: [front, rear] rotor thrusts [N]
        dt_s: Time step [s]
        """

        p = self.params
        s = self.state
        thrust_front_n, thrust_rear_n = np.clip(rotor_thrusts_n, 0.0, p.max_rotor_thrust_n)

        # Forces and moment
        collective_thrust_n = thrust_front_n + thrust_rear_n
        pitch_moment_nm = p.arm_length_m * (thrust_front_n - thrust_rear_n)

        # Thrust points to -z in the body frame; rotate it into NED and add gravity
        thrust_ned_n = planar_dcm(s.pitch_rad) @ np.array([0.0, -collective_thrust_n])
        accel_ned_mps2 = thrust_ned_n / p.mass_kg + np.array([0.0, p.gravity_mps2])
        pitch_accel_rps2 = pitch_moment_nm / p.inertia_kgm2

        # Integrate: velocity first, then position with the new velocity
        s.vx_mps += accel_ned_mps2[0] * dt_s
        s.vz_mps += accel_ned_mps2[1] * dt_s
        s.x_m += s.vx_mps * dt_s
        s.z_m += s.vz_mps * dt_s
        s.pitch_rate_rps += pitch_accel_rps2 * dt_s
        s.pitch_rad += s.pitch_rate_rps * dt_s

        # Simple ground contact at z = 0: the ground pushes back
        if s.z_m > 0.0:
            s.z_m = 0.0
            s.vx_mps, s.vz_mps = 0.0, min(s.vz_mps, 0.0)
            accel_ned_mps2 = np.array([0.0, min(accel_ned_mps2[1], 0.0)])

        self.accel_ned_mps2 = accel_ned_mps2

    def specific_force_body_mps2(self):
        """
        What an ideal accelerometer measures: acceleration minus gravity, in the body frame.
        At hover this is [0, -g] (the thrust holding the vehicle up), not zero.
        """

        gravity_ned = np.array([0.0, self.params.gravity_mps2])
        return planar_dcm(self.state.pitch_rad).T @ (self.accel_ned_mps2 - gravity_ned)

    def hover_thrust_n(self):
        """
        Collective thrust that balances gravity [N]
        """

        return self.params.mass_kg * self.params.gravity_mps2
