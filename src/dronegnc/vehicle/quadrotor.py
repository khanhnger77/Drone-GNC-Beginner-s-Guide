"""
quadrotor.py

Author: Khanh Ngo

3D quadrotor rigid body (6 degrees of freedom) in the NED frame, with an FRD body frame.
The rotor layout follows the PX4 "Quadrotor X" airframe:

          front
     3(CW)     1(CCW)
          \\   /
           \\ /
           / \\
          /   \\
     2(CCW)    4(CW)
          rear

PX4 numbers motors from 1; the arrays here use index 0..3 in the same order.
"""

import numpy as np
from copy import deepcopy
from dataclasses import dataclass, field
from math import sqrt

from dronegnc.common.rotation import (quat_multiply, quat_normalize,
                                      quat_from_rotation_vector, quat_to_dcm)


@dataclass
class Rotor:
    """
    One rotor
    position_body_m: Position of the rotor in the FRD body frame [m]
    spin_direction: +1 = counter-clockwise seen from above, -1 = clockwise
    """

    position_body_m: np.ndarray
    spin_direction: int


def quad_x_rotors(arm_length_m):
    """
    Rotor layout of a Quadrotor X with the PX4 motor order
    arm_length_m: Distance from the center to each rotor [m]
    """

    d = arm_length_m / sqrt(2.0)
    return [Rotor(np.array([d, d, 0.0]), +1),    # 1: front right, CCW
            Rotor(np.array([-d, -d, 0.0]), +1),  # 2: rear left, CCW
            Rotor(np.array([d, -d, 0.0]), -1),   # 3: front left, CW
            Rotor(np.array([-d, d, 0.0]), -1)]   # 4: rear right, CW


@dataclass
class QuadrotorParams:
    """
    Physical parameters of the quadrotor (values close to the 3DR Iris used in PX4 SITL)
    mass_kg: Mass [kg]
    inertia_kgm2: Diagonal of the inertia matrix [Ixx, Iyy, Izz] [kg m^2]
    arm_length_m: Distance from the center to each rotor [m]
    max_rotor_thrust_n: Maximum thrust of one rotor [N]
    moment_coef_m: Yaw reaction moment per unit thrust [N m / N]
    linear_drag_coef_nspm: Linear aerodynamic drag coefficient [N s/m]
    gravity_mps2: Gravitational acceleration [m/s^2]
    """

    mass_kg: float = 1.5
    inertia_kgm2: np.ndarray = field(default_factory=lambda: np.array([0.029, 0.029, 0.055]))
    arm_length_m: float = 0.25
    max_rotor_thrust_n: float = 8.0
    moment_coef_m: float = 0.016
    linear_drag_coef_nspm: float = 0.1
    gravity_mps2: float = 9.81

    def rotors(self):
        return quad_x_rotors(self.arm_length_m)


@dataclass
class QuadrotorState:
    """
    State of the quadrotor
    position_ned_m: Position in NED [m]
    velocity_ned_mps: Velocity in NED [m/s]
    quaternion: Attitude [w, x, y, z], body -> NED
    angular_velocity_body_rps: Angular velocity in the body frame [rad/s]
    """

    position_ned_m: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity_ned_mps: np.ndarray = field(default_factory=lambda: np.zeros(3))
    quaternion: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    angular_velocity_body_rps: np.ndarray = field(default_factory=lambda: np.zeros(3))


class Quadrotor:
    """
    True dynamics of the quadrotor. Only the simulator touches this object;
    the navigation stack sees it only through sensors.
    """

    def __init__(self, params=QuadrotorParams(), initial_state=QuadrotorState()):
        """
        Constructor
        params: QuadrotorParams
        initial_state: QuadrotorState at time 0
        """

        self.params = params
        self.rotors = params.rotors()
        self.inertia_kgm2 = np.diag(params.inertia_kgm2)
        self.state = deepcopy(initial_state)
        self.accel_ned_mps2 = np.zeros(3)  # last acceleration, needed by the IMU

    def forces_and_moments(self, rotor_thrusts_n):
        """
        Collective thrust [N] and moment in the body frame [N m] produced by the rotors
        rotor_thrusts_n: Thrust of each rotor in PX4 motor order [N]
        """

        collective_thrust_n = 0.0
        moment_body_nm = np.zeros(3)
        for rotor, thrust_n in zip(self.rotors, rotor_thrusts_n):
            force_body_n = np.array([0.0, 0.0, -thrust_n])  # thrust points up = -z
            moment_body_nm += np.cross(rotor.position_body_m, force_body_n)
            # The motor turns the propeller one way, the propeller turns the body the other way
            moment_body_nm[2] += rotor.spin_direction * self.params.moment_coef_m * thrust_n
            collective_thrust_n += thrust_n
        return collective_thrust_n, moment_body_nm

    def step(self, rotor_thrusts_n, dt_s):
        """
        Advance the true state by one time step (semi-implicit Euler)
        rotor_thrusts_n: Thrust of each rotor in PX4 motor order [N]
        dt_s: Time step [s]
        """

        p = self.params
        s = self.state
        rotor_thrusts_n = np.clip(rotor_thrusts_n, 0.0, p.max_rotor_thrust_n)
        collective_thrust_n, moment_body_nm = self.forces_and_moments(rotor_thrusts_n)

        # Translational dynamics (Newton) in NED
        R = quat_to_dcm(s.quaternion)
        thrust_ned_n = R @ np.array([0.0, 0.0, -collective_thrust_n])
        drag_ned_n = -p.linear_drag_coef_nspm * s.velocity_ned_mps
        gravity_ned_mps2 = np.array([0.0, 0.0, p.gravity_mps2])
        accel_ned_mps2 = (thrust_ned_n + drag_ned_n) / p.mass_kg + gravity_ned_mps2

        # Rotational dynamics (Euler's equation) in the body frame
        omega = s.angular_velocity_body_rps
        gyroscopic_nm = np.cross(omega, self.inertia_kgm2 @ omega)
        angular_accel_rps2 = np.linalg.solve(self.inertia_kgm2, moment_body_nm - gyroscopic_nm)

        # Integrate
        s.velocity_ned_mps = s.velocity_ned_mps + accel_ned_mps2 * dt_s
        s.position_ned_m = s.position_ned_m + s.velocity_ned_mps * dt_s
        s.angular_velocity_body_rps = omega + angular_accel_rps2 * dt_s
        # Rotate the attitude by (angular velocity * dt), expressed in the body frame
        delta_q = quat_from_rotation_vector(s.angular_velocity_body_rps * dt_s)
        s.quaternion = quat_normalize(quat_multiply(s.quaternion, delta_q))

        # Simple ground contact at z = 0
        if s.position_ned_m[2] > 0.0:
            s.position_ned_m[2] = 0.0
            s.velocity_ned_mps = np.array([0.0, 0.0, min(s.velocity_ned_mps[2], 0.0)])
            accel_ned_mps2 = np.array([0.0, 0.0, min(accel_ned_mps2[2], 0.0)])

        self.accel_ned_mps2 = accel_ned_mps2

    def specific_force_body_mps2(self):
        """
        What an ideal accelerometer measures: acceleration minus gravity, in the body frame.
        At hover this is [0, 0, -g].
        """

        gravity_ned_mps2 = np.array([0.0, 0.0, self.params.gravity_mps2])
        R = quat_to_dcm(self.state.quaternion)
        return R.T @ (self.accel_ned_mps2 - gravity_ned_mps2)

    def hover_thrust_n(self):
        """
        Collective thrust that balances gravity [N]
        """

        return self.params.mass_kg * self.params.gravity_mps2
