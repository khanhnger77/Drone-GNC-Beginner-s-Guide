"""
multicopter_cascade_controller.py

Author: Khanh Ngo

Cascaded controller for the 3D quadrotor with the same loops as the PX4 multicopter stack:

  position --P--> velocity --PID--> acceleration --> thrust vector     (mc_pos_control)
  thrust vector + yaw --> attitude setpoint                            (ControlMath::bodyzToAttitude)
  attitude --P (quaternion error)--> body rates                        (mc_att_control)
  body rates --PID--> moment                                           (mc_rate_control)

The result (collective thrust, moment) goes to ControlAllocation.
"""

import numpy as np
from dataclasses import dataclass
from math import cos, sin, tan, radians

from dronegnc.common.rotation import quat_multiply, quat_conjugate, dcm_to_quat
from dronegnc.control.pid import Pid


@dataclass
class MulticopterSetpoints:
    """
    Intermediate outputs of every loop, for plotting and debugging
    """

    velocity_ned_mps: np.ndarray
    accel_ned_mps2: np.ndarray
    quaternion: np.ndarray
    body_rates_rps: np.ndarray
    collective_thrust_n: float
    moment_body_nm: np.ndarray


def thrust_vector_to_attitude(thrust_ned_n, yaw_rad):
    """
    Attitude whose body -z axis points along the thrust vector, with the given yaw
    (same construction as PX4 ControlMath::bodyzToAttitude)
    thrust_ned_n: Desired thrust vector in NED [N]
    yaw_rad: Desired yaw [rad]
    """

    body_z = -thrust_ned_n / np.linalg.norm(thrust_ned_n)
    y_c = np.array([-sin(yaw_rad), cos(yaw_rad), 0.0])  # East-ish axis after the yaw rotation
    body_x = np.cross(y_c, body_z)
    body_x /= np.linalg.norm(body_x)
    body_y = np.cross(body_z, body_x)
    return dcm_to_quat(np.column_stack([body_x, body_y, body_z]))


class MulticopterCascadeController:
    """
    Position and yaw controller of the 3D quadrotor
    """

    def __init__(self, params, position_gain=1.0, velocity_gains=(3.0, 1.0, 0.0),
                 attitude_gains=(8.0, 8.0, 4.0), rate_gains=((20.0, 20.0, 8.0), (2.0, 2.0, 1.0), (0.0, 0.0, 0.0)),
                 max_horizontal_speed_mps=5.0, max_vertical_speed_mps=3.0,
                 max_tilt_rad=radians(35.0), max_body_rates_rps=np.radians([220.0, 220.0, 200.0])):
        """
        Constructor
        params: QuadrotorParams of the controlled vehicle
        position_gain: P gain from position error [m] to velocity setpoint [m/s] [1/s]
        velocity_gains: (kp, ki, kd) from velocity error [m/s] to acceleration [m/s^2]
        attitude_gains: P gains [roll, pitch, yaw] from attitude error [rad] to body rates [rad/s] [1/s]
        rate_gains: (kp, ki, kd), each [roll, pitch, yaw], from rate error [rad/s] to angular accel [rad/s^2]
        max_horizontal_speed_mps, max_vertical_speed_mps: Limits of the velocity setpoint [m/s]
        max_tilt_rad: Limit of the tilt of the thrust vector [rad]
        max_body_rates_rps: Limits of the body rate setpoint [roll, pitch, yaw] [rad/s]
        """

        self.params = params
        self.inertia_kgm2 = np.diag(params.inertia_kgm2)
        self.position_gain = position_gain
        self.velocity_pid = Pid(*velocity_gains, integral_limit=3.0)
        self.attitude_gains = np.asarray(attitude_gains)
        self.rate_pid = Pid(*rate_gains, integral_limit=10.0)
        self.max_horizontal_speed_mps = max_horizontal_speed_mps
        self.max_vertical_speed_mps = max_vertical_speed_mps
        self.max_tilt_rad = max_tilt_rad
        self.max_body_rates_rps = max_body_rates_rps
        self.setpoints = None

    def update(self, position_sp_ned_m, yaw_sp_rad, quaternion, angular_velocity_rps,
               position_ned_m, velocity_ned_mps, dt_s, velocity_ff_ned_mps=None):
        """
        Collective thrust [N] and moment in the body frame [N m]
        position_sp_ned_m: Desired position in NED [m]. NaN on an axis = no position control on it
        yaw_sp_rad: Desired yaw [rad]
        quaternion, angular_velocity_rps: Attitude and body rates used for feedback
        position_ned_m, velocity_ned_mps: Position and velocity used for feedback
        dt_s: Time step [s]
        velocity_ff_ned_mps: Optional velocity feed-forward in NED [m/s], added to the position loop output
        """

        p = self.params

        # 1. Position loop (P), plus the velocity feed-forward. Like PX4 trajectory_setpoint, a NaN
        #    position axis is velocity-controlled only (PositionControl: addIfNotNanVector3f).
        #    Then separate horizontal and vertical speed limits.
        velocity_sp = np.nan_to_num(self.position_gain * (np.asarray(position_sp_ned_m) - position_ned_m))
        if velocity_ff_ned_mps is not None:
            velocity_sp = velocity_sp + velocity_ff_ned_mps
        horizontal_speed = np.linalg.norm(velocity_sp[:2])
        if horizontal_speed > self.max_horizontal_speed_mps:
            velocity_sp[:2] *= self.max_horizontal_speed_mps / horizontal_speed
        velocity_sp[2] = np.clip(velocity_sp[2], -self.max_vertical_speed_mps, self.max_vertical_speed_mps)

        # 2. Velocity loop (PID) -> acceleration setpoint
        accel_sp = self.velocity_pid.update(velocity_sp, velocity_ned_mps, dt_s)

        # 3. Acceleration -> thrust vector F = m * (a - g), always pushing up a little
        thrust_ned = p.mass_kg * (accel_sp - np.array([0.0, 0.0, p.gravity_mps2]))
        thrust_ned[2] = min(thrust_ned[2], -0.1 * p.mass_kg * p.gravity_mps2)
        # Tilt limit: shrink the horizontal part, keep the vertical part (altitude first)
        max_horizontal_n = -thrust_ned[2] * tan(self.max_tilt_rad)
        horizontal_n = np.linalg.norm(thrust_ned[:2])
        if horizontal_n > max_horizontal_n:
            thrust_ned[:2] *= max_horizontal_n / horizontal_n
        collective_thrust_n = min(np.linalg.norm(thrust_ned), 4.0 * p.max_rotor_thrust_n)

        # 4. Thrust direction + yaw -> attitude setpoint
        quaternion_sp = thrust_vector_to_attitude(thrust_ned, yaw_sp_rad)

        # 5. Attitude loop: error quaternion q_err = q^-1 * q_sp, rate = 2 * K * vector part.
        #    (PX4 also gives yaw a lower priority than roll/pitch; omitted for clarity)
        q_error = quat_multiply(quat_conjugate(quaternion), quaternion_sp)
        if q_error[0] < 0.0:
            q_error = -q_error  # take the shorter way around
        body_rates_sp = np.clip(2.0 * self.attitude_gains * q_error[1:],
                                -self.max_body_rates_rps, self.max_body_rates_rps)

        # 6. Rate loop (PID) -> angular acceleration -> moment
        angular_accel_sp = self.rate_pid.update(body_rates_sp, angular_velocity_rps, dt_s)
        moment_body_nm = self.inertia_kgm2 @ angular_accel_sp

        self.setpoints = MulticopterSetpoints(velocity_sp, accel_sp, quaternion_sp, body_rates_sp,
                                              collective_thrust_n, moment_body_nm)
        return collective_thrust_n, moment_body_nm
