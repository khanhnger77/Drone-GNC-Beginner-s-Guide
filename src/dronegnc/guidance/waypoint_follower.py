"""
waypoint_follower.py

Author: Khanh Ngo

Simplest guidance: fly to one waypoint after another. A waypoint counts as reached
inside an acceptance radius (PX4 mission: NAV_ACC_RAD). Works in 2D and 3D.
"""

import numpy as np
from math import atan2

from dronegnc.common.rotation import wrap_pi


class WaypointFollower:
    """
    Gives the current target waypoint
    """

    def __init__(self, waypoints_m, acceptance_radius_m=0.5):
        """
        Constructor
        waypoints_m: Waypoints as an (N, 2) or (N, 3) array in NED [m]
        acceptance_radius_m: Distance at which a waypoint counts as reached [m]
        """

        self.waypoints_m = np.asarray(waypoints_m, dtype=float)
        self.acceptance_radius_m = acceptance_radius_m
        self.index = 0

    def update(self, position_m):
        """
        Current target waypoint [m]
        position_m: Current position in NED [m]
        """

        target = self.waypoints_m[self.index]
        is_reached = np.linalg.norm(target - np.asarray(position_m)) < self.acceptance_radius_m
        if is_reached and not self.is_last():
            self.index += 1
        return self.waypoints_m[self.index]

    def is_last(self):
        return self.index == len(self.waypoints_m) - 1


def yaw_towards_target(target_ned_m, position_ned_m, previous_yaw_sp_rad, max_yaw_rate_rps, dt_s):
    """
    Yaw setpoint that turns the nose towards the target at a limited rate
    (PX4: MPC_YAW_MODE 0 "towards waypoint", and MPC_YAWRAUTO_MAX limits the turn rate)
    target_ned_m: Target position in NED [m]
    position_ned_m: Current position in NED [m]
    previous_yaw_sp_rad: Yaw setpoint of the last step [rad]
    max_yaw_rate_rps: Maximum change rate of the yaw setpoint [rad/s]
    dt_s: Time step [s]
    """

    north_m, east_m = np.asarray(target_ned_m)[:2] - np.asarray(position_ned_m)[:2]
    if np.hypot(north_m, east_m) < 1.0:
        return previous_yaw_sp_rad  # target (almost) straight above or below: keep the yaw
    yaw_error_rad = wrap_pi(atan2(east_m, north_m) - previous_yaw_sp_rad)
    max_step_rad = max_yaw_rate_rps * dt_s
    return wrap_pi(previous_yaw_sp_rad + np.clip(yaw_error_rad, -max_step_rad, max_step_rad))
