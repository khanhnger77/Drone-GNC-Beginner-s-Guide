"""
collision_prevention.py

Author: Khanh Ngo

Collision prevention like PX4 (src/lib/collision_prevention/CollisionPrevention.cpp).
It does not plan a way around obstacles: it only limits the velocity towards the
closest obstacle so the vehicle can always stop CP_DIST before it. The velocity along
the obstacle is kept, so the vehicle slides along walls, unless something blocks that
direction too (a corner).

PX4 runs it only in Position mode, on the pilot's stick setpoint. Missions and offboard
setpoints are not protected: avoidance there is the job of a planner (see
vector_field_histogram.py and potential_field.py).

Simplifications compared to PX4 v1.17.0, for readability:
- The limit is applied to a velocity setpoint. PX4 applies the same idea to the
  acceleration setpoint of StickAccelerationXY.
- The largest speed that still stops in time uses a constant deceleration,
  v = sqrt(2 * a * d). PX4 also limits jerk (computeMaxSpeedFromDistance).
"""

import numpy as np


class CollisionPrevention:
    """
    Limits a horizontal velocity setpoint using an obstacle_distance scan
    """

    def __init__(self, min_distance_m=1.5, delay_s=0.4, max_decel_mps2=3.0):
        """
        Constructor
        min_distance_m: Distance to keep to every obstacle [m] (PX4: CP_DIST)
        delay_s: Sensor and control delay; the vehicle keeps flying this long before it brakes [s]
                 (PX4: CP_DELAY)
        max_decel_mps2: Deceleration used to brake [m/s^2] (PX4: MPC_ACC_HOR)
        """

        self.min_distance_m = min_distance_m
        self.delay_s = delay_s
        self.max_decel_mps2 = max_decel_mps2

    def max_speed_towards_mps(self, distance_m, speed_towards_mps):
        """
        Largest speed towards an obstacle at distance_m that still stops min_distance_m before it
        """

        if not np.isfinite(distance_m):
            return np.inf
        stop_distance_m = distance_m - self.min_distance_m - speed_towards_mps * self.delay_s
        return np.sqrt(2.0 * self.max_decel_mps2 * max(stop_distance_m, 0.0))

    def modify_setpoint(self, velocity_sp_ne_mps, obstacle_distance):
        """
        Constrained velocity setpoint [North, East] [m/s]
        velocity_sp_ne_mps: Desired horizontal velocity (e.g. from the sticks) [m/s]
        obstacle_distance: Latest ObstacleDistance scan, or None if there is no data yet
        """

        if obstacle_distance is None:
            return np.zeros(2)  # no data: do not move (PX4: CP_GO_NO_DATA = 0)

        velocity_sp = np.array(velocity_sp_ne_mps, dtype=float)
        distances_m = obstacle_distance.distances_m
        closest = int(np.argmin(distances_m))
        if not np.isfinite(distances_m[closest]):
            return velocity_sp  # nothing in range
        normal_dir = obstacle_distance.bin_directions_ne()[closest]

        speed_towards_mps = velocity_sp @ normal_dir
        if speed_towards_mps <= 0.0:
            return velocity_sp  # flying away from the closest obstacle: no limit

        # Split the setpoint into a part towards the closest obstacle and a part along it
        # (PX4: _constrainAccelerationSetpoint, normal and tangential components)
        tangential = velocity_sp - speed_towards_mps * normal_dir
        normal_speed_mps = min(speed_towards_mps,
                               self.max_speed_towards_mps(distances_m[closest], speed_towards_mps))

        # The part along the obstacle is limited by what lies in its own direction (a corner)
        tangential_speed_mps = np.linalg.norm(tangential)
        if tangential_speed_mps > 1e-6:
            distance_along_m = self.distance_in_direction_m(obstacle_distance, tangential)
            max_tangential_mps = self.max_speed_towards_mps(distance_along_m, tangential_speed_mps)
            tangential *= min(1.0, max_tangential_mps / tangential_speed_mps)

        return normal_speed_mps * normal_dir + tangential

    @staticmethod
    def distance_in_direction_m(obstacle_distance, direction_ne):
        """
        Reading of the bin that contains a direction [m] (PX4: _getObstacleDistance)
        """

        angle_rad = np.arctan2(direction_ne[1], direction_ne[0]) % (2.0 * np.pi)
        index = int(round(angle_rad / obstacle_distance.increment_rad)) % len(obstacle_distance.distances_m)
        return obstacle_distance.distances_m[index]
