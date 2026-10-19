"""
vector_field_histogram.py

Author: Khanh Ngo

Reactive obstacle avoidance with a Vector Field Histogram (VFH, Borenstein & Koren 1991),
with the obstacle enlargement of VFH+ (Ulrich & Borenstein 1998).

Every scan:
  1. Polar histogram: each bin gets an obstacle magnitude that grows as the obstacle gets
     closer, m = 1 - d / d_max. Each reading also covers the neighboring bins within
     asin(safety_radius / d), so the drone (plus a margin) fits through what is left free.
  2. Threshold: bins above the threshold are blocked.
  3. Valleys: runs of free bins. Candidate directions are the valley center (narrow valley),
     or the two sides of a wide valley plus the goal direction when it lies inside.
  4. The candidate closest to the goal (and to the last choice, to avoid zig-zag) wins.
  5. Speed drops when obstacles are close ahead and when the goal is near.

It reads the same 72-bin polar data that PX4 publishes as obstacle_distance. PX4 itself
does not steer around obstacles onboard (only CollisionPrevention brakes); a VFH-like
planner runs offboard and sends trajectory setpoints.

The histogram step follows PolarHistogram in AutonomousVehicleControlBeginnersGuide.
"""

import numpy as np

from dronegnc.common.rotation import wrap_pi


class VectorFieldHistogram:
    """
    Horizontal velocity setpoint towards a goal, around the obstacles of an obstacle_distance scan
    """

    def __init__(self, cruise_speed_mps=2.5, safety_radius_m=0.8, threshold=0.7,
                 wide_valley_rad=np.radians(40.0), previous_direction_weight=1.0,
                 slow_down_distance_m=3.0, goal_speed_gain=0.8):
        """
        Constructor
        cruise_speed_mps: Speed when nothing is close [m/s]
        safety_radius_m: Drone radius plus margin, used to enlarge obstacles [m]
        threshold: Magnitude above which a bin is blocked (0..1). 0.7 with a 10 m sensor
                   blocks obstacles closer than 3 m
        wide_valley_rad: Valleys at least this wide get side candidates instead of their center [rad]
        previous_direction_weight: Cost weight of turning away from the last direction (goal weight = 1)
        slow_down_distance_m: Clearance ahead over which the speed ramps up to cruise speed [m]
        goal_speed_gain: P gain from goal distance to speed near the goal [1/s]
        """

        self.cruise_speed_mps = cruise_speed_mps
        self.safety_radius_m = safety_radius_m
        self.threshold = threshold
        self.wide_valley_rad = wide_valley_rad
        self.previous_direction_weight = previous_direction_weight
        self.slow_down_distance_m = slow_down_distance_m
        self.goal_speed_gain = goal_speed_gain
        self.direction_rad = None  # last chosen direction
        self.histogram = None      # last polar histogram, for plotting

    def polar_histogram(self, scan):
        """
        Obstacle magnitude of every bin (steps 1 of the algorithm)
        scan: ObstacleDistance
        """

        angles_rad = scan.bin_angles_rad()
        histogram = np.zeros(len(angles_rad))
        for distance_m, angle_rad in zip(scan.distances_m, angles_rad):
            if not np.isfinite(distance_m):
                continue
            magnitude = 1.0 - distance_m / scan.max_distance_m
            # VFH+ enlargement: the drone's center must stay safety_radius_m away from this point
            half_width_rad = np.arcsin(min(1.0, self.safety_radius_m / max(distance_m, 1e-6)))
            covered = np.abs(wrap_pi(angles_rad - angle_rad)) <= half_width_rad
            histogram[covered] = np.maximum(histogram[covered], magnitude)
        return histogram

    def candidate_directions_rad(self, blocked, angles_rad, goal_angle_rad, increment_rad):
        """
        Candidate directions from the valleys of free bins (step 3)
        """

        if not blocked.any():
            return [goal_angle_rad]
        # Walk around the circle once, starting after a blocked bin, and cut it into valleys
        num_bins = len(blocked)
        start = int(np.argmax(blocked))
        valleys, valley = [], []
        for k in range(1, num_bins + 1):
            i = (start + k) % num_bins
            if blocked[i]:
                if valley:
                    valleys.append(valley)
                valley = []
            else:
                valley.append(i)

        wide_bins = int(round(self.wide_valley_rad / increment_rad))
        goal_bin = int(round((goal_angle_rad % (2.0 * np.pi)) / increment_rad)) % num_bins
        candidates = []
        for valley in valleys:
            if len(valley) <= wide_bins:
                candidates.append(angles_rad[valley[len(valley) // 2]])  # narrow: go through the middle
            else:
                inner = valley[wide_bins // 2: len(valley) - wide_bins // 2]
                candidates += [angles_rad[inner[0]], angles_rad[inner[-1]]]  # keep away from both sides
                if goal_bin in inner:
                    candidates.append(goal_angle_rad)
        return candidates

    def update(self, position_ne_m, goal_ne_m, scan):
        """
        Velocity setpoint [North, East] [m/s]
        position_ne_m: Current position [North, East] [m]
        goal_ne_m: Goal [North, East] [m]
        scan: Latest ObstacleDistance
        """

        to_goal = np.asarray(goal_ne_m) - np.asarray(position_ne_m)
        goal_distance_m = np.linalg.norm(to_goal)
        goal_angle_rad = np.arctan2(to_goal[1], to_goal[0])
        if self.direction_rad is None:
            self.direction_rad = goal_angle_rad

        # Steps 1-3
        self.histogram = self.polar_histogram(scan)
        blocked = self.histogram > self.threshold
        candidates = self.candidate_directions_rad(blocked, scan.bin_angles_rad(), goal_angle_rad,
                                                   scan.increment_rad)
        if not candidates:
            return np.zeros(2)  # surrounded: hover

        # Step 4: closest to the goal, and not too far from the last choice
        def cost(angle_rad):
            return (abs(wrap_pi(angle_rad - goal_angle_rad))
                    + self.previous_direction_weight * abs(wrap_pi(angle_rad - self.direction_rad)))
        self.direction_rad = min(candidates, key=cost)

        # Step 5: slow down when obstacles are close ahead (within 45 degrees of the direction),
        #         and near the goal
        ahead = np.abs(wrap_pi(scan.bin_angles_rad() - self.direction_rad)) <= np.radians(45.0)
        clearance_m = np.min(scan.distances_m[ahead]) - self.safety_radius_m
        speed_mps = self.cruise_speed_mps * np.clip(clearance_m / self.slow_down_distance_m, 0.2, 1.0)
        speed_mps = min(speed_mps, self.goal_speed_gain * goal_distance_m)
        return speed_mps * np.array([np.cos(self.direction_rad), np.sin(self.direction_rad)])
