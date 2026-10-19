"""
potential_field.py

Author: Khanh Ngo

Reactive obstacle avoidance with an artificial potential field (Khatib 1986).
The goal attracts, every obstacle reading repels, and the drone flies along the sum:

  attractive  = k_att * (goal - position), capped at max_speed
  repulsive_i = k_rep * (1/d_i - 1/d_0) / d_i^2   away from reading i, only if d_i < d_0
  velocity    = attractive + sum(repulsive_i), limited to max_speed

Simple and smooth, but the forces can cancel out before the goal: a local minimum,
for example inside a U-shaped obstacle. The lesson shows that trap next to VFH.

The repulsion is computed from the same 72-bin obstacle_distance data as PX4 uses.
The potential field map in AutonomousVehicleControlBeginnersGuide is the grid version of this.
"""

import numpy as np


class PotentialField:
    """
    Horizontal velocity setpoint from attractive and repulsive forces
    """

    def __init__(self, attractive_gain=0.5, repulsive_gain=2.0, influence_distance_m=3.0, max_speed_mps=2.5):
        """
        Constructor
        attractive_gain: k_att, from goal distance [m] to speed [m/s] [1/s]
        repulsive_gain: k_rep, strength of the repulsion [m^2/s]
        influence_distance_m: d_0, obstacles farther than this do not repel [m]
        max_speed_mps: Limit of the resulting speed [m/s]
        """

        self.attractive_gain = attractive_gain
        self.repulsive_gain = repulsive_gain
        self.influence_distance_m = influence_distance_m
        self.max_speed_mps = max_speed_mps

    def update(self, position_ne_m, goal_ne_m, scan):
        """
        Velocity setpoint [North, East] [m/s]
        position_ne_m: Current position [North, East] [m]
        goal_ne_m: Goal [North, East] [m]
        scan: Latest ObstacleDistance
        """

        # Attraction, capped at max_speed so that it cannot overpower the repulsion near obstacles
        velocity = self.attractive_gain * (np.asarray(goal_ne_m) - np.asarray(position_ne_m))
        attraction_mps = np.linalg.norm(velocity)
        if attraction_mps > self.max_speed_mps:
            velocity *= self.max_speed_mps / attraction_mps
        for distance_m, direction in zip(scan.distances_m, scan.bin_directions_ne()):
            if distance_m < self.influence_distance_m:
                d = max(distance_m, 0.1)
                strength = self.repulsive_gain * (1.0 / d - 1.0 / self.influence_distance_m) / d ** 2
                velocity -= strength * direction  # push away from the obstacle
        speed_mps = np.linalg.norm(velocity)
        if speed_mps > self.max_speed_mps:
            velocity *= self.max_speed_mps / speed_mps
        return velocity
