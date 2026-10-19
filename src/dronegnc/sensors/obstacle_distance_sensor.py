"""
obstacle_distance_sensor.py

Author: Khanh Ngo

A 360 degree horizontal range sensor (a scanning lidar) whose output has the layout of
the PX4 uORB topic obstacle_distance: 72 bins of 5 degrees, bin 0 centered on North,
angles growing clockwise (towards East), each bin holding the closest distance.

Differences to PX4, for readability: distances are in meters (PX4: uint16 centimeters),
and "no obstacle in range" is inf (PX4: max_distance + 1).
"""

import numpy as np
from dataclasses import dataclass

from dronegnc.sensors.sample_timer import SampleTimer


@dataclass
class ObstacleDistance:
    """
    One scan (PX4: obstacle_distance, frame MAV_FRAME_LOCAL_NED)
    distances_m: Closest distance in each bin [m], inf = nothing within max_distance_m
    increment_rad: Angular width of a bin [rad]
    max_distance_m: Range of the sensor [m]
    """

    distances_m: np.ndarray
    increment_rad: float
    max_distance_m: float

    def bin_angles_rad(self):
        """
        Center angle of every bin, 0 = North, positive towards East [rad]
        """

        return np.arange(len(self.distances_m)) * self.increment_rad

    def bin_directions_ne(self):
        """
        Unit vector [North, East] of every bin, shape (num_bins, 2)
        """

        angles_rad = self.bin_angles_rad()
        return np.column_stack([np.cos(angles_rad), np.sin(angles_rad)])


class ObstacleDistanceSensor:
    """
    Casts a few rays per bin against the true obstacles and keeps the closest hit,
    like PX4 does when it fills obstacle_distance from a lidar with finer resolution.
    """

    def __init__(self, obstacle_map, num_bins=72, rays_per_bin=3, max_distance_m=10.0,
                 noise_std_m=0.02, rate_hz=10.0, rng=None):
        """
        Constructor
        obstacle_map: ObstacleMap of the true world
        num_bins: Number of bins over 360 degrees (PX4: 72)
        rays_per_bin: Rays cast inside each bin
        max_distance_m: Range of the sensor [m]
        noise_std_m: Standard deviation of the distance noise [m]
        rate_hz: Output rate [Hz]
        rng: numpy random Generator (for reproducible runs)
        """

        self.obstacle_map = obstacle_map
        self.increment_rad = 2.0 * np.pi / num_bins
        self.num_bins = num_bins
        # Ray offsets spread over the bin, e.g. -1/3, 0, +1/3 of the bin width
        self.ray_offsets_rad = (np.arange(rays_per_bin) - (rays_per_bin - 1) / 2) * self.increment_rad / rays_per_bin
        self.max_distance_m = max_distance_m
        self.noise_std_m = noise_std_m
        self.timer = SampleTimer(rate_hz)
        self.rng = rng if rng is not None else np.random.default_rng()

    def measure(self, vehicle, time_s):
        """
        ObstacleDistance when a new scan is due, otherwise None
        vehicle: Quadrotor (the sensor is at its center)
        time_s: Current simulation time [s]
        """

        if not self.timer.is_due(time_s):
            return None
        position_ne_m = vehicle.state.position_ned_m[:2]
        distances_m = np.full(self.num_bins, np.inf)
        for i in range(self.num_bins):
            for offset_rad in self.ray_offsets_rad:
                hit_m = self.obstacle_map.ray_distance_m(position_ne_m, i * self.increment_rad + offset_rad,
                                                         self.max_distance_m)
                distances_m[i] = min(distances_m[i], hit_m)
        hits = np.isfinite(distances_m)
        distances_m[hits] = np.maximum(distances_m[hits] + self.rng.normal(0.0, self.noise_std_m, hits.sum()), 0.0)
        return ObstacleDistance(distances_m, self.increment_rad, self.max_distance_m)
