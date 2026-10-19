"""
obstacles.py

Author: Khanh Ngo

True obstacles of the simulated world, in the horizontal North-East plane.
Obstacles are infinitely tall (pillars and walls), so the drone cannot fly over them
and avoidance is a 2D problem at any altitude. Only the simulator and the range
sensor use this module; the avoidance algorithms see only sensor measurements.
"""

import numpy as np
from dataclasses import dataclass


@dataclass
class CircleObstacle:
    """
    A vertical pillar
    center_ne_m: Center [North, East] [m]
    radius_m: Radius [m]
    """

    center_ne_m: np.ndarray
    radius_m: float

    def ray_distance_m(self, origin_ne_m, direction_ne):
        """
        Distance along a ray to the pillar's surface [m], or inf if the ray misses it.
        Solves |origin + t * direction - center| = radius for the smallest t >= 0.
        """

        to_origin = origin_ne_m - self.center_ne_m
        b = to_origin @ direction_ne
        c = to_origin @ to_origin - self.radius_m ** 2
        discriminant = b * b - c
        if discriminant < 0.0:
            return np.inf
        for t in (-b - np.sqrt(discriminant), -b + np.sqrt(discriminant)):
            if t >= 0.0:
                return t
        return np.inf

    def distance_m(self, point_ne_m):
        """
        Distance from a point to the pillar's surface [m] (negative inside)
        """

        return np.linalg.norm(point_ne_m - self.center_ne_m) - self.radius_m


@dataclass
class WallObstacle:
    """
    A vertical wall of zero thickness
    start_ne_m, end_ne_m: End points [North, East] [m]
    """

    start_ne_m: np.ndarray
    end_ne_m: np.ndarray

    def ray_distance_m(self, origin_ne_m, direction_ne):
        """
        Distance along a ray to the wall [m], or inf if the ray misses it.
        Solves origin + t * direction = start + s * (end - start) with t >= 0 and 0 <= s <= 1.
        """

        wall = self.end_ne_m - self.start_ne_m
        denominator = direction_ne[0] * wall[1] - direction_ne[1] * wall[0]
        if abs(denominator) < 1e-12:
            return np.inf  # parallel
        to_start = self.start_ne_m - origin_ne_m
        t = (to_start[0] * wall[1] - to_start[1] * wall[0]) / denominator
        s = (to_start[0] * direction_ne[1] - to_start[1] * direction_ne[0]) / denominator
        return t if (t >= 0.0 and 0.0 <= s <= 1.0) else np.inf

    def distance_m(self, point_ne_m):
        """
        Distance from a point to the wall [m]
        """

        wall = self.end_ne_m - self.start_ne_m
        s = np.clip((point_ne_m - self.start_ne_m) @ wall / (wall @ wall), 0.0, 1.0)
        return np.linalg.norm(point_ne_m - (self.start_ne_m + s * wall))


class ObstacleMap:
    """
    All obstacles of the world
    """

    def __init__(self, obstacles):
        """
        Constructor
        obstacles: List of CircleObstacle and WallObstacle
        """

        self.obstacles = obstacles

    def ray_distance_m(self, origin_ne_m, angle_rad, max_range_m):
        """
        Distance to the first obstacle along a ray [m], or inf if nothing is within max_range_m
        origin_ne_m: Start of the ray [North, East] [m]
        angle_rad: Direction of the ray, 0 = North, positive towards East (like yaw) [rad]
        max_range_m: Longest distance that counts [m]
        """

        origin_ne_m = np.asarray(origin_ne_m, dtype=float)
        direction_ne = np.array([np.cos(angle_rad), np.sin(angle_rad)])
        distance_m = min((o.ray_distance_m(origin_ne_m, direction_ne) for o in self.obstacles), default=np.inf)
        return distance_m if distance_m <= max_range_m else np.inf

    def distance_to_nearest_m(self, point_ne_m):
        """
        Distance from a point to the closest obstacle surface [m]. Negative = inside a pillar (a crash).
        """

        point_ne_m = np.asarray(point_ne_m, dtype=float)
        return min(o.distance_m(point_ne_m) for o in self.obstacles)


def pillar_forest(rng, num_pillars, north_range_m, east_range_m, radius_range_m, keep_free_ne_m, free_radius_m):
    """
    Random pillars, kept away from some points (start, goal)
    rng: numpy random Generator (seeded, for reproducible worlds)
    num_pillars: Number of pillars
    north_range_m, east_range_m: (min, max) of the pillar centers [m]
    radius_range_m: (min, max) of the pillar radius [m]
    keep_free_ne_m: List of [North, East] points that must stay free [m]
    free_radius_m: No pillar surface closer than this to those points [m]
    """

    pillars = []
    while len(pillars) < num_pillars:
        pillar = CircleObstacle(np.array([rng.uniform(*north_range_m), rng.uniform(*east_range_m)]),
                                rng.uniform(*radius_range_m))
        if all(pillar.distance_m(np.asarray(p)) > free_radius_m for p in keep_free_ne_m):
            pillars.append(pillar)
    return pillars


def u_shaped_wall(back_north_m, depth_m, half_width_m):
    """
    Three walls forming a U that opens towards the South, centered on East = 0.
    A reactive planner coming from the South with a goal behind the U gets trapped inside.
    back_north_m: North position of the back wall [m]
    depth_m: Length of the two side walls [m]
    half_width_m: Half the distance between the side walls [m]
    """

    back, front = back_north_m, back_north_m - depth_m
    return [WallObstacle(np.array([back, -half_width_m]), np.array([back, half_width_m])),
            WallObstacle(np.array([front, -half_width_m]), np.array([back, -half_width_m])),
            WallObstacle(np.array([front, half_width_m]), np.array([back, half_width_m]))]
