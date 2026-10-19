"""
test_obstacle_avoidance.py

Author: Khanh Ngo

Unit tests of the obstacles, the obstacle_distance sensor and the avoidance algorithms.
"""

import numpy as np

from dronegnc.vehicle.quadrotor import Quadrotor, QuadrotorParams, QuadrotorState
from dronegnc.simulation.obstacles import CircleObstacle, WallObstacle, ObstacleMap, u_shaped_wall
from dronegnc.sensors.obstacle_distance_sensor import ObstacleDistance, ObstacleDistanceSensor
from dronegnc.guidance.collision_prevention import CollisionPrevention
from dronegnc.guidance.vector_field_histogram import VectorFieldHistogram
from dronegnc.guidance.astar_planner import OccupancyGrid, astar, shortcut


def scan_with(readings_m, num_bins=72, max_distance_m=10.0):
    """
    A scan with no obstacle except the given {bin index: distance}
    """

    distances_m = np.full(num_bins, np.inf)
    for index, distance_m in readings_m.items():
        distances_m[index] = distance_m
    return ObstacleDistance(distances_m, 2.0 * np.pi / num_bins, max_distance_m)


def test_ray_distances():
    pillar = CircleObstacle(np.array([5.0, 0.0]), 1.0)
    wall = WallObstacle(np.array([-2.0, 3.0]), np.array([2.0, 3.0]))
    world = ObstacleMap([pillar, wall])
    assert np.isclose(world.ray_distance_m([0.0, 0.0], 0.0, 10.0), 4.0)        # North hits the pillar
    assert np.isclose(world.ray_distance_m([0.0, 0.0], np.pi / 2, 10.0), 3.0)  # East hits the wall
    assert world.ray_distance_m([0.0, 0.0], np.pi, 10.0) == np.inf             # South: nothing
    assert world.ray_distance_m([0.0, 0.0], 0.0, 3.0) == np.inf                # out of range
    assert np.isclose(world.distance_to_nearest_m([0.0, 1.0]), 2.0)


def test_obstacle_distance_layout_matches_px4():
    # Bin 0 is North and angles grow towards East (PX4 obstacle_distance, MAV_FRAME_LOCAL_NED)
    world = ObstacleMap([CircleObstacle(np.array([0.0, 4.0]), 0.5)])  # pillar due East
    vehicle = Quadrotor(QuadrotorParams(), QuadrotorState(position_ned_m=np.array([0.0, 0.0, -2.0])))
    scan = ObstacleDistanceSensor(world, noise_std_m=0.0).measure(vehicle, 0.0)
    assert len(scan.distances_m) == 72 and np.isclose(np.degrees(scan.increment_rad), 5.0)
    assert int(np.argmin(scan.distances_m)) == 18  # 90 degrees / 5 degrees
    assert np.isclose(scan.distances_m[18], 3.5)
    assert scan.distances_m[0] == np.inf


def test_collision_prevention_brakes_only_towards_obstacles():
    cp = CollisionPrevention(min_distance_m=1.5)
    scan = scan_with({18: 1.5})  # wall due East at CP_DIST
    # Towards the wall: the East part is removed, the North part (sliding) stays
    assert np.allclose(cp.modify_setpoint([1.0, 2.0], scan), [1.0, 0.0])
    # Away from the wall: unchanged
    assert np.allclose(cp.modify_setpoint([0.0, -2.0], scan), [0.0, -2.0])
    # Far from the wall: full speed is still allowed
    assert np.allclose(cp.modify_setpoint([0.0, 2.0], scan_with({18: 9.0})), [0.0, 2.0])
    # No data: do not move
    assert np.allclose(cp.modify_setpoint([1.0, 1.0], None), [0.0, 0.0])


def test_vfh_steers_around_an_obstacle_ahead():
    vfh = VectorFieldHistogram()
    scan = scan_with({0: 2.0})  # obstacle straight ahead (North), goal North
    velocity = vfh.update([0.0, 0.0], [20.0, 0.0], scan)
    direction_deg = np.degrees(np.arctan2(velocity[1], velocity[0]))
    assert 20.0 < abs(direction_deg) < 90.0  # turns aside, but still forward
    # Nothing around: straight to the goal
    vfh = VectorFieldHistogram()
    velocity = vfh.update([0.0, 0.0], [0.0, 20.0], scan_with({}))
    assert np.allclose(velocity / np.linalg.norm(velocity), [0.0, 1.0])


def test_astar_finds_a_free_path_around_the_u():
    world = ObstacleMap(u_shaped_wall(back_north_m=14.0, depth_m=6.0, half_width_m=4.0))
    grid = OccupancyGrid(world, (-3.0, 27.0), (-9.0, 9.0), resolution_m=0.5, inflation_radius_m=1.0)
    path = astar(grid, np.array([0.0, 0.0]), np.array([24.0, 0.0]))
    assert path is not None
    assert all(world.distance_to_nearest_m(p) >= 1.0 - 0.5 for p in path)
    waypoints = shortcut(grid, path)
    assert len(waypoints) < len(path) / 5
    assert all(grid.is_line_free(a, b) for a, b in zip(waypoints[:-1], waypoints[1:]))
