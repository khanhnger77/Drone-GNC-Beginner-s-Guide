"""
astar_global_planning.py

Author: Khanh Ngo

Lesson: global path planning with A*, the way out of the trap of reactive methods.
The world is the U-shaped wall where both VFH and the potential field get stuck.
A* searches an occupancy grid of the known map before take-off, shortcuts the cell
path to a few waypoints, and the quadrotor flies them with the waypoint follower and
the PX4-style cascade, like a PX4 mission planned in QGroundControl.
The controller uses the TRUE state here. The range sensor only feeds the drawing.
"""

import numpy as np
import matplotlib.pyplot as plt

from dronegnc.vehicle.quadrotor import Quadrotor, QuadrotorParams, QuadrotorState
from dronegnc.sensors.obstacle_distance_sensor import ObstacleDistanceSensor
from dronegnc.guidance.astar_planner import OccupancyGrid, astar, shortcut
from dronegnc.guidance.waypoint_follower import WaypointFollower, yaw_towards_target
from dronegnc.control.multicopter_cascade_controller import MulticopterCascadeController
from dronegnc.control.control_allocation import ControlAllocation
from dronegnc.simulation.obstacles import ObstacleMap, u_shaped_wall
from dronegnc.simulation.history import History
from dronegnc.visualization.top_view import animate_top_view

show_plot = True

ALTITUDE_M = 2.0
START_NE_M = np.array([0.0, 0.0])
GOAL_NE_M = np.array([24.0, 0.0])


def make_world():
    """
    The U-trap of the potential field lesson
    """

    return ObstacleMap(u_shaped_wall(back_north_m=14.0, depth_m=6.0, half_width_m=4.0))


def main():
    dt_s = 0.005
    duration_s = 30.0
    rng = np.random.default_rng(seed=0)
    params = QuadrotorParams()

    # Truth: vehicle and world
    vehicle = Quadrotor(params, QuadrotorState(position_ned_m=np.array([*START_NE_M, -ALTITUDE_M])))
    obstacle_map = make_world()
    range_sensor = ObstacleDistanceSensor(obstacle_map, rng=rng)

    # Plan once, before take-off, on the known map
    grid = OccupancyGrid(obstacle_map, north_range_m=(-3.0, 27.0), east_range_m=(-9.0, 9.0),
                         resolution_m=0.25, inflation_radius_m=1.2)
    cell_path_ne_m = astar(grid, START_NE_M, GOAL_NE_M)
    waypoints_ne_m = shortcut(grid, cell_path_ne_m)
    waypoints_ned_m = np.column_stack([waypoints_ne_m, np.full(len(waypoints_ne_m), -ALTITUDE_M)])

    # Fly the waypoints
    guidance = WaypointFollower(waypoints_ned_m[1:], acceptance_radius_m=0.5)
    controller = MulticopterCascadeController(params)
    allocation = ControlAllocation(vehicle.rotors, params.moment_coef_m, params.max_rotor_thrust_n)
    history = History()
    yaw_sp_rad = 0.0
    scan = None

    for step in range(int(duration_s / dt_s)):
        time_s = step * dt_s
        s = vehicle.state

        new_scan = range_sensor.measure(vehicle, time_s)
        scan = new_scan if new_scan is not None else scan

        target_ned_m = guidance.update(s.position_ned_m)
        yaw_sp_rad = yaw_towards_target(target_ned_m, s.position_ned_m, yaw_sp_rad, np.radians(60.0), dt_s)
        thrust_n, moment_nm = controller.update(target_ned_m, yaw_sp_rad, s.quaternion,
                                                s.angular_velocity_body_rps, s.position_ned_m,
                                                s.velocity_ned_mps, dt_s)
        vehicle.step(allocation.allocate(thrust_n, moment_nm), dt_s)

        history.record(time_s=time_s + dt_s, true_position_ned_m=s.position_ned_m,
                       clearance_m=obstacle_map.distance_to_nearest_m(s.position_ned_m[:2]),
                       scan_distances_m=scan.distances_m, scan_increment_rad=scan.increment_rad)

    if show_plot:
        fig, ax = plt.subplots(figsize=(6, 7))
        north_m = grid.origin_ne_m[0] + grid.resolution_m * np.arange(grid.occupied.shape[0])
        east_m = grid.origin_ne_m[1] + grid.resolution_m * np.arange(grid.occupied.shape[1])
        ax.pcolormesh(east_m, north_m, grid.occupied, cmap="Greys", alpha=0.5, shading="nearest")
        cell_path = np.array(cell_path_ne_m)
        ax.plot(cell_path[:, 1], cell_path[:, 0], "-", color="#bf8700",
                label=f"A* cell path ({len(cell_path_ne_m)} cells)")
        ax.plot(waypoints_ne_m[:, 1], waypoints_ne_m[:, 0], "o--", color="#e5534b",
                label=f"waypoints after shortcut ({len(waypoints_ne_m)})")
        ax.set_aspect("equal")
        ax.set_xlabel("East [m]")
        ax.set_ylabel("North [m]")
        ax.legend(loc="lower right")
        ax.set_title("Occupancy grid (inflated) and A* path")

        anim_fig, animation = animate_top_view(history, obstacle_map, GOAL_NE_M, title="A* + waypoints")
        anim_fig.axes[0].plot(waypoints_ne_m[:, 1], waypoints_ne_m[:, 0], "x--", color="#e5534b", alpha=0.6)
        plt.show()

    return history


if __name__ == "__main__":
    main()
