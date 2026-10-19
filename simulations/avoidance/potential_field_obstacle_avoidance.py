"""
potential_field_obstacle_avoidance.py

Author: Khanh Ngo

Lesson: reactive obstacle avoidance with an artificial potential field, and its weak point.
The same quadrotor, sensor and controller as the VFH lesson fly two worlds:
  1. a sparse pillar forest: the potential field reaches the goal;
  2. a U-shaped wall in front of the goal: attraction and repulsion cancel inside the U,
     and the drone is stuck there (a local minimum).
VFH gets trapped in the U as well, and the potential field also fails in the VFH lesson's
denser forest: reactive methods only see the current scan. The A* lesson escapes the U.
The controller uses the TRUE state here; the obstacles are seen only through the sensor.
"""

import numpy as np
import matplotlib.pyplot as plt

from dronegnc.vehicle.quadrotor import Quadrotor, QuadrotorParams, QuadrotorState
from dronegnc.sensors.obstacle_distance_sensor import ObstacleDistanceSensor
from dronegnc.guidance.potential_field import PotentialField
from dronegnc.guidance.waypoint_follower import yaw_towards_target
from dronegnc.control.multicopter_cascade_controller import MulticopterCascadeController
from dronegnc.control.control_allocation import ControlAllocation
from dronegnc.simulation.obstacles import ObstacleMap, pillar_forest, u_shaped_wall
from dronegnc.simulation.history import History
from dronegnc.visualization.top_view import animate_top_view

show_plot = True

ALTITUDE_M = 2.0
START_NE_M = np.array([0.0, 0.0])
GOAL_NE_M = np.array([24.0, 0.0])


def make_forest(rng):
    """
    Fewer pillars than in the VFH lesson
    """

    pillars = pillar_forest(rng, num_pillars=10, north_range_m=(3.0, 21.0), east_range_m=(-6.0, 6.0),
                            radius_range_m=(0.3, 0.8), keep_free_ne_m=[START_NE_M, GOAL_NE_M],
                            free_radius_m=2.0)
    return ObstacleMap(pillars)


def make_u_trap():
    """
    A U-shaped wall opening towards the start, between the start and the goal
    """

    return ObstacleMap(u_shaped_wall(back_north_m=14.0, depth_m=6.0, half_width_m=4.0))


def fly(obstacle_map, duration_s, rng):
    """
    Fly from START_NE_M to GOAL_NE_M with the potential field; return the History
    """

    dt_s = 0.005
    params = QuadrotorParams()
    vehicle = Quadrotor(params, QuadrotorState(position_ned_m=np.array([*START_NE_M, -ALTITUDE_M])))
    range_sensor = ObstacleDistanceSensor(obstacle_map, rng=rng)
    potential_field = PotentialField()
    controller = MulticopterCascadeController(params)
    allocation = ControlAllocation(vehicle.rotors, params.moment_coef_m, params.max_rotor_thrust_n)
    history = History()
    velocity_sp_ne_mps = np.zeros(2)
    yaw_sp_rad = 0.0
    scan = None

    for step in range(int(duration_s / dt_s)):
        time_s = step * dt_s
        s = vehicle.state

        # 1. Sense, 2. Guide (on every new scan)
        new_scan = range_sensor.measure(vehicle, time_s)
        if new_scan is not None:
            scan = new_scan
            velocity_sp_ne_mps = potential_field.update(s.position_ned_m[:2], GOAL_NE_M, scan)

        # 3. Control: altitude by position, North/East by velocity, nose along the motion
        yaw_sp_rad = yaw_towards_target(s.position_ned_m + np.array([*velocity_sp_ne_mps, 0.0]) * 2.0,
                                        s.position_ned_m, yaw_sp_rad, np.radians(60.0), dt_s)
        thrust_n, moment_nm = controller.update(
            [np.nan, np.nan, -ALTITUDE_M], yaw_sp_rad, s.quaternion, s.angular_velocity_body_rps,
            s.position_ned_m, s.velocity_ned_mps, dt_s, velocity_ff_ned_mps=np.array([*velocity_sp_ne_mps, 0.0]))
        vehicle.step(allocation.allocate(thrust_n, moment_nm), dt_s)

        history.record(time_s=time_s + dt_s, true_position_ned_m=s.position_ned_m,
                       velocity_sp_ne_mps=velocity_sp_ne_mps,
                       clearance_m=obstacle_map.distance_to_nearest_m(s.position_ned_m[:2]),
                       scan_distances_m=scan.distances_m, scan_increment_rad=scan.increment_rad)
    return history


def main():
    rng = np.random.default_rng(seed=3)
    forest = make_forest(rng)
    u_trap = make_u_trap()
    histories = {"forest": fly(forest, 30.0, rng), "u_trap": fly(u_trap, 30.0, rng)}

    if show_plot:
        fig, ax = plt.subplots(figsize=(8, 4))
        for name, history in histories.items():
            distance_m = np.linalg.norm(history["true_position_ned_m"][:, :2] - GOAL_NE_M, axis=1)
            ax.plot(history["time_s"], distance_m, label=name)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Distance to goal [m]")
        ax.grid(True)
        ax.legend()
        fig.suptitle("Potential field: reaches the goal in a sparse forest, stuck in the U")

        forest_fig, forest_animation = animate_top_view(histories["forest"], forest, GOAL_NE_M,
                                                        title="Potential field: forest")
        trap_fig, trap_animation = animate_top_view(histories["u_trap"], u_trap, GOAL_NE_M,
                                                    title="Potential field: U-trap")
        plt.show()

    return histories


if __name__ == "__main__":
    main()
