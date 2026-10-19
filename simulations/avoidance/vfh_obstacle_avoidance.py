"""
vfh_obstacle_avoidance.py

Author: Khanh Ngo

Lesson: reactive obstacle avoidance with a Vector Field Histogram.
The quadrotor holds 2 m altitude and flies to a goal 24 m North through a forest of
pillars. A 360 degree range sensor publishes obstacle_distance (PX4 layout, 72 bins)
at 10 Hz; VFH turns each scan into a horizontal velocity setpoint, which goes to the
PX4-style cascade as a velocity feed-forward (position_sp = NaN on North and East).
The controller uses the TRUE state here; the obstacles are seen only through the sensor.
"""

import numpy as np
import matplotlib.pyplot as plt

from dronegnc.vehicle.quadrotor import Quadrotor, QuadrotorParams, QuadrotorState
from dronegnc.sensors.obstacle_distance_sensor import ObstacleDistanceSensor
from dronegnc.guidance.vector_field_histogram import VectorFieldHistogram
from dronegnc.guidance.waypoint_follower import yaw_towards_target
from dronegnc.control.multicopter_cascade_controller import MulticopterCascadeController
from dronegnc.control.control_allocation import ControlAllocation
from dronegnc.simulation.obstacles import ObstacleMap, pillar_forest
from dronegnc.simulation.history import History
from dronegnc.visualization.top_view import animate_top_view

show_plot = True

ALTITUDE_M = 2.0
START_NE_M = np.array([0.0, 0.0])
GOAL_NE_M = np.array([24.0, 0.0])


def make_world(rng):
    """
    Pillars between the start and the goal
    """

    pillars = pillar_forest(rng, num_pillars=22, north_range_m=(3.0, 21.0), east_range_m=(-6.0, 6.0),
                            radius_range_m=(0.3, 0.8), keep_free_ne_m=[START_NE_M, GOAL_NE_M],
                            free_radius_m=2.0)
    return ObstacleMap(pillars)


def main():
    dt_s = 0.005
    duration_s = 30.0
    rng = np.random.default_rng(seed=1)
    params = QuadrotorParams()

    # Truth: vehicle and world
    vehicle = Quadrotor(params, QuadrotorState(position_ned_m=np.array([*START_NE_M, -ALTITUDE_M])))
    obstacle_map = make_world(rng)
    range_sensor = ObstacleDistanceSensor(obstacle_map, rng=rng)

    # Guidance and control
    vfh = VectorFieldHistogram()
    controller = MulticopterCascadeController(params)
    allocation = ControlAllocation(vehicle.rotors, params.moment_coef_m, params.max_rotor_thrust_n)
    history = History()
    velocity_sp_ne_mps = np.zeros(2)
    yaw_sp_rad = 0.0
    scan = None

    for step in range(int(duration_s / dt_s)):
        time_s = step * dt_s
        s = vehicle.state

        # 1. Sense: a new scan at 10 Hz
        new_scan = range_sensor.measure(vehicle, time_s)
        if new_scan is not None:
            scan = new_scan
            # 2. Guide: VFH runs on every new scan
            velocity_sp_ne_mps = vfh.update(s.position_ned_m[:2], GOAL_NE_M, scan)

        # 3. Control: hold altitude by position, fly North/East by velocity, nose along the motion
        yaw_sp_rad = yaw_towards_target(s.position_ned_m + np.array([*velocity_sp_ne_mps, 0.0]) * 2.0,
                                        s.position_ned_m, yaw_sp_rad, np.radians(60.0), dt_s)
        thrust_n, moment_nm = controller.update(
            [np.nan, np.nan, -ALTITUDE_M], yaw_sp_rad, s.quaternion, s.angular_velocity_body_rps,
            s.position_ned_m, s.velocity_ned_mps, dt_s, velocity_ff_ned_mps=np.array([*velocity_sp_ne_mps, 0.0]))
        vehicle.step(allocation.allocate(thrust_n, moment_nm), dt_s)

        history.record(time_s=time_s + dt_s, true_position_ned_m=s.position_ned_m,
                       velocity_sp_ne_mps=velocity_sp_ne_mps,
                       clearance_m=obstacle_map.distance_to_nearest_m(s.position_ned_m[:2]),
                       scan_distances_m=scan.distances_m, scan_increment_rad=scan.increment_rad,
                       vfh_histogram=vfh.histogram, vfh_direction_rad=vfh.direction_rad)

    if show_plot:
        t = history["time_s"]
        fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
        axes[0].plot(t, history["clearance_m"], label="distance to nearest obstacle")
        axes[0].axhline(0.0, color="k", linestyle="--", label="collision")
        axes[0].set_ylabel("Clearance [m]")
        axes[1].plot(t, np.linalg.norm(history["velocity_sp_ne_mps"], axis=1), label="VFH speed setpoint")
        axes[1].set_ylabel("Speed [m/s]")
        axes[1].set_xlabel("Time [s]")
        for ax in axes:
            ax.grid(True)
            ax.legend(loc="upper right")
        fig.suptitle("VFH obstacle avoidance")

        anim_fig, animation = animate_top_view(history, obstacle_map, GOAL_NE_M, title="VFH")
        plt.show()

    return history


if __name__ == "__main__":
    main()
