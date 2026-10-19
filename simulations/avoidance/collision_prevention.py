"""
collision_prevention.py

Author: Khanh Ngo

Lesson: PX4 collision prevention (CP_DIST) in Position mode.
The pilot holds the stick so the drone flies East-North-East at 3.2 m/s, straight
into a wall, then into a corner. Collision prevention does not plan anything: it
brakes the velocity towards each obstacle so the drone stops CP_DIST before it,
keeps the velocity along the wall (the drone slides North), and stops in the corner.
The dashed line is where the stick command alone would have gone.
The controller uses the TRUE state here; the walls are seen only through the sensor.
"""

import numpy as np
import matplotlib.pyplot as plt

from dronegnc.vehicle.quadrotor import Quadrotor, QuadrotorParams, QuadrotorState
from dronegnc.sensors.obstacle_distance_sensor import ObstacleDistanceSensor
from dronegnc.guidance.collision_prevention import CollisionPrevention
from dronegnc.control.multicopter_cascade_controller import MulticopterCascadeController
from dronegnc.control.control_allocation import ControlAllocation
from dronegnc.simulation.obstacles import ObstacleMap, WallObstacle
from dronegnc.simulation.history import History
from dronegnc.visualization.top_view import animate_top_view

show_plot = True

ALTITUDE_M = 2.0
STICK_VELOCITY_NE_MPS = np.array([1.0, 3.0])  # what the pilot asks for: mostly East, a bit North
MIN_DISTANCE_M = 1.5                          # PX4: CP_DIST


def make_world():
    """
    A wall running North-South 8 m East of the start, and a wall closing it 12 m North
    """

    return ObstacleMap([WallObstacle(np.array([-5.0, 8.0]), np.array([12.0, 8.0])),
                        WallObstacle(np.array([12.0, -2.0]), np.array([12.0, 8.0]))])


def main():
    dt_s = 0.005
    duration_s = 15.0
    rng = np.random.default_rng(seed=0)
    params = QuadrotorParams()

    # Truth: vehicle and world
    vehicle = Quadrotor(params, QuadrotorState(position_ned_m=np.array([0.0, 0.0, -ALTITUDE_M])))
    obstacle_map = make_world()
    range_sensor = ObstacleDistanceSensor(obstacle_map, rng=rng)

    # Position mode: stick -> velocity setpoint -> collision prevention -> controller
    collision_prevention = CollisionPrevention(min_distance_m=MIN_DISTANCE_M)
    controller = MulticopterCascadeController(params)
    allocation = ControlAllocation(vehicle.rotors, params.moment_coef_m, params.max_rotor_thrust_n)
    history = History()
    scan = None
    velocity_sp_ne_mps = np.zeros(2)

    for step in range(int(duration_s / dt_s)):
        time_s = step * dt_s
        s = vehicle.state

        # 1. Sense
        new_scan = range_sensor.measure(vehicle, time_s)
        if new_scan is not None:
            scan = new_scan
            # 2. Constrain the pilot's velocity setpoint with the latest scan
            velocity_sp_ne_mps = collision_prevention.modify_setpoint(STICK_VELOCITY_NE_MPS, scan)

        # 3. Control: altitude by position, North/East by velocity, yaw fixed North
        thrust_n, moment_nm = controller.update(
            [np.nan, np.nan, -ALTITUDE_M], 0.0, s.quaternion, s.angular_velocity_body_rps,
            s.position_ned_m, s.velocity_ned_mps, dt_s, velocity_ff_ned_mps=np.array([*velocity_sp_ne_mps, 0.0]))
        vehicle.step(allocation.allocate(thrust_n, moment_nm), dt_s)

        history.record(time_s=time_s + dt_s, true_position_ned_m=s.position_ned_m,
                       true_velocity_ned_mps=s.velocity_ned_mps, velocity_sp_ne_mps=velocity_sp_ne_mps,
                       clearance_m=obstacle_map.distance_to_nearest_m(s.position_ned_m[:2]),
                       scan_distances_m=scan.distances_m, scan_increment_rad=scan.increment_rad)

    if show_plot:
        t = history["time_s"]
        fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
        axes[0].plot(t, history["clearance_m"], label="distance to nearest wall")
        axes[0].axhline(MIN_DISTANCE_M, color="k", linestyle="--", label="CP_DIST")
        axes[0].set_ylabel("Clearance [m]")
        velocity_sp = history["velocity_sp_ne_mps"]
        axes[1].plot(t, velocity_sp[:, 0], label="North setpoint")
        axes[1].plot(t, velocity_sp[:, 1], label="East setpoint")
        axes[1].axhline(STICK_VELOCITY_NE_MPS[1], color="C1", linestyle=":", label="East stick")
        axes[1].set_ylabel("Velocity [m/s]")
        axes[1].set_xlabel("Time [s]")
        for ax in axes:
            ax.grid(True)
            ax.legend(loc="upper right")
        fig.suptitle("Collision prevention: brake towards obstacles, slide along them")

        anim_fig, animation = animate_top_view(history, obstacle_map, title="Collision prevention")
        stick_path = np.outer(t, STICK_VELOCITY_NE_MPS)
        anim_fig.axes[0].plot(stick_path[:, 1], stick_path[:, 0], "k--", alpha=0.4, label="stick only")
        anim_fig.axes[0].legend(loc="lower right")
        plt.show()

    return history


if __name__ == "__main__":
    main()
