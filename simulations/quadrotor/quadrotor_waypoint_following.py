"""
quadrotor_waypoint_following.py

Author: Khanh Ngo

Lesson: the PX4-style multicopter control cascade in 3D.
position -> velocity -> thrust vector -> attitude -> body rates -> moment -> rotor thrusts.
The vehicle yaws to face the next waypoint. The controller uses the TRUE state here.
"""

import numpy as np
import matplotlib.pyplot as plt

from dronegnc.vehicle.quadrotor import Quadrotor, QuadrotorParams
from dronegnc.control.multicopter_cascade_controller import MulticopterCascadeController
from dronegnc.control.control_allocation import ControlAllocation
from dronegnc.guidance.waypoint_follower import WaypointFollower, yaw_towards_target
from dronegnc.visualization.quadrotor_animation import animate_quadrotor
from dronegnc.common.rotation import quat_to_euler
from dronegnc.simulation.history import History

show_plot = True

# [North, East, Down] in NED: altitude = -Down
WAYPOINTS_NED_M = np.array([[0.0, 0.0, -3.0], [8.0, 0.0, -3.0], [8.0, 8.0, -5.0],
                            [0.0, 8.0, -5.0], [0.0, 0.0, -3.0], [0.0, 0.0, -1.0]])


def main():
    dt_s = 0.005
    duration_s = 40.0
    params = QuadrotorParams()
    vehicle = Quadrotor(params)
    controller = MulticopterCascadeController(params)
    allocation = ControlAllocation(vehicle.rotors, params.moment_coef_m, params.max_rotor_thrust_n)
    guidance = WaypointFollower(WAYPOINTS_NED_M, acceptance_radius_m=0.5)
    history = History()
    yaw_sp_rad = 0.0

    for step in range(int(duration_s / dt_s)):
        s = vehicle.state
        target_ned_m = guidance.update(s.position_ned_m)
        yaw_sp_rad = yaw_towards_target(target_ned_m, s.position_ned_m, yaw_sp_rad,
                                        np.radians(60.0), dt_s)  # PX4 default MPC_YAWRAUTO_MAX

        collective_thrust_n, moment_body_nm = controller.update(
            target_ned_m, yaw_sp_rad, s.quaternion, s.angular_velocity_body_rps,
            s.position_ned_m, s.velocity_ned_mps, dt_s)
        rotor_thrusts_n = allocation.allocate(collective_thrust_n, moment_body_nm)
        vehicle.step(rotor_thrusts_n, dt_s)

        history.record(time_s=(step + 1) * dt_s, true_position_ned_m=s.position_ned_m,
                       true_quaternion=s.quaternion, true_euler_rad=quat_to_euler(s.quaternion),
                       rotor_thrusts_n=rotor_thrusts_n)

    if show_plot:
        t = history["time_s"]
        fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 8))
        position = history["true_position_ned_m"]
        for i, label in enumerate(["North", "East", "Down"]):
            axes[0].plot(t, position[:, i], label=label)
        axes[0].set_ylabel("Position [m]")
        euler = np.degrees(history["true_euler_rad"])
        for i, label in enumerate(["roll", "pitch", "yaw"]):
            axes[1].plot(t, euler[:, i], label=label)
        axes[1].set_ylabel("Attitude [deg]")
        thrusts = history["rotor_thrusts_n"]
        for i in range(4):
            axes[2].plot(t, thrusts[:, i], label=f"motor {i + 1}")
        axes[2].set_ylabel("Rotor thrust [N]")
        axes[2].set_xlabel("Time [s]")
        for ax in axes:
            ax.grid(True)
            ax.legend(loc="right")
        fig.suptitle("Quadrotor: waypoint following with the cascaded controller")

        anim_fig, animation = animate_quadrotor(history, vehicle.rotors, WAYPOINTS_NED_M)
        plt.show()

    return history


if __name__ == "__main__":
    main()
