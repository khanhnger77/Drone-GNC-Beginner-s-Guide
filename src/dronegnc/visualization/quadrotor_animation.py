"""
quadrotor_animation.py

Author: Khanh Ngo

3D animation of a recorded quadrotor flight. NED is converted to East-North-Up
only for drawing, because matplotlib's 3D axes expect z up.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from dronegnc.common.rotation import quat_to_dcm


def ned_to_plot(points_ned):
    """
    (N, 3) NED points -> (N, 3) [East, North, Up] for drawing
    """

    points_ned = np.atleast_2d(points_ned)
    return np.column_stack([points_ned[:, 1], points_ned[:, 0], -points_ned[:, 2]])


def animate_quadrotor(history, rotors, waypoints_ned_m=None, frame_step=5, scale=3.0):
    """
    Animate a 3D flight from a History
    history: History with time_s, true_position_ned_m, true_quaternion
    rotors: List of Rotor, used to draw the arms
    waypoints_ned_m: Optional (N, 3) waypoints in NED [m]
    frame_step: Draw every n-th recorded step
    scale: Drawing scale of the vehicle, so it stays visible on a large plot
    """

    time_s = history["time_s"]
    positions = history["true_position_ned_m"]
    quaternions = history["true_quaternion"]
    path = ned_to_plot(positions)

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(projection="3d")
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_zlabel("Altitude [m]")
    center = 0.5 * (path.max(axis=0) + path.min(axis=0))
    half_range = 0.5 * max(np.ptp(path, axis=0).max(), 2.0) + 1.0
    ax.set_xlim(center[0] - half_range, center[0] + half_range)
    ax.set_ylim(center[1] - half_range, center[1] + half_range)
    ax.set_zlim(max(0.0, center[2] - half_range), center[2] + half_range)

    if waypoints_ned_m is not None:
        w = ned_to_plot(waypoints_ned_m)
        ax.plot(w[:, 0], w[:, 1], w[:, 2], "kx--", alpha=0.5, label="waypoints")
    (path_line,) = ax.plot([], [], [], "b-", label="true path")
    # Arm 1-2 and arm 3-4 (diagonals of the X), plus a short red line for "front"
    (arm_a,) = ax.plot([], [], [], "k-", linewidth=3)
    (arm_b,) = ax.plot([], [], [], "k-", linewidth=3)
    (nose,) = ax.plot([], [], [], "r-", linewidth=3)
    title = ax.set_title("")
    ax.legend(loc="upper left")

    rotor_positions_body = np.array([scale * r.position_body_m for r in rotors])
    nose_body = np.array([[0.0, 0.0, 0.0], [scale * np.linalg.norm(rotors[0].position_body_m), 0.0, 0.0]])

    def draw(i):
        R = quat_to_dcm(quaternions[i])
        rotors_plot = ned_to_plot((R @ rotor_positions_body.T).T + positions[i])
        nose_plot = ned_to_plot((R @ nose_body.T).T + positions[i])
        path_line.set_data_3d(path[:i + 1, 0], path[:i + 1, 1], path[:i + 1, 2])
        arm_a.set_data_3d(rotors_plot[[0, 1], 0], rotors_plot[[0, 1], 1], rotors_plot[[0, 1], 2])
        arm_b.set_data_3d(rotors_plot[[2, 3], 0], rotors_plot[[2, 3], 1], rotors_plot[[2, 3], 2])
        nose.set_data_3d(nose_plot[:, 0], nose_plot[:, 1], nose_plot[:, 2])
        title.set_text(f"t = {time_s[i]:.2f} s")
        return path_line, arm_a, arm_b, nose, title

    frames = range(0, len(time_s), frame_step)
    animation = FuncAnimation(fig, draw, frames=frames, interval=20, blit=False, repeat=False)
    return fig, animation
