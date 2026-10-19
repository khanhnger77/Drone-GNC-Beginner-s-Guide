"""
planar_animation.py

Author: Khanh Ngo

Animation of a recorded planar flight. The plot shows North to the right and
altitude (= -z) up, which is easier to read than NED's z-down.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from dronegnc.common.rotation import planar_dcm


def planar_body_outline(x_m, z_m, pitch_rad, arm_length_m):
    """
    Points (north, altitude) of the arm and the two rotor masts, for drawing
    """

    R = planar_dcm(pitch_rad)
    mast_m = 0.3 * arm_length_m
    points_body = np.array([[arm_length_m, -mast_m], [arm_length_m, 0.0],
                            [-arm_length_m, 0.0], [-arm_length_m, -mast_m]])
    points_ned = (R @ points_body.T).T + np.array([x_m, z_m])
    return points_ned[:, 0], -points_ned[:, 1]


def animate_planar(history, arm_length_m, waypoints_m=None, estimate_prefix=None,
                   frame_step=4, scale=3.0):
    """
    Animate a planar flight from a History
    history: History with time_s, true_x_m, true_z_m, true_pitch_rad
    arm_length_m: Arm length of the vehicle [m]
    waypoints_m: Optional (N, 2) waypoints in NED [m]
    estimate_prefix: Optional prefix ("est") to also draw est_x_m / est_z_m
    frame_step: Draw every n-th recorded step
    scale: Drawing scale of the vehicle, so it stays visible on a large plot
    """

    time_s = history["time_s"]
    x_m, z_m, pitch_rad = history["true_x_m"], history["true_z_m"], history["true_pitch_rad"]

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_aspect("equal")
    ax.set_xlabel("North [m]")
    ax.set_ylabel("Altitude [m]")
    ax.grid(True)
    margin = 1.0
    ax.set_xlim(x_m.min() - margin, x_m.max() + margin)
    ax.set_ylim(min(0.0, (-z_m).min()) - margin, (-z_m).max() + margin)

    if waypoints_m is not None:
        waypoints_m = np.asarray(waypoints_m)
        ax.plot(waypoints_m[:, 0], -waypoints_m[:, 1], "kx--", alpha=0.5, label="waypoints")
    (true_path,) = ax.plot([], [], "b-", label="true path")
    estimate_label = "estimated path" if estimate_prefix is not None else None
    (estimate_path,) = ax.plot([], [], "r--", label=estimate_label)
    (body,) = ax.plot([], [], "k-", linewidth=3)
    title = ax.set_title("")
    ax.legend(loc="upper left")

    def draw(i):
        true_path.set_data(x_m[:i + 1], -z_m[:i + 1])
        if estimate_prefix is not None:
            estimate_path.set_data(history[f"{estimate_prefix}_x_m"][:i + 1],
                                   -history[f"{estimate_prefix}_z_m"][:i + 1])
        body.set_data(*planar_body_outline(x_m[i], z_m[i], pitch_rad[i], scale * arm_length_m))
        title.set_text(f"t = {time_s[i]:.2f} s")
        return true_path, estimate_path, body, title

    frames = range(0, len(time_s), frame_step)
    animation = FuncAnimation(fig, draw, frames=frames, interval=20, blit=False, repeat=False)
    return fig, animation
