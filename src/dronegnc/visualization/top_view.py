"""
top_view.py

Author: Khanh Ngo

Top view (map view) of a flight among obstacles: East to the right, North up,
like a map. Used by the obstacle avoidance lessons.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from dronegnc.simulation.obstacles import CircleObstacle


def draw_obstacles(ax, obstacle_map, color="#59636e"):
    """
    Pillars as filled circles, walls as thick lines
    """

    for obstacle in obstacle_map.obstacles:
        if isinstance(obstacle, CircleObstacle):
            north_m, east_m = obstacle.center_ne_m
            ax.add_patch(plt.Circle((east_m, north_m), obstacle.radius_m, color=color, alpha=0.8))
        else:
            ax.plot([obstacle.start_ne_m[1], obstacle.end_ne_m[1]],
                    [obstacle.start_ne_m[0], obstacle.end_ne_m[0]], "-", color=color, linewidth=4)


def scan_points_ne(position_ne_m, distances_m, increment_rad):
    """
    [North, East] points of the finite readings of an obstacle_distance scan
    """

    angles_rad = np.arange(len(distances_m)) * increment_rad
    hit = np.isfinite(distances_m)
    return (np.asarray(position_ne_m)
            + distances_m[hit, None] * np.column_stack([np.cos(angles_rad[hit]), np.sin(angles_rad[hit])]))


def animate_top_view(history, obstacle_map, goal_ne_m=None, title="", frame_step=20):
    """
    Animate a recorded flight seen from above
    history: History with time_s, true_position_ned_m, scan_distances_m, scan_increment_rad
    obstacle_map: ObstacleMap of the true world
    goal_ne_m: Optional goal [North, East] [m]
    frame_step: Draw every n-th recorded step
    """

    time_s = history["time_s"]
    position_ne = history["true_position_ned_m"][:, :2]
    scans = history["scan_distances_m"]
    increment_rad = history["scan_increment_rad"][0]

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect("equal")
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.grid(True, alpha=0.3)
    draw_obstacles(ax, obstacle_map)
    if goal_ne_m is not None:
        ax.plot(goal_ne_m[1], goal_ne_m[0], "*", color="#2da44e", markersize=16, label="goal")
    (path,) = ax.plot([], [], "-", color="#1f6feb", linewidth=2, label="path")
    (scan_dots,) = ax.plot([], [], ".", color="#e5534b", markersize=4, label="obstacle_distance")
    (drone,) = ax.plot([], [], "o", color="#1f6feb", markersize=8)
    ax.legend(loc="lower right")
    title_text = ax.set_title(title)

    def draw(i):
        path.set_data(position_ne[:i + 1, 1], position_ne[:i + 1, 0])
        points = scan_points_ne(position_ne[i], scans[i], increment_rad)
        scan_dots.set_data(points[:, 1], points[:, 0])
        drone.set_data([position_ne[i, 1]], [position_ne[i, 0]])
        title_text.set_text(f"{title}  t = {time_s[i]:.1f} s")

    frames = range(0, len(time_s), frame_step)
    animation = FuncAnimation(fig, draw, frames=frames, interval=20, repeat=False)
    return fig, animation
