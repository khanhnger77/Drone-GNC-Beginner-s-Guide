"""
make_readme_media.py

Author: Khanh Ngo

Runs every simulation (with show_plot = False) and renders the GIFs used in README.md
into doc/images/. Each GIF is drawn only from the History the simulation returns,
like the plots inside the simulations themselves.

Usage (from the repository root, inside the virtual environment):
    python doc/make_readme_media.py            # all GIFs
    python doc/make_readme_media.py ekf hero   # only some of them
"""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from dronegnc.common.rotation import quat_to_dcm, wrap_pi
from dronegnc.estimation.planar_ekf import X, PITCH
from dronegnc.vehicle.planar_quadrotor import PlanarQuadrotorParams
from dronegnc.vehicle.quadrotor import QuadrotorParams
from dronegnc.visualization.planar_animation import planar_body_outline
from dronegnc.visualization.quadrotor_animation import ned_to_plot

ROOT = Path(__file__).absolute().parent.parent
OUTPUT_DIR = ROOT / "doc" / "images"

FPS = 20
DPI = 80

# One palette for every GIF
TRUE_COLOR = "#1f6feb"
ESTIMATE_COLOR = "#e5534b"
SETPOINT_COLOR = "#222222"
GYRO_ONLY_COLOR = "#2da44e"
BAND_COLOR = "#9aa4b2"
AXIS_COLORS = ["#1f6feb", "#e5534b", "#2da44e", "#bf8700"]

plt.rcParams.update({
    "font.size": 10,
    "axes.titlesize": 11,
    "axes.titleweight": "bold",
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "legend.fontsize": 8,
    "legend.framealpha": 0.9,
    "figure.facecolor": "white",
})


def run_simulation(relative_path):
    """
    Import a simulation script, turn off its plots, run main() and return (module, history)
    """

    path = ROOT / "simulations" / relative_path
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.show_plot = False
    print(f"running {relative_path} ...")
    return module, module.main()


def frame_indices(time_s, frame_period_s):
    """
    Indices of the recorded steps to draw, one every frame_period_s of simulated time
    """

    step_s = time_s[1] - time_s[0]
    stride = max(1, int(round(frame_period_s / step_s)))
    indices = list(range(0, len(time_s), stride))
    if indices[-1] != len(time_s) - 1:
        indices.append(len(time_s) - 1)
    return indices


def save_gif(fig, draw, indices, name, hold_last_s=1.5):
    """
    Render draw(i) for each index into doc/images/<name>.gif, then hold the last frame
    """

    frames = indices + [indices[-1]] * int(hold_last_s * FPS)
    animation = FuncAnimation(fig, draw, frames=frames, blit=False)
    path = OUTPUT_DIR / f"{name}.gif"
    animation.save(path, writer=PillowWriter(fps=FPS), dpi=DPI)
    plt.close(fig)
    print(f"  -> {path.relative_to(ROOT)} ({path.stat().st_size / 1e6:.1f} MB, {len(frames)} frames)")


def time_cursor(axes):
    """
    A vertical line on each time plot that follows the animation
    """

    return [ax.axvline(0.0, color="#888888", linewidth=1.0, alpha=0.7) for ax in axes]


def planar_scene(ax, x_m, z_m, margin_m=1.0):
    """
    Common look of the North / altitude view
    """

    ax.set_aspect("equal")
    ax.set_xlabel("North [m]")
    ax.set_ylabel("Altitude [m]")
    ax.set_xlim(x_m.min() - margin_m, x_m.max() + margin_m)
    ax.set_ylim(min(0.0, (-z_m).min()) - 0.5, (-z_m).max() + margin_m)
    ax.axhspan(-10.0, 0.0, color="#d8c9a8", alpha=0.5, zorder=0)  # the ground


def make_hover_step_response():
    _, h = run_simulation("planar/planar_hover_step_response.py")
    t, x, z, pitch = h["time_s"], h["true_x_m"], h["true_z_m"], h["true_pitch_rad"]
    arm_m = PlanarQuadrotorParams().arm_length_m

    fig = plt.figure(figsize=(10, 4.6))
    grid = fig.add_gridspec(2, 2, width_ratios=[1.1, 1.0])
    scene = fig.add_subplot(grid[:, 0])
    north_ax = fig.add_subplot(grid[0, 1])
    pitch_ax = fig.add_subplot(grid[1, 1], sharex=north_ax)

    planar_scene(scene, np.append(x, 3.0), np.append(z, -3.0))
    scene.set_title("Step 3 m North at t = 5 s")
    (setpoint,) = scene.plot([], [], "*", color=SETPOINT_COLOR, markersize=14, label="setpoint")
    (path,) = scene.plot([], [], "-", color=TRUE_COLOR, alpha=0.6, label="path")
    (body,) = scene.plot([], [], "k-", linewidth=4, solid_capstyle="round")
    scene.legend(loc="upper left")

    north_ax.plot(t, h["sp_x_m"], "--", color=SETPOINT_COLOR, label="setpoint")
    (north_line,) = north_ax.plot([], [], color=TRUE_COLOR, label="true")
    north_ax.set_ylabel("North [m]")
    north_ax.set_title("Outer loop: position")
    pitch_ax.plot([], [], "--", color=SETPOINT_COLOR, label="setpoint")
    (pitch_sp_line,) = pitch_ax.plot([], [], "--", color=SETPOINT_COLOR)
    (pitch_line,) = pitch_ax.plot([], [], color=TRUE_COLOR, label="true")
    pitch_ax.set_ylabel("Pitch [deg]")
    pitch_ax.set_xlabel("Time [s]")
    pitch_ax.set_title("Inner loop: attitude tilts first")
    pitch_deg, pitch_sp_deg = np.degrees(pitch), np.degrees(h["sp_pitch_rad"])
    pitch_ax.set_ylim(min(pitch_deg.min(), pitch_sp_deg.min()) - 3, max(pitch_deg.max(), pitch_sp_deg.max()) + 3)
    north_ax.set_xlim(0.0, t[-1])
    north_ax.set_ylim(-0.5, 3.8)
    for ax in (north_ax, pitch_ax):
        ax.legend(loc="upper left")
    cursors = time_cursor([north_ax, pitch_ax])
    fig.suptitle("Cascaded PID: position → velocity → pitch → pitch rate → rotors", fontweight="bold")
    fig.tight_layout()

    def draw(i):
        setpoint.set_data([h["sp_x_m"][i]], [-h["sp_z_m"][i]])
        path.set_data(x[:i + 1], -z[:i + 1])
        body.set_data(*planar_body_outline(x[i], z[i], pitch[i], 3.0 * arm_m))
        north_line.set_data(t[:i + 1], x[:i + 1])
        pitch_line.set_data(t[:i + 1], pitch_deg[:i + 1])
        pitch_sp_line.set_data(t[:i + 1], pitch_sp_deg[:i + 1])
        for cursor in cursors:
            cursor.set_xdata([t[i], t[i]])

    save_gif(fig, draw, frame_indices(t, 0.08), "planar_hover_step_response")


def make_planar_waypoint_following():
    module, h = run_simulation("planar/planar_waypoint_following.py")
    t, x, z, pitch = h["time_s"], h["true_x_m"], h["true_z_m"], h["true_pitch_rad"]
    waypoints_m = module.WAYPOINTS_M
    arm_m = PlanarQuadrotorParams().arm_length_m

    fig, ax = plt.subplots(figsize=(8, 5.2))
    planar_scene(ax, np.append(x, waypoints_m[:, 0]), np.append(z, waypoints_m[:, 1]), margin_m=1.5)
    ax.plot(waypoints_m[:, 0], -waypoints_m[:, 1], "--", color=SETPOINT_COLOR, alpha=0.35)
    for k, (wx, wz) in enumerate(waypoints_m):
        ax.add_patch(plt.Circle((wx, -wz), 0.3, color=SETPOINT_COLOR, alpha=0.12))
        ax.annotate(f"WP{k + 1}", (wx, -wz), xytext=(6, 6), textcoords="offset points", fontsize=9)
    ax.plot([], [], "o", color=SETPOINT_COLOR, alpha=0.3, label="waypoint + acceptance radius")
    (path,) = ax.plot([], [], "-", color=TRUE_COLOR, linewidth=2, label="path")
    (body,) = ax.plot([], [], "k-", linewidth=4, solid_capstyle="round")
    clock = ax.text(0.98, 0.04, "", transform=ax.transAxes, ha="right", family="monospace")
    ax.legend(loc="upper right")
    ax.set_title("Planar quadrotor: waypoint following")
    fig.tight_layout()

    def draw(i):
        path.set_data(x[:i + 1], -z[:i + 1])
        body.set_data(*planar_body_outline(x[i], z[i], pitch[i], 3.0 * arm_m))
        clock.set_text(f"t = {t[i]:5.1f} s")

    save_gif(fig, draw, frame_indices(t, 0.12), "planar_waypoint_following")


def make_planar_ekf():
    module, h = run_simulation("planar/planar_ekf_imu_gps_baro.py")
    t, x, z, pitch = h["time_s"], h["true_x_m"], h["true_z_m"], h["true_pitch_rad"]
    std = h["std"]
    waypoints_m = module.WAYPOINTS_M
    arm_m = PlanarQuadrotorParams().arm_length_m
    true_bias_rps = 0.02  # same as in the simulation's main()

    fig = plt.figure(figsize=(10.5, 5.0))
    grid = fig.add_gridspec(3, 2, width_ratios=[1.15, 1.0])
    scene = fig.add_subplot(grid[:, 0])
    north_ax = fig.add_subplot(grid[0, 1])
    pitch_ax = fig.add_subplot(grid[1, 1], sharex=north_ax)
    bias_ax = fig.add_subplot(grid[2, 1], sharex=north_ax)

    planar_scene(scene, np.append(x, waypoints_m[:, 0]), np.append(z, waypoints_m[:, 1]))
    scene.plot(waypoints_m[:, 0], -waypoints_m[:, 1], "x--", color=SETPOINT_COLOR, alpha=0.35, label="waypoints")
    (true_path,) = scene.plot([], [], "-", color=TRUE_COLOR, linewidth=2, label="truth")
    (est_path,) = scene.plot([], [], "--", color=ESTIMATE_COLOR, linewidth=1.5, label="EKF estimate")
    (body,) = scene.plot([], [], "k-", linewidth=4, solid_capstyle="round")
    scene.legend(loc="upper right")
    scene.set_title("Controller flies on the estimate")

    rows = [(north_ax, h["est_x_m"] - x, std[:, X], "North err [m]"),
            (pitch_ax, np.degrees(h["est_pitch_rad"] - pitch), np.degrees(std[:, PITCH]), "Pitch err [deg]")]
    error_lines = []
    for ax, error, sigma, label in rows:
        ax.fill_between(t, -3 * sigma, 3 * sigma, color=BAND_COLOR, alpha=0.35, label="±3σ")
        (line,) = ax.plot([], [], color=ESTIMATE_COLOR, linewidth=1, label="estimate − truth")
        error_lines.append((line, error))
        limit = 1.2 * max(np.abs(error).max(), 3 * sigma[len(t) // 10:].max())
        ax.set_ylim(-limit, limit)
        ax.set_ylabel(label)
    north_ax.set_title("Error stays inside the EKF's own 3σ bound")
    north_ax.legend(loc="upper right", ncol=2)
    bias_ax.axhline(true_bias_rps, linestyle="--", color=SETPOINT_COLOR, label="true")
    (bias_line,) = bias_ax.plot([], [], color=ESTIMATE_COLOR, label="estimated")
    bias_ax.set_ylim(-0.005, 0.03)
    bias_ax.set_ylabel("Gyro bias [rad/s]")
    bias_ax.set_xlabel("Time [s]")
    bias_ax.legend(loc="lower right", ncol=2)
    north_ax.set_xlim(0.0, t[-1])
    for ax in (north_ax, pitch_ax):
        plt.setp(ax.get_xticklabels(), visible=False)
    cursors = time_cursor([north_ax, pitch_ax, bias_ax])
    fig.suptitle("2D EKF: IMU predicts (200 Hz), GPS (10 Hz) and baro (50 Hz) correct", fontweight="bold")
    fig.tight_layout()

    def draw(i):
        true_path.set_data(x[:i + 1], -z[:i + 1])
        est_path.set_data(h["est_x_m"][:i + 1], -h["est_z_m"][:i + 1])
        body.set_data(*planar_body_outline(x[i], z[i], pitch[i], 3.0 * arm_m))
        for line, error in error_lines:
            line.set_data(t[:i + 1], error[:i + 1])
        bias_line.set_data(t[:i + 1], h["est_gyro_bias_rps"][:i + 1])
        for cursor in cursors:
            cursor.set_xdata([t[i], t[i]])

    save_gif(fig, draw, frame_indices(t, 0.15), "planar_ekf_imu_gps_baro")


def quadrotor_drawing(ax, rotors, scale):
    """
    Artists for a quadrotor X: two arms, a nose and one disc per rotor (color = spin direction)
    """

    arms = [ax.plot([], [], [], "-", color="#333333", linewidth=3)[0] for _ in range(2)]
    nose = ax.plot([], [], [], "-", color=ESTIMATE_COLOR, linewidth=3)[0]
    discs = [ax.plot([], [], [], "-", linewidth=2,
                     color=TRUE_COLOR if r.spin_direction > 0 else GYRO_ONLY_COLOR)[0] for r in rotors]
    rotor_body = np.array([scale * r.position_body_m for r in rotors])
    disc_radius_m = 0.45 * np.linalg.norm(rotor_body[0])
    angles = np.linspace(0.0, 2.0 * np.pi, 24)
    circle = disc_radius_m * np.column_stack([np.cos(angles), np.sin(angles), np.zeros_like(angles)])
    nose_body = np.array([[0.0, 0.0, 0.0], [1.2 * np.linalg.norm(rotor_body[0]), 0.0, 0.0]])

    def update(position_ned_m, quaternion):
        R = quat_to_dcm(quaternion)

        def to_plot(points_body):
            return ned_to_plot((R @ points_body.T).T + position_ned_m)

        rotors_plot = to_plot(rotor_body)
        for arm, (a, b) in zip(arms, [(0, 1), (2, 3)]):
            arm.set_data_3d(rotors_plot[[a, b], 0], rotors_plot[[a, b], 1], rotors_plot[[a, b], 2])
        n = to_plot(nose_body)
        nose.set_data_3d(n[:, 0], n[:, 1], n[:, 2])
        for disc, center_body in zip(discs, rotor_body):
            c = to_plot(circle + center_body)
            disc.set_data_3d(c[:, 0], c[:, 1], c[:, 2])

    return update


def make_quadrotor_waypoint_following():
    module, h = run_simulation("quadrotor/quadrotor_waypoint_following.py")
    t = h["time_s"]
    positions, quaternions = h["true_position_ned_m"], h["true_quaternion"]
    euler_deg = np.degrees(np.unwrap(h["true_euler_rad"], axis=0))  # no ±180° jumps in yaw
    thrusts = h["rotor_thrusts_n"]
    params = QuadrotorParams()
    rotors = params.rotors()
    path = ned_to_plot(positions)
    waypoints = ned_to_plot(module.WAYPOINTS_NED_M)

    fig = plt.figure(figsize=(11, 5.4))
    grid = fig.add_gridspec(2, 2, width_ratios=[1.25, 1.0])
    scene = fig.add_subplot(grid[:, 0], projection="3d")
    attitude_ax = fig.add_subplot(grid[0, 1])
    thrust_ax = fig.add_subplot(grid[1, 1], sharex=attitude_ax)

    scene.set_xlabel("East [m]")
    scene.set_ylabel("North [m]")
    scene.set_zlabel("Altitude [m]")
    scene.set_xlim(-1.5, 9.5)
    scene.set_ylim(-1.5, 9.5)
    scene.set_zlim(0.0, 6.5)
    scene.set_box_aspect((1.0, 1.0, 0.6))
    scene.plot(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], "x--", color=SETPOINT_COLOR,
               alpha=0.4, label="waypoints")
    (trail,) = scene.plot([], [], [], "-", color=TRUE_COLOR, linewidth=1.5, label="path")
    (shadow,) = scene.plot([], [], [], "-", color="#999999", linewidth=1, alpha=0.5)
    draw_vehicle = quadrotor_drawing(scene, rotors, scale=3.0)
    scene.legend(loc="upper left")
    clock = scene.text2D(0.02, 0.02, "", transform=scene.transAxes, family="monospace")

    lines = []
    for k, label in enumerate(["roll", "pitch", "yaw"]):
        lines.append((attitude_ax.plot([], [], color=AXIS_COLORS[k], label=label)[0], euler_deg[:, k]))
    attitude_ax.set_ylim(euler_deg.min() - 10, euler_deg.max() + 10)
    attitude_ax.set_ylabel("Attitude [deg]")
    attitude_ax.set_title("Yaw rate-limited to 60°/s (MPC_YAWRAUTO_MAX)")
    attitude_ax.legend(loc="upper left", ncol=3)
    for k in range(4):
        lines.append((thrust_ax.plot([], [], color=AXIS_COLORS[k], linewidth=1, label=f"motor {k + 1}")[0],
                      thrusts[:, k]))
    thrust_ax.axhline(params.max_rotor_thrust_n, color=SETPOINT_COLOR, linestyle=":", label="max")
    thrust_ax.set_ylim(0.0, params.max_rotor_thrust_n * 1.1)
    thrust_ax.set_ylabel("Rotor thrust [N]")
    thrust_ax.set_xlabel("Time [s]")
    thrust_ax.set_title("Control allocation (PX4 motor order)")
    thrust_ax.legend(loc="lower right", ncol=5, fontsize=7)
    attitude_ax.set_xlim(0.0, t[-1])
    cursors = time_cursor([attitude_ax, thrust_ax])
    fig.suptitle("6DOF quadrotor X: PX4-style control cascade", fontweight="bold")
    fig.tight_layout()

    def draw(i):
        trail.set_data_3d(path[:i + 1, 0], path[:i + 1, 1], path[:i + 1, 2])
        shadow.set_data_3d(path[:i + 1, 0], path[:i + 1, 1], np.zeros(i + 1))
        draw_vehicle(positions[i], quaternions[i])
        scene.view_init(elev=22, azim=-60 + 12 * np.sin(2 * np.pi * t[i] / t[-1]))
        clock.set_text(f"t = {t[i]:5.1f} s")
        for line, values in lines:
            line.set_data(t[:i + 1], values[:i + 1])
        for cursor in cursors:
            cursor.set_xdata([t[i], t[i]])

    # Stop the GIF one second after the vehicle has settled at the last waypoint
    speed_mps = np.linalg.norm(np.diff(positions, axis=0), axis=1) / (t[1] - t[0])
    end = min(len(t) - 1, np.nonzero(speed_mps > 0.05)[0][-1] + int(1.0 / (t[1] - t[0])))
    attitude_ax.set_xlim(0.0, t[end])
    indices = [i for i in frame_indices(t, 0.25) if i <= end]
    save_gif(fig, draw, indices, "quadrotor_waypoint_following")


def make_attitude_complementary_filter():
    _, h = run_simulation("quadrotor/quadrotor_attitude_complementary_filter.py")
    t = h["time_s"]
    true_euler = h["true_euler_rad"]
    true_bias_rps = np.array([0.02, -0.01, 0.015])  # same as in the simulation's main()

    fig, axes = plt.subplots(2, 2, figsize=(10.5, 5.2), sharex=True)
    lines = []
    for ax, k, name in [(axes[0, 0], 0, "Roll"), (axes[0, 1], 1, "Pitch"), (axes[1, 0], 2, "Yaw")]:
        all_errors = []
        for key, color, label in [("gyro_only_euler_rad", GYRO_ONLY_COLOR, "gyro integration only"),
                                  ("filter_euler_rad", ESTIMATE_COLOR, "complementary filter")]:
            error_deg = np.degrees(wrap_pi(h[key][:, k] - true_euler[:, k]))
            lines.append((ax.plot([], [], color=color, label=label)[0], error_deg))
            all_errors.append(error_deg)
        limit = 1.15 * np.abs(np.concatenate(all_errors)).max()
        ax.set_ylim(-limit, limit)
        ax.set_ylabel(f"{name} error [deg]")
    axes[0, 0].legend(loc="upper left")
    axes[0, 0].set_title("Gyro alone drifts with its bias")
    axes[0, 1].set_title("Accelerometer corrects roll / pitch")
    axes[1, 0].set_title("Magnetometer corrects yaw")
    bias = h["est_gyro_bias_rps"]
    for k, axis in enumerate("xyz"):
        axes[1, 1].axhline(true_bias_rps[k], color=AXIS_COLORS[k], linestyle="--", alpha=0.6)
        lines.append((axes[1, 1].plot([], [], color=AXIS_COLORS[k], label=f"estimated {axis}")[0], bias[:, k]))
    axes[1, 1].set_ylim(-0.02, 0.03)
    axes[1, 1].set_ylabel("Gyro bias [rad/s]")
    axes[1, 1].set_title("Bias learned (dashed = true)")
    axes[1, 1].legend(loc="lower right", ncol=3, fontsize=7)
    for ax in axes[1]:
        ax.set_xlabel("Time [s]")
    axes[0, 0].set_xlim(0.0, t[-1])
    cursors = time_cursor(axes.ravel())
    fig.suptitle("Attitude complementary filter (PX4 attitude_estimator_q)", fontweight="bold")
    fig.tight_layout()

    def draw(i):
        for line, values in lines:
            line.set_data(t[:i + 1], values[:i + 1])
        for cursor in cursors:
            cursor.set_xdata([t[i], t[i]])

    save_gif(fig, draw, frame_indices(t, 0.25), "quadrotor_attitude_complementary_filter")


GIFS = {
    "step": make_hover_step_response,
    "waypoints2d": make_planar_waypoint_following,
    "ekf": make_planar_ekf,
    "hero": make_quadrotor_waypoint_following,
    "attitude": make_attitude_complementary_filter,
}


if __name__ == "__main__":
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for name in sys.argv[1:] or GIFS:
        GIFS[name]()
