"""
planar_ekf_imu_gps_baro.py

Author: Khanh Ngo

Lesson: EKF in the loop, a 2D miniature of PX4 EKF2.
- IMU (200 Hz) drives the prediction, GPS (10 Hz) and barometer (50 Hz) correct it.
- The gyro has a bias the filter does not know; watch the estimated bias converge.
- The controller now uses the ESTIMATE, not the truth. The truth is only used for plots.
"""

import numpy as np
import matplotlib.pyplot as plt

from dronegnc.vehicle.planar_quadrotor import PlanarQuadrotor, PlanarQuadrotorParams
from dronegnc.sensors.planar_sensors import PlanarImu, PlanarGps, PlanarBarometer
from dronegnc.estimation.planar_ekf import PlanarEkf, X, Z, PITCH, GYRO_BIAS
from dronegnc.control.planar_cascade_controller import PlanarCascadeController
from dronegnc.guidance.waypoint_follower import WaypointFollower
from dronegnc.visualization.planar_animation import animate_planar
from dronegnc.simulation.history import History

show_plot = True

WAYPOINTS_M = np.array([[0.0, -3.0], [6.0, -3.0], [6.0, -6.0], [0.0, -6.0], [0.0, -1.0]])


def main():
    dt_s = 0.005
    duration_s = 30.0
    rng = np.random.default_rng(seed=0)
    params = PlanarQuadrotorParams()

    # Truth (simulator side)
    vehicle = PlanarQuadrotor(params)
    imu = PlanarImu(gyro_bias_rps=0.02, rng=rng)
    gps = PlanarGps(rng=rng)
    baro = PlanarBarometer(rng=rng)

    # Navigation stack: sees only measurements
    ekf = PlanarEkf(gravity_mps2=params.gravity_mps2)
    guidance = WaypointFollower(WAYPOINTS_M, acceptance_radius_m=0.3)
    controller = PlanarCascadeController(params)
    history = History()

    for step in range(int(duration_s / dt_s)):
        time_s = step * dt_s

        # 1. Sense
        imu_sample = imu.measure(vehicle)
        gps_sample = gps.measure(vehicle, time_s)
        altitude_m = baro.measure(vehicle, time_s)

        # 2. Estimate
        ekf.predict(imu_sample, dt_s)
        if gps_sample is not None:
            ekf.fuse_gps(gps_sample)
        if altitude_m is not None:
            ekf.fuse_baro(altitude_m)
        estimate = ekf.estimate()

        # 3. Guide and control using the estimate
        target_m = guidance.update([estimate.x_m, estimate.z_m])
        rotor_thrusts_n = controller.update(target_m, estimate, dt_s)

        # 4. Move the true vehicle
        vehicle.step(rotor_thrusts_n, dt_s)

        s = vehicle.state
        history.record(time_s=time_s + dt_s, true_x_m=s.x_m, true_z_m=s.z_m, true_pitch_rad=s.pitch_rad,
                       est_x_m=ekf.x[X], est_z_m=ekf.x[Z], est_pitch_rad=ekf.x[PITCH],
                       est_gyro_bias_rps=ekf.x[GYRO_BIAS], std=ekf.std())

    if show_plot:
        t = history["time_s"]
        std = history["std"]
        fig, axes = plt.subplots(4, 1, sharex=True, figsize=(8, 9))
        rows = [("x_m", X, 1.0, "North error [m]"), ("z_m", Z, 1.0, "Down error [m]"),
                ("pitch_rad", PITCH, np.degrees(1.0), "Pitch error [deg]")]
        for ax, (name, index, unit, label) in zip(axes, rows):
            error = history[f"est_{name}"] - history[f"true_{name}"]
            ax.plot(t, unit * error, "r-", label="estimate - truth")
            ax.fill_between(t, -3 * unit * std[:, index], 3 * unit * std[:, index],
                            color="gray", alpha=0.3, label="3 sigma")
            ax.set_ylabel(label)
        axes[3].plot(t, np.full_like(t, imu.gyro_bias_rps), "k--", label="true")
        axes[3].plot(t, history["est_gyro_bias_rps"], "r-", label="estimate")
        axes[3].set_ylabel("Gyro bias [rad/s]")
        axes[3].set_xlabel("Time [s]")
        for ax in axes:
            ax.grid(True)
            ax.legend(loc="right")
        fig.suptitle("Planar EKF: estimation error stays inside its 3 sigma bound")

        anim_fig, animation = animate_planar(history, params.arm_length_m, WAYPOINTS_M, estimate_prefix="est")
        plt.show()

    return history


if __name__ == "__main__":
    main()
