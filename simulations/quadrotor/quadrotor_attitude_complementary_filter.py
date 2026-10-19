"""
quadrotor_attitude_complementary_filter.py

Author: Khanh Ngo

Lesson: attitude estimation with a complementary filter (PX4 attitude_estimator_q).
The quadrotor flies the waypoint mission with the TRUE state, while the filter
runs alongside on IMU + magnetometer data. Compare:
- pure gyro integration (drifts because of the gyro bias)
- the complementary filter (stays close to the truth and learns the bias)
"""

import numpy as np
import matplotlib.pyplot as plt

from dronegnc.vehicle.quadrotor import Quadrotor, QuadrotorParams
from dronegnc.sensors.quadrotor_sensors import Imu, Magnetometer
from dronegnc.estimation.attitude_complementary_filter import AttitudeComplementaryFilter
from dronegnc.control.multicopter_cascade_controller import MulticopterCascadeController
from dronegnc.control.control_allocation import ControlAllocation
from dronegnc.guidance.waypoint_follower import WaypointFollower, yaw_towards_target
from dronegnc.common.rotation import (quat_to_euler, quat_multiply, quat_normalize,
                                      quat_from_rotation_vector, wrap_pi)
from dronegnc.simulation.history import History

show_plot = True

WAYPOINTS_NED_M = np.array([[0.0, 0.0, -3.0], [8.0, 0.0, -3.0], [8.0, 8.0, -5.0],
                            [0.0, 8.0, -5.0], [0.0, 0.0, -3.0]])


def main():
    dt_s = 0.005
    duration_s = 40.0
    rng = np.random.default_rng(seed=0)
    params = QuadrotorParams()

    # Truth and sensors
    vehicle = Quadrotor(params)
    imu = Imu(gyro_bias_rps=np.array([0.02, -0.01, 0.015]), rng=rng)
    mag = Magnetometer(rng=rng)

    # Flight control (uses the truth in this lesson) and the estimators under test
    controller = MulticopterCascadeController(params)
    allocation = ControlAllocation(vehicle.rotors, params.moment_coef_m, params.max_rotor_thrust_n)
    guidance = WaypointFollower(WAYPOINTS_NED_M, acceptance_radius_m=0.5)
    attitude_filter = AttitudeComplementaryFilter()
    gyro_only_quaternion = vehicle.state.quaternion.copy()
    history = History()
    yaw_sp_rad = 0.0

    for step in range(int(duration_s / dt_s)):
        time_s = step * dt_s
        s = vehicle.state

        # Estimators
        imu_sample = imu.measure(vehicle)
        mag_sample = mag.measure(vehicle, time_s)
        attitude_filter.update(imu_sample, mag_sample, dt_s)
        gyro_only_quaternion = quat_normalize(quat_multiply(
            gyro_only_quaternion, quat_from_rotation_vector(imu_sample.gyro_rps * dt_s)))

        # Flight control with the truth
        target_ned_m = guidance.update(s.position_ned_m)
        yaw_sp_rad = yaw_towards_target(target_ned_m, s.position_ned_m, yaw_sp_rad,
                                        np.radians(60.0), dt_s)  # PX4 default MPC_YAWRAUTO_MAX
        thrust_n, moment_nm = controller.update(target_ned_m, yaw_sp_rad, s.quaternion,
                                                s.angular_velocity_body_rps, s.position_ned_m,
                                                s.velocity_ned_mps, dt_s)
        vehicle.step(allocation.allocate(thrust_n, moment_nm), dt_s)

        if attitude_filter.quaternion is not None:
            history.record(time_s=time_s + dt_s,
                           true_euler_rad=quat_to_euler(vehicle.state.quaternion),
                           filter_euler_rad=quat_to_euler(attitude_filter.quaternion),
                           gyro_only_euler_rad=quat_to_euler(gyro_only_quaternion),
                           est_gyro_bias_rps=attitude_filter.gyro_bias_rps)

    if show_plot:
        t = history["time_s"]
        true_euler = history["true_euler_rad"]
        fig, axes = plt.subplots(4, 1, sharex=True, figsize=(8, 9))
        for i, name in enumerate(["Roll", "Pitch", "Yaw"]):
            for key, style, label in [("gyro_only_euler_rad", "g-", "gyro only"),
                                      ("filter_euler_rad", "r-", "complementary filter")]:
                error_deg = np.degrees(wrap_pi(history[key][:, i] - true_euler[:, i]))
                axes[i].plot(t, error_deg, style, label=label)
            axes[i].set_ylabel(f"{name} error [deg]")
        bias = history["est_gyro_bias_rps"]
        for i, axis in enumerate("xyz"):
            axes[3].plot(t, bias[:, i], label=f"estimated {axis}")
            axes[3].axhline(imu.gyro_bias_rps[i], color=f"C{i}", linestyle="--")
        axes[3].set_ylabel("Gyro bias [rad/s]")
        axes[3].set_xlabel("Time [s]")
        for ax in axes:
            ax.grid(True)
            ax.legend(loc="right")
        fig.suptitle("Attitude estimation: gyro integration vs complementary filter")
        plt.show()

    return history


if __name__ == "__main__":
    main()
