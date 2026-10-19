"""
test_quadrotor_simulations.py

Author: Khanh Ngo
"""

import numpy as np

from dronegnc.common.rotation import quat_to_dcm, wrap_pi


def test_quadrotor_waypoint_following(load_simulation):
    module = load_simulation("quadrotor/quadrotor_waypoint_following.py")
    history = module.main()
    assert np.linalg.norm(history["true_position_ned_m"][-1] - module.WAYPOINTS_NED_M[-1]) < 0.2
    tilt_deg = np.degrees([np.arccos(np.clip(quat_to_dcm(q)[2, 2], -1, 1)) for q in history["true_quaternion"]])
    assert tilt_deg.max() < 40.0


def test_quadrotor_attitude_complementary_filter(load_simulation):
    history = load_simulation("quadrotor/quadrotor_attitude_complementary_filter.py").main()
    true_euler = history["true_euler_rad"]
    filter_error_deg = np.degrees(np.abs(wrap_pi(history["filter_euler_rad"] - true_euler)))
    gyro_only_error_deg = np.degrees(np.abs(wrap_pi(history["gyro_only_euler_rad"] - true_euler)))
    assert filter_error_deg.max() < 15.0
    assert np.all(filter_error_deg[-1] < gyro_only_error_deg[-1])
