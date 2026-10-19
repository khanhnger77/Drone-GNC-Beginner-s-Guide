"""
test_planar_simulations.py

Author: Khanh Ngo
"""

import numpy as np


def test_planar_hover_step_response(load_simulation):
    history = load_simulation("planar/planar_hover_step_response.py").main()
    assert abs(history["true_x_m"][-1] - 3.0) < 0.1
    assert abs(history["true_z_m"][-1] + 2.0) < 0.1


def test_planar_waypoint_following(load_simulation):
    history = load_simulation("planar/planar_waypoint_following.py").main()
    assert abs(history["true_x_m"][-1]) < 0.1
    assert abs(history["true_z_m"][-1] + 1.0) < 0.1


def test_planar_ekf_imu_gps_baro(load_simulation):
    module = load_simulation("planar/planar_ekf_imu_gps_baro.py")
    history = module.main()
    position_error_m = np.hypot(history["est_x_m"] - history["true_x_m"],
                                history["est_z_m"] - history["true_z_m"])
    assert np.sqrt(np.mean(position_error_m ** 2)) < 0.3
    assert abs(history["est_gyro_bias_rps"][-1] - 0.02) < 0.005
