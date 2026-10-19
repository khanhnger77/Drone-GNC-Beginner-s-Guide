"""
test_rotation.py

Author: Khanh Ngo
"""

import numpy as np

from dronegnc.common.rotation import (euler_to_quat, quat_to_euler, quat_to_dcm, dcm_to_quat,
                                      quat_multiply, quat_conjugate, quat_from_rotation_vector,
                                      planar_dcm, wrap_pi)


def test_euler_quaternion_round_trip():
    for euler in [(0.1, -0.2, 0.3), (-1.0, 0.5, 3.0), (0.0, 0.0, -2.5)]:
        assert np.allclose(quat_to_euler(euler_to_quat(*euler)), euler)


def test_dcm_quaternion_round_trip():
    q = euler_to_quat(0.4, -0.7, 2.9)
    q_back = dcm_to_quat(quat_to_dcm(q))
    assert np.allclose(q_back, q) or np.allclose(q_back, -q)


def test_yaw_90_deg_turns_body_forward_to_east():
    R = quat_to_dcm(euler_to_quat(0.0, 0.0, np.pi / 2))
    assert np.allclose(R @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])


def test_quaternion_times_conjugate_is_identity():
    q = euler_to_quat(0.3, 0.2, 0.1)
    assert np.allclose(quat_multiply(q, quat_conjugate(q)), [1.0, 0.0, 0.0, 0.0])


def test_rotation_vector_matches_euler():
    assert np.allclose(quat_from_rotation_vector(np.array([0.0, 0.0, 0.5])), euler_to_quat(0.0, 0.0, 0.5))


def test_planar_dcm_is_3d_pitch_rotation():
    R = quat_to_dcm(euler_to_quat(0.0, 0.3, 0.0))
    assert np.allclose(planar_dcm(0.3), R[np.ix_([0, 2], [0, 2])])


def test_wrap_pi():
    assert np.isclose(wrap_pi(3 * np.pi / 2), -np.pi / 2)
