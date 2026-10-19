"""
rotation.py

Author: Khanh Ngo

Rotation utilities with the same conventions as PX4:
- World frame: NED (x North, y East, z Down)
- Body frame: FRD (x Forward, y Right, z Down)
- Quaternion: [w, x, y, z], rotates a vector from the body frame to the NED frame
- Euler angles: ZYX order (yaw, then pitch, then roll)
"""

import numpy as np
from math import atan2, asin, cos, sin, sqrt, pi


def wrap_pi(angle_rad):
    """
    Wrap an angle to [-pi, pi)
    angle_rad: Angle [rad]
    """

    return (angle_rad + pi) % (2.0 * pi) - pi


def quat_multiply(p, q):
    """
    Hamilton product p * q
    p, q: Quaternions [w, x, y, z]
    """

    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return np.array([pw * qw - px * qx - py * qy - pz * qz,
                     pw * qx + px * qw + py * qz - pz * qy,
                     pw * qy - px * qz + py * qw + pz * qx,
                     pw * qz + px * qy - py * qx + pz * qw])


def quat_conjugate(q):
    """
    Conjugate (= inverse for a unit quaternion)
    q: Quaternion [w, x, y, z]
    """

    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_normalize(q):
    """
    Scale a quaternion to unit length
    q: Quaternion [w, x, y, z]
    """

    return q / np.linalg.norm(q)


def quat_from_rotation_vector(rot_vec_rad):
    """
    Quaternion of a rotation by |rot_vec| around the axis rot_vec/|rot_vec|
    rot_vec_rad: Rotation vector [rad], e.g. angular velocity * dt
    """

    angle_rad = np.linalg.norm(rot_vec_rad)
    if angle_rad < 1e-9:
        # First order approximation avoids division by zero
        return quat_normalize(np.array([1.0, *(0.5 * rot_vec_rad)]))
    axis = rot_vec_rad / angle_rad
    return np.array([cos(angle_rad / 2.0), *(sin(angle_rad / 2.0) * axis)])


def quat_to_dcm(q):
    """
    Direction cosine matrix R (body -> NED), so that v_ned = R @ v_body
    q: Quaternion [w, x, y, z]
    """

    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def dcm_to_quat(R):
    """
    Quaternion from a direction cosine matrix (body -> NED)
    R: 3x3 rotation matrix
    """

    trace = np.trace(R)
    if trace > 0.0:
        s = 2.0 * sqrt(trace + 1.0)
        q = np.array([0.25 * s,
                      (R[2, 1] - R[1, 2]) / s,
                      (R[0, 2] - R[2, 0]) / s,
                      (R[1, 0] - R[0, 1]) / s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = np.array([(R[2, 1] - R[1, 2]) / s,
                      0.25 * s,
                      (R[0, 1] + R[1, 0]) / s,
                      (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = np.array([(R[0, 2] - R[2, 0]) / s,
                      (R[0, 1] + R[1, 0]) / s,
                      0.25 * s,
                      (R[1, 2] + R[2, 1]) / s])
    else:
        s = 2.0 * sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = np.array([(R[1, 0] - R[0, 1]) / s,
                      (R[0, 2] + R[2, 0]) / s,
                      (R[1, 2] + R[2, 1]) / s,
                      0.25 * s])
    return quat_normalize(q)


def euler_to_quat(roll_rad, pitch_rad, yaw_rad):
    """
    Quaternion from ZYX Euler angles
    roll_rad, pitch_rad, yaw_rad: Euler angles [rad]
    """

    cr, sr = cos(roll_rad / 2.0), sin(roll_rad / 2.0)
    cp, sp = cos(pitch_rad / 2.0), sin(pitch_rad / 2.0)
    cy, sy = cos(yaw_rad / 2.0), sin(yaw_rad / 2.0)
    return np.array([cr * cp * cy + sr * sp * sy,
                     sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy])


def quat_to_euler(q):
    """
    ZYX Euler angles (roll, pitch, yaw) [rad] from a quaternion
    q: Quaternion [w, x, y, z]
    """

    w, x, y, z = q
    roll_rad = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch_rad = asin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw_rad = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll_rad, pitch_rad, yaw_rad])


def planar_dcm(pitch_rad):
    """
    2D rotation matrix (body -> NED) in the North-Down plane, so that
    [x_ned, z_ned] = R @ [x_body, z_body]. Positive pitch = nose up.
    pitch_rad: Pitch angle [rad]
    """

    return np.array([[cos(pitch_rad), sin(pitch_rad)],
                     [-sin(pitch_rad), cos(pitch_rad)]])
