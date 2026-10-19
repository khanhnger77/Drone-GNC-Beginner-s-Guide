"""
control_allocation.py

Author: Khanh Ngo

Control allocation (PX4: src/modules/control_allocator). The controller asks for a
collective thrust and a 3-axis moment; allocation finds the thrust of each rotor.

The effectiveness matrix B maps rotor thrusts to [Mx, My, Mz, T]:
    [Mx, My, Mz, T]^T = B @ rotor_thrusts
so the allocation is rotor_thrusts = pinv(B) @ [Mx, My, Mz, T]^T.
"""

import numpy as np


class ControlAllocation:
    """
    Pseudo-inverse allocation that gives up yaw first when rotors saturate
    """

    def __init__(self, rotors, moment_coef_m, max_rotor_thrust_n):
        """
        Constructor
        rotors: List of Rotor (position and spin direction)
        moment_coef_m: Yaw reaction moment per unit thrust [N m / N]
        max_rotor_thrust_n: Maximum thrust of one rotor [N]
        """

        self.max_rotor_thrust_n = max_rotor_thrust_n
        self.effectiveness = np.zeros((4, len(rotors)))
        for i, rotor in enumerate(rotors):
            rx, ry, _ = rotor.position_body_m
            # Moment of the force [0, 0, -T] at [rx, ry, 0] is [-ry*T, rx*T, 0]
            self.effectiveness[:, i] = [-ry, rx, rotor.spin_direction * moment_coef_m, 1.0]
        self.mixer = np.linalg.pinv(self.effectiveness)

    def allocate(self, collective_thrust_n, moment_body_nm):
        """
        Rotor thrusts [N] in PX4 motor order
        collective_thrust_n: Desired total thrust [N]
        moment_body_nm: Desired moment in the body frame [N m]
        """

        # Roll, pitch and thrust keep the vehicle in the air; yaw is the least important.
        # Like PX4, when a rotor would saturate we reduce the yaw moment first.
        thrusts_without_yaw = self.mixer @ np.array([*moment_body_nm[:2], 0.0, collective_thrust_n])
        thrusts_for_yaw = self.mixer @ np.array([0.0, 0.0, moment_body_nm[2], 0.0])

        # Largest yaw scale in [0, 1] that keeps every rotor inside [0, max]
        yaw_scale = 1.0
        for base, yaw in zip(thrusts_without_yaw, thrusts_for_yaw):
            if yaw > 0.0:
                yaw_scale = min(yaw_scale, max(0.0, (self.max_rotor_thrust_n - base) / yaw))
            elif yaw < 0.0:
                yaw_scale = min(yaw_scale, max(0.0, -base / yaw))

        thrusts = thrusts_without_yaw + yaw_scale * thrusts_for_yaw
        return np.clip(thrusts, 0.0, self.max_rotor_thrust_n)
