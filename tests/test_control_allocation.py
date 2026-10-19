"""
test_control_allocation.py

Author: Khanh Ngo
"""

import numpy as np

from dronegnc.vehicle.quadrotor import Quadrotor, QuadrotorParams
from dronegnc.control.control_allocation import ControlAllocation


def make():
    params = QuadrotorParams()
    vehicle = Quadrotor(params)
    return vehicle, ControlAllocation(vehicle.rotors, params.moment_coef_m, params.max_rotor_thrust_n)


def test_hover_gives_equal_rotor_thrusts():
    vehicle, allocation = make()
    thrusts = allocation.allocate(vehicle.hover_thrust_n(), np.zeros(3))
    assert np.allclose(thrusts, vehicle.hover_thrust_n() / 4)


def test_allocation_reproduces_the_demand():
    vehicle, allocation = make()
    moment = np.array([0.2, -0.1, 0.05])
    thrust, produced_moment = vehicle.forces_and_moments(allocation.allocate(15.0, moment))
    assert np.isclose(thrust, 15.0)
    assert np.allclose(produced_moment, moment)


def test_yaw_is_given_up_first_when_saturated():
    vehicle, allocation = make()
    moment = np.array([0.5, 0.0, 5.0])  # impossible yaw demand
    _, produced_moment = vehicle.forces_and_moments(allocation.allocate(15.0, moment))
    assert np.isclose(produced_moment[0], 0.5)  # roll is kept
    assert produced_moment[2] < 5.0
