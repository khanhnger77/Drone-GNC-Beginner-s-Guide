"""
pid.py

Author: Khanh Ngo
"""

import numpy as np


class Pid:
    """
    PID controller for a scalar or a vector (each element is controlled independently).
    The derivative acts on the measurement, not on the error, so a step in the
    setpoint does not cause a spike ("derivative kick"). PX4 rate control does the same.
    """

    def __init__(self, kp, ki=0.0, kd=0.0, integral_limit=np.inf):
        """
        Constructor
        kp, ki, kd: Proportional, integral and derivative gains (scalar or array)
        integral_limit: Limit of the integral term output (anti-windup)
        """

        self.kp = np.asarray(kp, dtype=float)
        self.ki = np.asarray(ki, dtype=float)
        self.kd = np.asarray(kd, dtype=float)
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.last_measurement = None

    def update(self, setpoint, measurement, dt_s):
        """
        Controller output for one time step
        setpoint: Desired value
        measurement: Current value
        dt_s: Time step [s]
        """

        error = np.asarray(setpoint) - np.asarray(measurement)
        self.integral = np.clip(self.integral + self.ki * error * dt_s,
                                -self.integral_limit, self.integral_limit)

        derivative = 0.0
        if self.last_measurement is not None:
            derivative = -(np.asarray(measurement) - self.last_measurement) / dt_s
        self.last_measurement = np.asarray(measurement, dtype=float)

        return self.kp * error + self.integral + self.kd * derivative
