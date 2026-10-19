"""
planar_hover_step_response.py

Author: Khanh Ngo

Lesson: cascaded PID control of a planar quadrotor.
The vehicle takes off to 2 m, then the position setpoint jumps 3 m North.
Watch how the pitch tilts first (inner loops) before the vehicle moves (outer loop).
The controller uses the TRUE state here ("perfect estimation").
"""

import numpy as np
import matplotlib.pyplot as plt

from dronegnc.vehicle.planar_quadrotor import PlanarQuadrotor, PlanarQuadrotorParams
from dronegnc.control.planar_cascade_controller import PlanarCascadeController
from dronegnc.simulation.history import History

show_plot = True


def main():
    dt_s = 0.005
    duration_s = 12.0
    params = PlanarQuadrotorParams()
    vehicle = PlanarQuadrotor(params)
    controller = PlanarCascadeController(params)
    history = History()

    for step in range(int(duration_s / dt_s)):
        time_s = step * dt_s
        position_sp_m = np.array([0.0, -2.0]) if time_s < 5.0 else np.array([3.0, -2.0])

        rotor_thrusts_n = controller.update(position_sp_m, vehicle.state, dt_s)
        vehicle.step(rotor_thrusts_n, dt_s)

        s = vehicle.state
        history.record(time_s=time_s + dt_s, sp_x_m=position_sp_m[0], sp_z_m=position_sp_m[1],
                       true_x_m=s.x_m, true_z_m=s.z_m, true_pitch_rad=s.pitch_rad,
                       sp_pitch_rad=controller.setpoints.pitch_rad, rotor_thrusts_n=rotor_thrusts_n)

    if show_plot:
        t = history["time_s"]
        fig, axes = plt.subplots(4, 1, sharex=True, figsize=(8, 9))
        axes[0].plot(t, history["sp_x_m"], "k--", label="setpoint")
        axes[0].plot(t, history["true_x_m"], "b-", label="true")
        axes[0].set_ylabel("North [m]")
        axes[1].plot(t, -history["sp_z_m"], "k--", label="setpoint")
        axes[1].plot(t, -history["true_z_m"], "b-", label="true")
        axes[1].set_ylabel("Altitude [m]")
        axes[2].plot(t, np.degrees(history["sp_pitch_rad"]), "k--", label="setpoint")
        axes[2].plot(t, np.degrees(history["true_pitch_rad"]), "b-", label="true")
        axes[2].set_ylabel("Pitch [deg]")
        axes[3].plot(t, history["rotor_thrusts_n"][:, 0], label="front")
        axes[3].plot(t, history["rotor_thrusts_n"][:, 1], label="rear")
        axes[3].set_ylabel("Rotor thrust [N]")
        axes[3].set_xlabel("Time [s]")
        for ax in axes:
            ax.grid(True)
            ax.legend(loc="right")
        fig.suptitle("Planar quadrotor: step response of the cascaded controller")
        plt.show()

    return history


if __name__ == "__main__":
    main()
