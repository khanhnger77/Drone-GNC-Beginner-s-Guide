"""
planar_waypoint_following.py

Author: Khanh Ngo

Lesson: guidance + control. A waypoint follower gives the next target,
the cascaded controller flies there. The controller uses the TRUE state here.
"""

import numpy as np
import matplotlib.pyplot as plt

from dronegnc.vehicle.planar_quadrotor import PlanarQuadrotor, PlanarQuadrotorParams
from dronegnc.control.planar_cascade_controller import PlanarCascadeController
from dronegnc.guidance.waypoint_follower import WaypointFollower
from dronegnc.visualization.planar_animation import animate_planar
from dronegnc.simulation.history import History

show_plot = True

# [North, Down] in NED: altitude = -Down
WAYPOINTS_M = np.array([[0.0, -3.0], [6.0, -3.0], [6.0, -6.0], [0.0, -6.0], [0.0, -1.0]])


def main():
    dt_s = 0.005
    duration_s = 25.0
    params = PlanarQuadrotorParams()
    vehicle = PlanarQuadrotor(params)
    controller = PlanarCascadeController(params)
    guidance = WaypointFollower(WAYPOINTS_M, acceptance_radius_m=0.3)
    history = History()

    for step in range(int(duration_s / dt_s)):
        s = vehicle.state
        target_m = guidance.update([s.x_m, s.z_m])
        rotor_thrusts_n = controller.update(target_m, s, dt_s)
        vehicle.step(rotor_thrusts_n, dt_s)

        history.record(time_s=(step + 1) * dt_s, true_x_m=s.x_m, true_z_m=s.z_m,
                       true_pitch_rad=s.pitch_rad)

    if show_plot:
        fig, animation = animate_planar(history, params.arm_length_m, WAYPOINTS_M)
        plt.show()

    return history


if __name__ == "__main__":
    main()
