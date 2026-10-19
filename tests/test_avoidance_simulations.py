"""
test_avoidance_simulations.py

Author: Khanh Ngo
"""

import numpy as np


def distance_to_goal_m(history, goal_ne_m):
    return np.linalg.norm(history["true_position_ned_m"][-1, :2] - goal_ne_m)


def test_collision_prevention(load_simulation):
    module = load_simulation("avoidance/collision_prevention.py")
    history = module.main()
    position_ne = history["true_position_ned_m"][:, :2]
    # Never closer than CP_DIST (small margin for the controller), slid North, stopped in the corner
    assert history["clearance_m"].min() > 0.9 * module.MIN_DISTANCE_M
    assert position_ne[-1, 0] > 8.0
    assert np.linalg.norm(history["true_velocity_ned_mps"][-1, :2]) < 0.1


def test_vfh_obstacle_avoidance(load_simulation):
    module = load_simulation("avoidance/vfh_obstacle_avoidance.py")
    history = module.main()
    assert distance_to_goal_m(history, module.GOAL_NE_M) < 0.3
    assert history["clearance_m"].min() > 0.5


def test_potential_field_obstacle_avoidance(load_simulation):
    module = load_simulation("avoidance/potential_field_obstacle_avoidance.py")
    histories = module.main()
    # Reaches the goal in the sparse forest...
    assert distance_to_goal_m(histories["forest"], module.GOAL_NE_M) < 0.3
    assert histories["forest"]["clearance_m"].min() > 0.5
    # ...but is trapped inside the U (back wall at 14 m North)
    trapped_ne = histories["u_trap"]["true_position_ned_m"][-1, :2]
    assert 8.0 < trapped_ne[0] < 14.0 and abs(trapped_ne[1]) < 4.0


def test_astar_global_planning(load_simulation):
    module = load_simulation("avoidance/astar_global_planning.py")
    history = module.main()
    assert distance_to_goal_m(history, module.GOAL_NE_M) < 0.3
    assert history["clearance_m"].min() > 0.8
