# Lessons

One page per lesson. Each page follows the three layers of the course:

1. **Theory**: the equations, short.
2. **The code**: where each equation lives in this repository, top to bottom.
3. **PX4**: where the same idea lives in PX4 `v1.17.0`, which names map to which, and what PX4 does differently.

Every page also lists what you should see when you run the simulation (numbers measured with this
repository) and a few exercises.

PX4 paths are relative to the PX4 repository root. Run `./tools/fetch_px4.sh` to get the pinned
source in `external/PX4-Autopilot` and open it next to these pages.

| # | Lesson | Simulation | PX4 |
|---|---|---|---|
| 0 | [Frames and rotations](00_frames_and_rotations.md) | (unit tests) | `src/lib/matrix` |
| 1 | [Cascaded PID control (2D)](01_planar_cascaded_pid.md) | `planar/planar_hover_step_response.py` | `mc_pos_control`, `mc_att_control`, `mc_rate_control` |
| 2 | [Waypoint guidance](02_waypoint_guidance.md) | `planar/planar_waypoint_following.py` | `navigator`, `NAV_ACC_RAD` |
| 3 | [EKF in the loop (2D)](03_planar_ekf.md) | `planar/planar_ekf_imu_gps_baro.py` | `ekf2` |
| 4 | [6DOF quadrotor and the PX4 control cascade](04_quadrotor_cascade.md) | `quadrotor/quadrotor_waypoint_following.py` | `ControlMath`, `AttitudeControl`, `control_allocator` |
| 5 | [Attitude complementary filter](05_attitude_complementary_filter.md) | `quadrotor/quadrotor_attitude_complementary_filter.py` | `attitude_estimator_q` |
| 6 | [Collision prevention](06_collision_prevention.md) | `avoidance/collision_prevention.py` | `src/lib/collision_prevention` |
| 7 | [Vector Field Histogram](07_vector_field_histogram.md) | `avoidance/vfh_obstacle_avoidance.py` | `obstacle_distance` (planner runs offboard) |
| 8 | [Potential field and local minima](08_potential_field.md) | `avoidance/potential_field_obstacle_avoidance.py` | offboard |
| 9 | [Global planning with A*](09_astar_global_planning.md) | `avoidance/astar_global_planning.py` | mission, `navigator` |

Recommended order: 0 → 1 → 2 → 4 (control), then 3 → 5 (estimation), then 6 → 9 (avoidance).
Lessons 3 and 5 can be watched before 4 if you prefer estimation first.
