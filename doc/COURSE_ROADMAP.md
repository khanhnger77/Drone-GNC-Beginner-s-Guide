# Course Roadmap

Plan of the video course. Every lesson has three layers:

1. **Theory**: equations and diagrams.
2. **Minimal Python** in this repository: the algorithm alone, readable top to bottom.
3. **PX4**: where the same idea lives in PX4, a variable mapping table, and a SITL run or flight log
   (ulog) viewed with PlotJuggler or Flight Review.

The course is pinned to **PX4 `v1.17.0`**. Link to files at that tag: EKF2 in particular changes a
lot between releases. Run `tools/fetch_px4.sh` to clone it into `external/PX4-Autopilot` (git-ignored)
and read it next to the lessons. Paths are relative to the PX4 repository root.

The paths, uORB topics and parameters of the ✅ lessons were checked against `v1.17.0`. Those of the ⬜
lessons exist at that tag, but their content still has to be read when the lesson is written.

Status: ✅ done, ⬜ planned

## Module 0: Foundations

| | Lesson | Repo | PX4 |
|---|---|---|---|
| ✅ | Frames NED/FRD, DCM, Euler, quaternion | `common/rotation.py` | `src/lib/matrix/matrix/` (`Quaternion`, `Dcm`, `Euler`) |
| ⬜ | Rigid body dynamics, numerical integration | `vehicle/` | `src/modules/simulation/simulator_sih` |

## Module 1: Vehicle and sensors (truth vs estimate)

| | Lesson | Repo | PX4 |
|---|---|---|---|
| ✅ | 2D planar quadrotor | `vehicle/planar_quadrotor.py` | — |
| ✅ | 3D quadrotor 6DOF, rotor layout | `vehicle/quadrotor.py` | `src/modules/simulation/simulator_sih`, `ROMFS/px4fmu_common/init.d/airframes/4001_quad_x` (`CA_ROTOR*`) |
| ✅ | IMU, GPS, baro, mag: noise, bias, rates | `sensors/` | uORB `sensor_combined`, `vehicle_gps_position`, `sensor_baro`, `sensor_mag` |
| ✅ | 360° range sensor with the `obstacle_distance` layout (72 bins of 5°, bin 0 = North) | `sensors/obstacle_distance_sensor.py`, `simulation/obstacles.py` | `msg/ObstacleDistance.msg` |
| ⬜ | Motor dynamics (first-order lag), thrust curve | | `THR_MDL_FAC` (`src/lib/mixer_module/motor_params.c`) |

## Module 2: Control

| | Lesson | Repo | PX4 |
|---|---|---|---|
| ✅ | PID, anti-windup, derivative on measurement | `control/pid.py` | `src/lib/rate_control` |
| ✅ | 2D cascade: position → velocity → pitch → rate | `planar_cascade_controller.py` | — |
| ✅ | 3D: thrust vector → attitude (`bodyzToAttitude`) | `multicopter_cascade_controller.py` | `src/modules/mc_pos_control/PositionControl/ControlMath.cpp` (`bodyzToAttitude`) |
| ✅ | Quaternion attitude control | same | `src/modules/mc_att_control/AttitudeControl/` |
| ✅ | Control allocation, yaw desaturation | `control/control_allocation.py` | `src/modules/control_allocator`, `src/lib/control_allocation/control_allocation/ControlAllocationSequentialDesaturation.cpp` (`mixYaw`) |
| ⬜ | Multi-rate loops: each loop runs when its input topic is published (rate ~400 Hz, attitude ~200 Hz, position ~100 Hz) | | `IMU_GYRO_RATEMAX`, `IMU_INTEG_RATE`, `EKF2_PREDICT_US`, `SubscriptionCallbackWorkItem` in `mc_*_control` |
| ⬜ | Tuning: step response, bandwidth separation | | PID tuning guide, `MC_*` params |
| ⬜ | LQR / MPC for a quadrotor (extension; the vehicle repo has LQR, MPC and MPPI path tracking) | | — |

## Module 3: Navigation (estimation)

The ladder from a textbook EKF to EKF2. Each step adds one idea.

| | Step | New idea | Repo | PX4 |
|---|---|---|---|---|
| ✅ | 1 | 2D EKF with IMU as input, GPS + baro, gyro bias, sequential scalar fusion, test ratio | `estimation/planar_ekf.py` | `src/modules/ekf2` overall structure |
| ✅ | 2 | Complementary attitude filter (Mahony) | `attitude_complementary_filter.py` | `src/modules/attitude_estimator_q` (bias learned only when `spinRate < 0.175` rad/s) |
| ⬜ | 3 | 3D EKF with quaternion attitude | | |
| ⬜ | 4 | Error-state formulation (3D angle error instead of 4D quaternion) | | `src/modules/ekf2/EKF/covariance.cpp` |
| ⬜ | 5 | Accel bias, mag field states (24-state EKF2 layout) | | `src/modules/ekf2/EKF/common.h` (state struct) |
| ⬜ | 6 | Deriving Jacobians symbolically (SymPy) | | `src/modules/ekf2/EKF/python/ekf_derivation/` (SymForce) |
| ⬜ | 7 | Delayed fusion horizon + output predictor, ring buffers | | `src/modules/ekf2/EKF/output_predictor/`, `src/modules/ekf2/EKF/estimator_interface.cpp` |
| ⬜ | 8 | Multiple aiding sources, height source selection | | `src/modules/ekf2/EKF/aid_sources/` |
| ⬜ | 9 | Run the Python EKF on a real ulog and compare with `estimator_states` | | `ekf2` replay |

Side lessons, from the vehicle repo: UKF and particle filter on the same 2D problem, compared with the EKF
(the vehicle repo has `ekf_ukf_pf_comparison`). PX4 uses neither, which is itself worth a short explanation.

Tip: in the videos, explain EKF2's math from the Python derivation scripts, not from the generated C++.

## Module 4: Guidance

The obstacle lessons port ideas from the vehicle repo (AutonomousVehicleControlBeginnersGuide:
`polar_histogram`, `potential_field`, `astar`, `elastic_bands`, `rrt_star`, `dstar`) to a quadrotor that
sees obstacles through the PX4 `obstacle_distance` layout.

| | Lesson | Repo | PX4 |
|---|---|---|---|
| ✅ | Waypoints with acceptance radius, yaw towards waypoint | `guidance/waypoint_follower.py` | `src/modules/navigator`, `NAV_ACC_RAD`, `MPC_YAW_MODE`, `MPC_YAWRAUTO_MAX` (default 60°/s) |
| ⬜ | Jerk-limited trajectory generation | | `src/lib/motion_planning/PositionSmoothing.cpp` |
| ⬜ | Line following for fixed wing (L1 / NPFG) | | `src/lib/npfg` |
| ✅ | Collision prevention: brake towards obstacles, slide along them (Position mode only) | `guidance/collision_prevention.py` | `src/lib/collision_prevention/CollisionPrevention.cpp`, `CP_DIST`, `CP_DELAY` |
| ✅ | Reactive avoidance with a Vector Field Histogram (VFH+ enlargement) | `guidance/vector_field_histogram.py` | offboard (PX4 has no onboard planner); reads `obstacle_distance` |
| ✅ | Reactive avoidance with a potential field, and its local minimum (U-trap) | `guidance/potential_field.py` | offboard |
| ✅ | Global planning: A* on an inflated occupancy grid + shortcut, flown as a mission | `guidance/astar_planner.py` | mission planned off the vehicle, flown by `src/modules/navigator` |
| ⬜ | Sampling-based planning (RRT, RRT*, informed RRT*) in 3D | | — |
| ⬜ | Replanning when the map changes (D* Lite) | | — |
| ⬜ | Path smoothing around obstacles (elastic bands) | | — |
| ⬜ | Occupancy grid built online from range scans (mapping), then replanning | | — |

## Module 5: Behavior

| | Lesson | Repo | PX4 |
|---|---|---|---|
| ⬜ | Flight modes as a state machine: takeoff, mission, RTL, land | | `src/modules/commander`, `src/modules/navigator` |
| ⬜ | Failsafe (GPS loss, low battery) | | `src/modules/commander/failsafe` |

## Module 6: Simulation with Gazebo and PX4 SITL

The real PX4 flies a vehicle in Gazebo. Each lesson puts the Python model from Modules 1-5 next to the
same thing in PX4 + Gazebo. This is where lesson layer 3 happens.

Setup: PX4 `v1.17.0` expects **Gazebo Harmonic** (`gz-harmonic` in `Tools/setup/ubuntu.sh`). Models and
worlds live in the submodule `Tools/simulation/gz`, so run `git submodule update --init --recursive`
in the PX4 clone before building. Start with `make px4_sitl gz_x500`.

| | Lesson | Repo | PX4 |
|---|---|---|---|
| ⬜ | Toolchain and the simulation loop: PX4 SITL ↔ Gazebo, simulated clock, QGroundControl | | `ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim`, `src/modules/simulation/gz_bridge` (`/world/<world>/clock`) |
| ⬜ | Truth vs measurement in Gazebo: which sensors Gazebo produces (IMU, NavSat, FluidPressure, Magnetometer) and how they become uORB topics | `sensors/` | `src/modules/simulation/gz_bridge/GZBridge.cpp` |
| ⬜ | The x500 model vs `QuadrotorParams`: mass, inertia, rotor geometry, motor constants and time constant | `vehicle/quadrotor.py` | `ROMFS/.../airframes/4001_gz_x500` (`CA_ROTOR*`, `MPC_THR_HOVER`), x500 `model.sdf` in `Tools/simulation/gz`, `gz_plugins/generic_motor` |
| ⬜ | Actuator path: `actuator_motors` → ESC outputs → motor plugin | `control/control_allocation.py` | `gz_bridge/GZMixingInterfaceESC.cpp`, `SIM_GZ_EC_*` |
| ⬜ | Same mission in Python and in Gazebo: fly it in SITL, log a ulog, plot both with the same code | `simulations/quadrotor/` | `src/modules/logger`, Flight Review |
| ⬜ | Parameter experiments in SITL: predict with Python first (e.g. `MPC_YAWRAUTO_MAX`, `MC_*` gains, `IMU_GYRO_CUTOFF`), then check in Gazebo | | `param set` |
| ⬜ | Failure injection and failsafe: GPS loss, motor failure | | `src/systemcmds/failure`, `SYS_FAILURE_EN`, `src/modules/commander/failsafe` |
| ⬜ | Offboard control: the repo's guidance drives PX4 in Gazebo through ROS 2 | `guidance/` | `src/modules/uxrce_dds_client`, `px4_msgs`, Micro XRCE-DDS Agent |
| ⬜ | Sensors the Python simulation does not have: optical flow, lidar, depth camera, and EKF2 aiding with them | | `airframes/4021_gz_x500_flow`, `4013_gz_x500_lidar_2d`, `4002_gz_x500_depth`, `gz_plugins/optical_flow` |

Tip: `simulator_sih` (Module 0) runs PX4 without Gazebo and without a GPU, so it is handy for quick,
headless SITL runs in the early modules.
