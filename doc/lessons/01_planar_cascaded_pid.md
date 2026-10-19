# Lesson 1: Cascaded PID control (2D)

A multicopter cannot push itself sideways. To move North it must first tilt its thrust, and to tilt
it must first spin up a pitch rate. Cascaded control follows that chain, one loop per link.

- Simulation: [`simulations/planar/planar_hover_step_response.py`](../../simulations/planar/planar_hover_step_response.py)
- Code: [`control/planar_cascade_controller.py`](../../src/dronegnc/control/planar_cascade_controller.py),
  [`control/pid.py`](../../src/dronegnc/control/pid.py),
  [`vehicle/planar_quadrotor.py`](../../src/dronegnc/vehicle/planar_quadrotor.py)
- Run: `python simulations/planar/planar_hover_step_response.py`

![Step response](../images/planar_hover_step_response.gif)

## 1. Theory

**Vehicle.** Two rotors (front, rear) at distance $l$ from the center, mass $m$, inertia $I$, pitch $\theta$:

$$m\,\ddot{p} = R(\theta)\begin{bmatrix}0\\-T\end{bmatrix} + m\begin{bmatrix}0\\g\end{bmatrix}, \qquad I\,\ddot\theta = l\,(T_f - T_r), \qquad T = T_f + T_r$$

**The cascade.** Each loop turns its error into the setpoint of the next, faster loop:

| Loop | Law | Output |
|---|---|---|
| Position | $v_{sp} = K_p\,(p_{sp} - p)$, limited to 4 m/s | velocity setpoint |
| Velocity | PID on $v_{sp} - v$ | acceleration setpoint $a_{sp}$ |
| Thrust vector | $F = m\,(a_{sp} - g)$ | thrust in NED |
| Thrust → attitude | $\theta_{sp} = \operatorname{atan2}(-F_x, -F_z)$, limited to 35° | pitch setpoint, collective $T$ |
| Attitude | $\dot\theta_{sp} = K_\theta\,(\theta_{sp} - \theta)$ | pitch rate setpoint |
| Rate | PID on $\dot\theta_{sp} - \dot\theta$, times $I$ | pitch moment $M$ |
| Mixer | $T_f = T/2 + M/2l$, $T_r = T/2 - M/2l$ | rotor thrusts |

**Why it works: bandwidth separation.** Each inner loop must settle much faster than the loop around
it, so the outer loop can treat it as "done". Here the gains are roughly 1 < 3 < 8 < 20 rad/s from
position to rate.

**Altitude first.** When the tilt is limited, the collective thrust is $T = -F_z / \cos\theta_{sp}$: the
vertical part of the thrust is kept, and the horizontal part is what gets cut.

**Derivative on measurement.** The D term uses $-\dot{y}$ instead of $\dot{e}$. A step in the setpoint
would make $\dot e$ infinite for one step ("derivative kick"). The measurement never jumps.

## 2. The code

`PlanarCascadeController.update()` is the table above, numbered 1 to 7 in the comments. Each loop's
output is stored in `self.setpoints` for plotting. The simulation:

```python
position_sp_m = np.array([0.0, -2.0]) if time_s < 5.0 else np.array([3.0, -2.0])
rotor_thrusts_n = controller.update(position_sp_m, vehicle.state, dt_s)
vehicle.step(rotor_thrusts_n, dt_s)
```

The controller reads `vehicle.state`, the truth. Lesson 3 replaces it with an estimate.

## 3. What you should see

The vehicle climbs to 2 m, then the setpoint jumps 3 m North at t = 5 s. Measured:

| | Value |
|---|---|
| Pitch reaches the 35° tilt limit (−34.8°) | t = 5.34 s |
| Vehicle has moved 5 cm North | t = 5.24 s |
| Rise time 10 % → 90 % of the step | 1.17 s |
| Overshoot | none (final x = 2.99 m) |
| Largest altitude change during the step | 4.4 cm |

The order is the point of the lesson: the **pitch moves first**, the position follows. And thanks to
"altitude first", the vehicle barely loses height while tilted at the limit.

## 4. In PX4 `v1.17.0`

| This repo | PX4 | Note |
|---|---|---|
| Position P, `position_gain` | `PositionControl::_positionControl()`, `MPC_XY_P` (default 0.95), `MPC_Z_P` | |
| Velocity PID, `velocity_gains` | `PositionControl::_velocityControl()`, `MPC_XY_VEL_P_ACC` (1.8), `MPC_XY_VEL_I_ACC`, `MPC_XY_VEL_D_ACC` | PX4 gains are in acceleration units too |
| $F = m(a - g)$ | `PositionControl::_accelerationControl()` | PX4 has no mass: it scales by the hover thrust `MPC_THR_HOVER` |
| Tilt limit 35° | `MPC_TILTMAX_AIR` (default 45°), `ControlMath::limitTilt` | |
| Altitude first | `MPC_THR_XY_MARG` keeps a horizontal margin, the rest goes to vertical | Same priority idea, different mechanism |
| Attitude P | `mc_att_control`, `MC_PITCH_P` | |
| Rate PID, D on measurement | `src/lib/rate_control/rate_control.cpp`: `torque = P*error + integral - D*angular_accel + FF*rate_sp` | PX4 differentiates the measured rate (angular acceleration) |
| Integral limit | `MC_RR_INT_LIM`, `MC_PR_INT_LIM` | |
| Mixer | `control_allocator` | Lesson 4 |

## 5. Try it

1. Set `rate_gains=(5.0, 0.5, 0.0)`: the rate loop is now slower than the attitude loop. What happens to
   the step response? (Bandwidth separation broken.)
2. Remove the `min(..., 2 * max_rotor_thrust)` and the tilt limit. How far does the vehicle tilt, and does
   it lose altitude?
3. Change `Pid` to use the derivative of the error and add a D gain. Plot `rotor_thrusts_n` at t = 5 s.
