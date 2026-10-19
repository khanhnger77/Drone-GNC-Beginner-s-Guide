# Lesson 0: Frames and rotations

Every later lesson moves vectors between two frames. Get the conventions right once, and use exactly
the ones PX4 uses, so that numbers can be compared with a flight log without sign flips.

- Code: [`src/dronegnc/common/rotation.py`](../../src/dronegnc/common/rotation.py)
- Tests: [`tests/test_rotation.py`](../../tests/test_rotation.py)

## 1. Theory

**World frame NED**: x North, y East, z Down. Altitude is $-z$.
**Body frame FRD**: x Forward, y Right, z Down. Rotor thrust points along $-z_{body}$.

The attitude is the rotation from body to NED. Three ways to write it:

| Form | Size | Use |
|---|---|---|
| Rotation matrix (DCM) $R$ | 9 | Rotate a vector: $v_{NED} = R\,v_{body}$ |
| Quaternion $q = [w, x, y, z]$ | 4 | Store and integrate the attitude, no singularity |
| Euler angles (roll $\phi$, pitch $\theta$, yaw $\psi$), order ZYX | 3 | Read and plot; singular at $\theta = \pm 90°$ |

ZYX means: start aligned with NED, turn by yaw about z, then by pitch about the new y, then by roll
about the new x.

**Integrating the attitude.** With body rates $\omega$ over a step $\Delta t$, the rotation in that step
is the rotation vector $\omega\,\Delta t$. As a quaternion:

$$\Delta q = \left[\cos\frac{|\omega|\Delta t}{2},\; \sin\frac{|\omega|\Delta t}{2}\,\frac{\omega}{|\omega|}\right], \qquad q_{k+1} = q_k \otimes \Delta q$$

The body rates are multiplied on the **right**, because they are expressed in the body frame.

**The 2D slice.** The planar lessons live in the North-Down plane. A positive pitch lifts the nose:

$$R(\theta) = \begin{bmatrix} \cos\theta & \sin\theta \\ -\sin\theta & \cos\theta \end{bmatrix}, \qquad \begin{bmatrix} x \\ z \end{bmatrix}_{NED} = R(\theta) \begin{bmatrix} x \\ z \end{bmatrix}_{body}$$

This is the x-z block of the 3D matrix for roll = yaw = 0, so nothing changes when moving to 3D.

## 2. The code

| Function | What it does |
|---|---|
| `quat_multiply(p, q)` | Hamilton product $p \otimes q$ |
| `quat_from_rotation_vector(v)` | $\Delta q$ above (first-order form for tiny angles) |
| `quat_to_dcm(q)`, `dcm_to_quat(R)` | Quaternion ↔ matrix, body → NED |
| `euler_to_quat`, `quat_to_euler` | ZYX Euler ↔ quaternion |
| `wrap_pi(a)` | Angle into $[-\pi, \pi)$. Use it on every angle difference |
| `planar_dcm(pitch)` | The 2D slice |

Integration as used by the 6DOF model ([`vehicle/quadrotor.py`](../../src/dronegnc/vehicle/quadrotor.py)):

```python
delta_q = quat_from_rotation_vector(s.angular_velocity_body_rps * dt_s)
s.quaternion = quat_normalize(quat_multiply(s.quaternion, delta_q))
```

## 3. What you should see

`./run_tests.sh tests/test_rotation.py` passes. The tests check round trips (Euler → quaternion → Euler,
quaternion → DCM → quaternion), that a 90° yaw turns the body x axis to East, that a rotation vector
about z equals a yaw, and that `planar_dcm` is exactly the x-z block of the 3D pitch rotation.

## 4. In PX4 `v1.17.0`

| This repo | PX4 |
|---|---|
| `quat_multiply`, `quat_conjugate` | `matrix::Quatf::operator*`, `inversed()` in `src/lib/matrix/matrix/Quaternion.hpp` |
| `quat_to_dcm`, `dcm_to_quat` | `matrix::Dcmf(q)`, `matrix::Quatf(dcm)` (`Dcm.hpp`, `Quaternion.hpp`) |
| `quat_to_euler`, `euler_to_quat` | `matrix::Eulerf(q)`, `matrix::Quatf(Eulerf(...))` (`Euler.hpp`) |
| `wrap_pi` | `matrix::wrap_pi` |
| quaternion layout `[w, x, y, z]` | `vehicle_attitude.q`: "Quaternion rotation from the FRD body frame to the NED earth frame", same order |

## 5. Try it

1. A drone has yaw 90° and pitch 10° nose up. Which NED direction does its thrust point to? Check with
   `quat_to_dcm(euler_to_quat(0, radians(10), radians(90))) @ [0, 0, -1]`.
2. Integrate a constant body rate of 1 rad/s about x for 1 s in 200 steps. Compare the result with
   `euler_to_quat(1.0, 0, 0)`. Then remove `quat_normalize` and integrate for 1000 s: what happens to $|q|$?
3. Why is `wrap_pi` needed in `yaw_sp - yaw` when yaw goes from 179° to -179°?
