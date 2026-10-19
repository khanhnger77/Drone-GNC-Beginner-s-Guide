# Lesson 7: Vector Field Histogram

A reactive planner: at every scan, pick the free direction closest to the goal. VFH works directly on a
polar histogram, which is exactly the shape of PX4's `obstacle_distance`.

- Simulation: [`simulations/avoidance/vfh_obstacle_avoidance.py`](../../simulations/avoidance/vfh_obstacle_avoidance.py)
- Code: [`guidance/vector_field_histogram.py`](../../src/dronegnc/guidance/vector_field_histogram.py)
- Run: `python simulations/avoidance/vfh_obstacle_avoidance.py`
- Origin: Borenstein & Koren (1991) VFH; Ulrich & Borenstein (1998) VFH+. The histogram step follows
  `PolarHistogram` in the [vehicle repository](https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide).

## 1. Theory

**1. Polar histogram.** Each reading at distance $d_j$ in direction $\beta_j$ has a magnitude that grows as
the obstacle gets closer:

$$m_j = 1 - d_j / d_{max}$$

**2. Enlargement (VFH+).** The drone is not a point. A reading at distance $d_j$ blocks every direction
within

$$\gamma_j = \arcsin\!\left(\min\!\left(1,\; r_{safe} / d_j\right)\right)$$

of $\beta_j$, where $r_{safe}$ is the drone radius plus a margin. Bin $k$ gets $h_k = \max_j m_j$ over the
readings that cover it.

**3. Threshold.** Bins with $h_k > \tau$ are blocked. With $\tau = 0.7$ and a 10 m sensor, obstacles closer
than 3 m block.

**4. Valleys and candidates.** Runs of free bins are valleys. A narrow valley gives one candidate, its
center. A wide one gives its two sides (kept away from the edges), plus the goal direction if it lies
inside.

**5. Choice.** The candidate with the lowest cost wins:

$$J(c) = \lvert c - \psi_{goal} \rvert + \mu\,\lvert c - c_{previous} \rvert$$

The second term ($\mu = 1$) keeps the choice from flipping between two valleys.

**6. Speed.** Slower when obstacles are close ahead (within ±45° of the chosen direction) and near the goal.

## 2. The code

| Step | In `VectorFieldHistogram` |
|---|---|
| 1-2 | `polar_histogram()` |
| 3 | `blocked = self.histogram > self.threshold` in `update()` |
| 4 | `candidate_directions_rad()` |
| 5 | `cost()` and `min(candidates, key=cost)` in `update()` |
| 6 | end of `update()` |

The simulation runs VFH on every new scan (10 Hz) and sends the result to the cascade as a velocity
feed-forward, holding 2 m altitude.

## 3. What you should see

22 random pillars between the start and a goal 24 m North (seed 1).

| | Value |
|---|---|
| Goal reached | t = 22.1 s |
| Smallest distance to a pillar | 0.90 m |
| Success over 6 random forests (seeds 1-6) | 6 / 6 |

**Tuning matters.** The defaults ($r_{safe}$ = 0.8 m, $\mu$ = 1) were chosen by running those 6 forests.
Other values fail in some of them:

| Change | Forests reached (seeds 1-6) |
|---|---|
| none | 6 / 6 |
| $\mu$ = 0.5 | 5 / 6 |
| $r_{safe}$ = 0.6 m | 4 / 6 |
| $\mu$ = 0 | 3 / 6 |

A failed run looks like this: the drone enters a pocket between pillars, turns back, comes back, and
never arrives. VFH has no memory: it only sees the current scan.

**And it has a blind spot.** In the U-shaped wall of lesson 8, VFH is trapped inside the U too (it stops
12 m from the goal). Every reactive method shares this problem; lesson 9 solves it.

## 4. In PX4 `v1.17.0`

| This repo | PX4 | Note |
|---|---|---|
| Input scan | `obstacle_distance` (`msg/ObstacleDistance.msg`), the same 72 × 5° layout | |
| VFH itself | **not in PX4** | PX4 does not steer around obstacles onboard |
| Where it would run | A companion computer, sending `trajectory_setpoint` in offboard mode (`uxrce_dds_client` + ROS 2) | Module 6 lesson |
| Velocity feed-forward with NaN position | `trajectory_setpoint` + `PositionControl::_positionControl()` (`addIfNotNanVector3f`) | Same convention |

## 5. Try it

1. Plot `history["vfh_histogram"]` as an image (time × bin) and mark `vfh_direction_rad`. Can you see the
   valleys open and close?
2. Set `previous_direction_weight=0.0` and run seeds 1-6. How many reach the goal?
3. Replace the forest with `u_shaped_wall(14.0, 6.0, 4.0)`. Where does the drone end up?
