# Lesson 8: Potential field and local minima

The most intuitive reactive planner: the goal pulls, obstacles push. It is smooth and short to write,
and it shows clearly why reactive methods are not enough.

- Simulation: [`simulations/avoidance/potential_field_obstacle_avoidance.py`](../../simulations/avoidance/potential_field_obstacle_avoidance.py)
- Code: [`guidance/potential_field.py`](../../src/dronegnc/guidance/potential_field.py)
- Run: `python simulations/avoidance/potential_field_obstacle_avoidance.py`
- Origin: Khatib (1986). The vehicle repository has the grid version (`potential_field_map_construction`).

## 1. Theory

The velocity setpoint is the negative gradient of a potential:

$$v = \underbrace{k_{att}\,(p_{goal} - p)}_{\text{capped at } v_{max}} \;-\; \sum_{d_i < d_0} k_{rep}\left(\frac{1}{d_i} - \frac{1}{d_0}\right)\frac{1}{d_i^2}\,n_i, \qquad \text{then } \lVert v \rVert \le v_{max}$$

where $n_i$ is the unit vector towards reading $i$ of the scan and $d_0$ the influence distance. Every
reading of the 72-bin scan is a small repelling point.

**Cap the attraction.** Without the cap, a goal 10 m away pulls with $k_{att} \cdot 10$, which overpowers
the repulsion and drives the drone within a few centimeters of obstacles.

**Local minimum.** Where attraction and repulsion cancel, $v = 0$ before the goal. Inside a U-shaped wall
facing the goal, this happens on the axis of the U. Nothing in the method can get out of it.

## 2. The code

`PotentialField.update()` is the equation above, in three blocks: capped attraction, sum of repulsions,
final speed limit. The simulation calls `fly()` twice, on two worlds, and returns both histories in a dict.

## 3. What you should see

| World | Result |
|---|---|
| Sparse forest (10 pillars, seed 3) | Goal reached, smallest distance 0.86 m |
| U-shaped wall (back wall 14 m North, 8 m wide) | **Trapped** at 12.75 m North, inside the U, 11 m from the goal |

More results that are not in the script but worth showing:

- In the **dense forest of lesson 7**, the potential field fails, whatever repulsion law ($1/d$, $1/d^2$,
  $1/d^3$) and gains were tried. Around pillars it oscillates: the setpoint jumps between ±2.5 m/s at
  the 10 Hz scan rate.
- With 10 pillars and **seed 1**, a pillar stands almost on the straight line to the goal (8 m North).
  The drone gets stuck beside it at (7.7 m N, 1.5 m E), the setpoint still swinging at about 2 m/s,
  and never passes it.
- VFH is trapped in the same U (lesson 7).

## 4. In PX4 `v1.17.0`

| This repo | PX4 | Note |
|---|---|---|
| Repulsion from the scan | `obstacle_distance` | Same input as lessons 6-7 |
| Potential field | **not in PX4** | Would run offboard, like VFH |

## 5. Try it

1. Remove the attraction cap. What is the smallest distance to a pillar now?
2. Add a small tangential force (rotate the repulsion by 90° when the drone is nearly stopped). Can it
   escape the U? Is it guaranteed to?
3. Run the U world with `VectorFieldHistogram` instead. Compare where both methods stop.
