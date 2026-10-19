# Lesson 9: Global planning with A*

Reactive methods only see the present scan, so a U-shaped wall traps them. A global planner searches the
whole map before flying. The cost: it needs that map.

- Simulation: [`simulations/avoidance/astar_global_planning.py`](../../simulations/avoidance/astar_global_planning.py)
- Code: [`guidance/astar_planner.py`](../../src/dronegnc/guidance/astar_planner.py)
- Run: `python simulations/avoidance/astar_global_planning.py`
- Origin: Hart, Nilsson & Raphael (1968). The grid and the search follow `AStarPathPlanner` in the
  [vehicle repository](https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide).

## 1. Theory

**1. Occupancy grid with inflation.** Cut the map into cells of 0.25 m. A cell is occupied if an obstacle
is closer than the inflation radius $r_{infl}$ (drone radius + margin). The drone can then be planned
as a point.

**2. A\*.** Keep a priority queue of cells ordered by

$$f(n) = g(n) + h(n), \qquad g = \text{cost from the start}, \qquad h = \lVert n - goal \rVert$$

Pop the lowest $f$, relax its 8 neighbors (cost 1 or $\sqrt 2$), stop at the goal. Because the straight-line
distance $h$ never overestimates the true cost, the path found is the shortest on the grid.

**3. Shortcut.** A grid path has one point per cell and zig-zags in 45° steps. From each waypoint, jump to
the farthest later point that a straight, collision-free line can reach. What is left is a handful of
waypoints.

**4. Fly it.** The waypoints become a mission for the waypoint follower of lesson 2 and the 3D cascade of
lesson 4.

## 2. The code

| Step | Where |
|---|---|
| 1 | `OccupancyGrid.__init__()`, `ne_to_cell()`, `cell_to_ne()`, `is_free()` |
| 2 | `astar()`: `heapq`, `cost_so_far`, `came_from` |
| 3 | `shortcut()`, `OccupancyGrid.is_line_free()` |
| 4 | `WaypointFollower(waypoints_ned_m[1:])` in the simulation |

The planner is given the **true map** (a prior map, as a mission planner would have). The range sensor
runs only for the drawing. Building the map online from scans is a planned lesson.

## 3. What you should see

The same U-shaped wall as lesson 8, `r_infl` = 1.2 m.

| | Value |
|---|---|
| Grid path | 97 cells |
| After shortcut | 4 waypoints, around the West side of the U |
| Goal reached | t = 7.2 s |
| Smallest distance to the wall | 1.24 m |

Compare with lesson 8: same world, same drone, same controller; the potential field never arrives.

## 4. In PX4 `v1.17.0`

| This repo | PX4 | Note |
|---|---|---|
| A* before take-off | **not onboard**: a ground station (QGroundControl) or a companion computer plans | |
| Waypoints after shortcut | A mission (`mission_item`s) uploaded over MAVLink | |
| Flying the waypoints | `src/modules/navigator` (mission), `NAV_ACC_RAD` | Lesson 2 |
| Position steps between waypoints | `FlightTaskAuto` smooths them (`src/lib/motion_planning/PositionSmoothing.cpp`) | Planned lesson |

## 5. Try it

1. Set `inflation_radius_m=0.3`. How close does the path pass to the wall, and does the drone touch it
   (`clearance_m`)? Why does the drone need a larger margin than its size?
2. Set the heuristic to 0 (Dijkstra). Count the expanded cells with and without the heuristic.
3. Close the U with a fourth wall. What does `astar()` return, and what should the drone do then?
