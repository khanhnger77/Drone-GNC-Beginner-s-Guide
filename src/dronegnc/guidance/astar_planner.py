"""
astar_planner.py

Author: Khanh Ngo

Global path planning with A* on an occupancy grid, in the horizontal North-East plane.
Unlike the reactive methods (VFH, potential field), A* searches the whole map, so it
finds the way out of a U-shaped obstacle. The price: it needs a map.

  1. Occupancy grid: a cell is occupied if an obstacle is closer than the inflation
     radius (drone radius + margin), so the path can treat the drone as a point.
  2. A*: expand the cell with the lowest cost-so-far + straight-line distance to the goal,
     8 neighbors, until the goal is reached.
  3. Shortcut: drop the waypoints that a straight, collision-free line can skip, so the
     drone gets a few waypoints instead of one per cell.

PX4 does not plan paths onboard: a mission planner (QGroundControl) or an offboard
computer does, and PX4 flies the resulting waypoints (navigator mission).
The grid and the search follow AStarPathPlanner in AutonomousVehicleControlBeginnersGuide.
"""

import heapq

import numpy as np


class OccupancyGrid:
    """
    Binary grid over a rectangle of the North-East plane
    """

    def __init__(self, obstacle_map, north_range_m, east_range_m, resolution_m=0.25, inflation_radius_m=1.0):
        """
        Constructor
        obstacle_map: ObstacleMap (the prior map the planner knows)
        north_range_m, east_range_m: (min, max) of the area covered by the grid [m]
        resolution_m: Cell size [m]
        inflation_radius_m: Cells closer than this to an obstacle are occupied [m]
        """

        self.origin_ne_m = np.array([north_range_m[0], east_range_m[0]])
        self.resolution_m = resolution_m
        num_north = int(np.ceil((north_range_m[1] - north_range_m[0]) / resolution_m)) + 1
        num_east = int(np.ceil((east_range_m[1] - east_range_m[0]) / resolution_m)) + 1
        self.occupied = np.zeros((num_north, num_east), dtype=bool)
        for i in range(num_north):
            for j in range(num_east):
                self.occupied[i, j] = obstacle_map.distance_to_nearest_m(self.cell_to_ne((i, j))) < inflation_radius_m

    def ne_to_cell(self, point_ne_m):
        """
        Grid cell (row = North, column = East) that contains a point
        """

        i, j = np.round((np.asarray(point_ne_m) - self.origin_ne_m) / self.resolution_m).astype(int)
        return int(i), int(j)

    def cell_to_ne(self, cell):
        """
        [North, East] center of a cell [m]
        """

        return self.origin_ne_m + self.resolution_m * np.asarray(cell, dtype=float)

    def is_free(self, cell):
        """
        True if the cell is inside the grid and not occupied
        """

        i, j = cell
        return 0 <= i < self.occupied.shape[0] and 0 <= j < self.occupied.shape[1] and not self.occupied[i, j]

    def is_line_free(self, start_ne_m, end_ne_m):
        """
        True if every cell along the straight line between two points is free
        """

        num_checks = int(np.ceil(np.linalg.norm(end_ne_m - start_ne_m) / (0.5 * self.resolution_m))) + 1
        return all(self.is_free(self.ne_to_cell(start_ne_m + s * (end_ne_m - start_ne_m)))
                   for s in np.linspace(0.0, 1.0, num_checks))


def astar(grid, start_ne_m, goal_ne_m):
    """
    Shortest path of cells from start to goal, as a list of [North, East] points [m], or None
    grid: OccupancyGrid
    start_ne_m, goal_ne_m: [North, East] [m]
    """

    start, goal = grid.ne_to_cell(start_ne_m), grid.ne_to_cell(goal_ne_m)
    neighbors = [(di, dj) for di in (-1, 0, 1) for dj in (-1, 0, 1) if (di, dj) != (0, 0)]

    def heuristic(cell):
        return np.hypot(cell[0] - goal[0], cell[1] - goal[1])  # straight line, never overestimates

    open_heap = [(heuristic(start), start)]  # (cost so far + heuristic, cell)
    cost_so_far = {start: 0.0}
    came_from = {start: None}
    while open_heap:
        _, cell = heapq.heappop(open_heap)
        if cell == goal:
            path = []
            while cell is not None:
                path.append(grid.cell_to_ne(cell))
                cell = came_from[cell]
            return path[::-1]
        for di, dj in neighbors:
            neighbor = (cell[0] + di, cell[1] + dj)
            if not grid.is_free(neighbor):
                continue
            new_cost = cost_so_far[cell] + np.hypot(di, dj)
            if new_cost < cost_so_far.get(neighbor, np.inf):
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = cell
                heapq.heappush(open_heap, (new_cost + heuristic(neighbor), neighbor))
    return None  # the goal cannot be reached


def shortcut(grid, path_ne_m):
    """
    Keep only the waypoints a straight, free line cannot skip
    """

    waypoints = [path_ne_m[0]]
    i = 0
    while i < len(path_ne_m) - 1:
        # Farthest point still visible in a straight line from the current waypoint
        j = len(path_ne_m) - 1
        while j > i + 1 and not grid.is_line_free(path_ne_m[i], path_ne_m[j]):
            j -= 1
        waypoints.append(path_ne_m[j])
        i = j
    return np.array(waypoints)
