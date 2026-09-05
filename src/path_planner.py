"""
Classic A* over the GridWorld. Returns a list of (x, y) waypoints from start
to goal, or None if no path exists.
"""
import heapq
from typing import List, Optional, Tuple

from src.simulation_world import GridWorld

Point = Tuple[int, int]

_NEIGHBORS_8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def _heuristic(a: Point, b: Point) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def a_star(world: GridWorld, start: Point, goal: Point) -> Optional[List[Point]]:
    if world.is_blocked(*start) or world.is_blocked(*goal):
        return None

    open_set = [(0.0, start)]
    came_from = {}
    g_score = {start: 0.0}
    visited = set()

    while open_set:
        _, current = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for dx, dy in _NEIGHBORS_8:
            neighbor = (current[0] + dx, current[1] + dy)
            if world.is_blocked(*neighbor):
                continue
            step_cost = (dx * dx + dy * dy) ** 0.5
            tentative_g = g_score[current] + step_cost
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + _heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))

    return None
