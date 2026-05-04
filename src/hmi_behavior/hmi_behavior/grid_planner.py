from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Iterable, Optional

from nav_msgs.msg import OccupancyGrid


@dataclass(frozen=True)
class GridSpec:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float


@dataclass(frozen=True)
class GridPlan:
    grid_cells: list[tuple[int, int]]
    world_points: list[tuple[float, float]]
    cost: float


def make_grid_spec(msg: OccupancyGrid) -> GridSpec:
    return GridSpec(
        width=int(msg.info.width),
        height=int(msg.info.height),
        resolution=float(msg.info.resolution),
        origin_x=float(msg.info.origin.position.x),
        origin_y=float(msg.info.origin.position.y),
    )


def clone_grid_data(msg: OccupancyGrid) -> list[int]:
    return [100 if value < 0 else int(value) for value in msg.data]


def world_to_cell(spec: GridSpec, x: float, y: float) -> tuple[int, int] | None:
    grid_x = int(math.floor((x - spec.origin_x) / spec.resolution))
    grid_y = int(math.floor((y - spec.origin_y) / spec.resolution))
    if grid_x < 0 or grid_y < 0 or grid_x >= spec.width or grid_y >= spec.height:
        return None
    return grid_x, grid_y


def cell_to_world(spec: GridSpec, grid_x: int, grid_y: int) -> tuple[float, float]:
    return (
        spec.origin_x + (grid_x + 0.5) * spec.resolution,
        spec.origin_y + (grid_y + 0.5) * spec.resolution,
    )


def is_cell_blocked(data: list[int], spec: GridSpec, grid_x: int, grid_y: int, threshold: int = 50) -> bool:
    if grid_x < 0 or grid_y < 0 or grid_x >= spec.width or grid_y >= spec.height:
        return True
    value = data[grid_y * spec.width + grid_x]
    return value < 0 or value >= threshold


def paint_disc(
    data: list[int],
    spec: GridSpec,
    center_x: float,
    center_y: float,
    radius: float,
    value: int = 100,
) -> None:
    min_x = max(0, int(math.floor((center_x - radius - spec.origin_x) / spec.resolution)))
    max_x = min(spec.width - 1, int(math.floor((center_x + radius - spec.origin_x) / spec.resolution)))
    min_y = max(0, int(math.floor((center_y - radius - spec.origin_y) / spec.resolution)))
    max_y = min(spec.height - 1, int(math.floor((center_y + radius - spec.origin_y) / spec.resolution)))
    radius_sq = radius * radius

    for grid_y in range(min_y, max_y + 1):
        world_y = spec.origin_y + (grid_y + 0.5) * spec.resolution
        dy = world_y - center_y
        for grid_x in range(min_x, max_x + 1):
            world_x = spec.origin_x + (grid_x + 0.5) * spec.resolution
            dx = world_x - center_x
            if dx * dx + dy * dy <= radius_sq:
                data[grid_y * spec.width + grid_x] = value


def paint_oriented_box(
    data: list[int],
    spec: GridSpec,
    center_x: float,
    center_y: float,
    yaw: float,
    size_x: float,
    size_y: float,
    inflate_radius: float = 0.0,
    value: int = 100,
) -> None:
    half_x = size_x * 0.5 + inflate_radius
    half_y = size_y * 0.5 + inflate_radius
    abs_cos = abs(math.cos(yaw))
    abs_sin = abs(math.sin(yaw))
    bound_x = abs_cos * half_x + abs_sin * half_y
    bound_y = abs_sin * half_x + abs_cos * half_y

    min_x = max(0, int(math.floor((center_x - bound_x - spec.origin_x) / spec.resolution)))
    max_x = min(spec.width - 1, int(math.floor((center_x + bound_x - spec.origin_x) / spec.resolution)))
    min_y = max(0, int(math.floor((center_y - bound_y - spec.origin_y) / spec.resolution)))
    max_y = min(spec.height - 1, int(math.floor((center_y + bound_y - spec.origin_y) / spec.resolution)))

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    for grid_y in range(min_y, max_y + 1):
        world_y = spec.origin_y + (grid_y + 0.5) * spec.resolution
        for grid_x in range(min_x, max_x + 1):
            world_x = spec.origin_x + (grid_x + 0.5) * spec.resolution
            dx = world_x - center_x
            dy = world_y - center_y
            local_x = cos_yaw * dx + sin_yaw * dy
            local_y = -sin_yaw * dx + cos_yaw * dy
            if abs(local_x) <= half_x and abs(local_y) <= half_y:
                data[grid_y * spec.width + grid_x] = value


def nearest_free_cell(
    data: list[int],
    spec: GridSpec,
    cell: tuple[int, int],
    threshold: int = 50,
    max_radius: int = 8,
) -> tuple[int, int] | None:
    x, y = cell
    if not is_cell_blocked(data, spec, x, y, threshold):
        return cell

    for radius in range(1, max_radius + 1):
        min_x = max(0, x - radius)
        max_x = min(spec.width - 1, x + radius)
        min_y = max(0, y - radius)
        max_y = min(spec.height - 1, y + radius)

        for nx in range(min_x, max_x + 1):
            for ny in (min_y, max_y):
                if not is_cell_blocked(data, spec, nx, ny, threshold):
                    return nx, ny
        for ny in range(min_y + 1, max_y):
            for nx in (min_x, max_x):
                if not is_cell_blocked(data, spec, nx, ny, threshold):
                    return nx, ny
    return None


def plan_a_star(
    msg: OccupancyGrid,
    data: list[int],
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    threshold: int = 50,
) -> GridPlan | None:
    spec = make_grid_spec(msg)
    start = world_to_cell(spec, start_xy[0], start_xy[1])
    goal = world_to_cell(spec, goal_xy[0], goal_xy[1])
    if start is None or goal is None:
        return None

    start = nearest_free_cell(data, spec, start, threshold=threshold)
    goal = nearest_free_cell(data, spec, goal, threshold=threshold)
    if start is None or goal is None:
        return None

    if start == goal:
        world = [cell_to_world(spec, *start)]
        return GridPlan(grid_cells=[start], world_points=world, cost=0.0)

    frontier: list[tuple[float, tuple[int, int]]] = [(0.0, start)]
    came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
    g_score: dict[tuple[int, int], float] = {start: 0.0}

    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            break

        for next_cell, step_cost in _neighbors(current):
            nx, ny = next_cell
            if is_cell_blocked(data, spec, nx, ny, threshold):
                continue
            if abs(nx - current[0]) == 1 and abs(ny - current[1]) == 1:
                if is_cell_blocked(data, spec, current[0], ny, threshold):
                    continue
                if is_cell_blocked(data, spec, nx, current[1], threshold):
                    continue

            tentative_g = g_score[current] + step_cost
            if tentative_g >= g_score.get(next_cell, math.inf):
                continue
            came_from[next_cell] = current
            g_score[next_cell] = tentative_g
            priority = tentative_g + _heuristic(next_cell, goal)
            heapq.heappush(frontier, (priority, next_cell))

    if goal not in came_from:
        return None

    grid_cells = _reconstruct_path(came_from, goal)
    grid_cells = _simplify_grid_path(data, spec, grid_cells, threshold)
    world_points = [cell_to_world(spec, x, y) for x, y in grid_cells]
    return GridPlan(
        grid_cells=grid_cells,
        world_points=world_points,
        cost=g_score.get(goal, 0.0),
    )


def _neighbors(cell: tuple[int, int]) -> Iterable[tuple[tuple[int, int], float]]:
    x, y = cell
    offsets = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    ]
    for dx, dy, cost in offsets:
        yield (x + dx, y + dy), cost


def _heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _reconstruct_path(
    came_from: dict[tuple[int, int], tuple[int, int] | None],
    goal: tuple[int, int],
) -> list[tuple[int, int]]:
    path: list[tuple[int, int]] = [goal]
    current = goal
    while came_from[current] is not None:
        current = came_from[current]  # type: ignore[assignment]
        path.append(current)
    path.reverse()
    return path


def _simplify_grid_path(
    data: list[int],
    spec: GridSpec,
    path: list[tuple[int, int]],
    threshold: int,
) -> list[tuple[int, int]]:
    if len(path) <= 2:
        return path

    simplified: list[tuple[int, int]] = [path[0]]
    anchor_index = 0
    while anchor_index < len(path) - 1:
        probe_index = len(path) - 1
        while probe_index > anchor_index + 1:
            if _line_of_sight(data, spec, path[anchor_index], path[probe_index], threshold):
                break
            probe_index -= 1
        simplified.append(path[probe_index])
        anchor_index = probe_index
    return simplified


def _line_of_sight(
    data: list[int],
    spec: GridSpec,
    start: tuple[int, int],
    end: tuple[int, int],
    threshold: int,
) -> bool:
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x = x0
    y = y0
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1

    if dx > dy:
        err = dx / 2.0
        while x != x1:
            if is_cell_blocked(data, spec, x, y, threshold):
                return False
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            if is_cell_blocked(data, spec, x, y, threshold):
                return False
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    return not is_cell_blocked(data, spec, x1, y1, threshold)
