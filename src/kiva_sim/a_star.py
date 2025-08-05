from __future__ import annotations
import os
import heapq
import logging
from pathlib import Path
from collections import defaultdict
from typing import List, Dict, Tuple, Optional
from kiva_sim.states import Direction
from kiva_sim.maps import Map


# Load environment variables from .env file
TIME_OF_TURN90 = int(os.getenv("TIME_OF_TURN90", "2"))
TIME_OF_TURN180 = int(os.getenv("TIME_OF_TURN180", "3"))

# Cost for spatial movement, can be adjusted based on requirements
SPATIAL_COST = int(os.getenv("SPATIAL_COST", "10"))
TEMPORAL_COST = int(os.getenv("TEMPORAL_COST", "10"))


class Node:
    """A node in the spatial-temporal A* pathfinding algorithm for AGV navigation.
    This class represents a state in the search space, including position, time,
    direction, and pathfinding costs. It supports constraint checking for multi-agent
    collision avoidance.
    Attributes:
        x (int): X coordinate position.
        y (int): Y coordinate position.
        t (int): Timestep.
        direction (Direction): Current facing direction.
        g (int): Cost from start to current node.
        h (int): Heuristic cost from current node to goal.
        father (Optional[Node]): Parent node in the path.
        valid (bool): Whether the node is valid.
        f (int): Total cost (g + h).
    """

    def __init__(
        self,
        x: int,
        y: int,
        t: int,
        direction: Direction,
        g: int,
        h: int,
        father: Optional[Node] = None,
    ):
        self.x = x
        self.y = y
        self.t = t
        self.direction = direction
        self.g = g
        self.h = h
        self.father = father
        self.valid = True
        self.f = g + h

    def __lt__(self, other: Node) -> bool:
        """Less than or equal comparison based on f value (g + h)."""
        return self.f < other.f

    def __repr__(self) -> str:
        """String representation of the Node."""
        return f"Node(x={self.x}, y={self.y}, t={self.t}, direction={self.direction}, g={self.g}, h={self.h}, f={self.f})\n"

    @classmethod
    def get_h(cls, x: int, y: int, endx: int, endy: int) -> int:
        """Calculate the heuristic (h) value using Manhattan distance."""
        return (abs(x - endx) + abs(y - endy)) * SPATIAL_COST

    @classmethod
    def get_dt(cls, new_direction: Direction, cur_direction: Direction) -> int:
        """Calculate the time cost for turning to a new direction."""
        turning_angle = abs(new_direction - cur_direction)
        if turning_angle > 180:
            turning_angle -= 180
        dt = 1 + turning_angle // 90 * TIME_OF_TURN90
        return dt

    def _voilate_vertex_constraint(self, constraint: Dict) -> bool:
        """Check if the node violates a vertex constraint.
        Direct match: The current node's position and time exactly match a constraint
        Turn scenario: During a turning maneuver, the AGV stays at the parent's position for multiple timesteps,
            so if a constraint applies to that position during the turn duration, it's also a violation
        """

        return (constraint["loc"] == [(self.x, self.y)] and constraint["timestep"] == self.t) or (
            self.father is not None
            and constraint["loc"] == [(self.father.x, self.father.y)]
            and (self.father.t < constraint["timestep"] < self.t)
        )

    def _violate_edge_constraint(self, constraint: Dict) -> bool:
        """Check if the node violates an edge constraint.
        This checks if the current node's position and time match an edge constraint,
        which is defined by two positions and a timestep.
        """
        return (
            self.father is not None
            and constraint["loc"] == [(self.father.x, self.father.y), (self.x, self.y)]
            and constraint["timestep"] == self.t
        )

    def _voilate_time_constraint(self, endx, endy, earliest_stopping_time: float) -> bool:
        """Check if the node violates a time constraint.
        This checks if early arrival at destination occurs.
        """
        return (endx, endy) == (self.x, self.y) and self.t < earliest_stopping_time

    def violate_constraints(
        self, constraints: List[Dict], endx: int, endy: int, earliest_stopping_time: float
    ) -> bool:
        """Check if the node violates any constraints.
        This method validates whether the current node violates any of the given constraints,
        including vertex collisions, edge collisions, and early arrival at destination.
        Args:
            constraints (list): List of constraint dictionaries. Each constraint contains:
                - agent (int): AGV id
                - loc (list): Constraint location, either [(x, y)] for vertex collision
                             or [(x1, y1), (x2, y2)] for edge collision
                - timestep (int): Timestep when collision occurs. For edge collisions,
                                 this is the timestep after position swap (timestep of (x2, y2))
                - type (str): One of 'positive', 'negative', 'additional'
            endx (int): X coordinate of destination.
            endy (int): Y coordinate of destination.
            earliest_stopping_time (int): Maximum timestep in the constraint list.
        Returns:
            bool: True if the node violates any constraint in the constraint list,
                  False otherwise.
        """
        return any(
            self._voilate_vertex_constraint(constraint)
            or self._violate_edge_constraint(constraint)
            or self._voilate_time_constraint(endx, endy, earliest_stopping_time)
            for constraint in constraints
        )


def fill_path(
    path: List[Tuple[int, int, int, Direction]],
) -> List[Tuple[int, int, int, Direction]]:
    """Fill missing time steps in a path to ensure temporal continuity.

    Args:
        path: List of waypoints as (x, y, t, direction) tuples.

    Returns:
        A path with all intermediate time steps filled in, where the AGV
        stays at the previous position during turning maneuvers.
    """
    filled_path = []

    for i, (x, y, t, direction) in enumerate(path):
        # If the current waypoint is the first one, just add it
        if i == 0:
            filled_path.append((x, y, t, direction))
            continue
        # If the time step is not consecutive, fill the gap
        last_x, last_y, last_t, last_direction = filled_path[-1]
        if t != last_t + 1:
            for j in range(last_t + 1, t - 1):
                filled_path.append((last_x, last_y, j, last_direction))
            filled_path.append((last_x, last_y, t - 1, direction))
        filled_path.append((x, y, t, direction))
    return filled_path


def resolve_constraints(
    constraints: List[Dict],
) -> Tuple[List[Dict], Dict, Dict, int]:
    """Resolve constraints into positive, negative, and additional constraints."""

    positive_constraints: List[Dict] = []
    negative_constraints: Dict[int, List[Dict]] = defaultdict(list)
    additional_constraint: Dict = {}

    earliest_stopping_time = 0

    for constraint in constraints:
        if constraint["type"] == "positive":
            positive_constraints.append(constraint)
            earliest_stopping_time = max(earliest_stopping_time, constraint["timestep"])
        if constraint["type"] == "negative":
            negative_constraints[constraint["timestep"]].append(constraint)
            earliest_stopping_time = max(earliest_stopping_time, constraint["timestep"])
        if constraint["type"] == "additional":
            additional_constraint = constraint

    return (
        positive_constraints,
        negative_constraints,
        additional_constraint,
        earliest_stopping_time,
    )


def astar(
    mapdata: Map,
    start: Tuple[int, int],
    end: Tuple[int, int],
    startdirection: Direction,
    constraints: List[Dict] = [],
    max_iter=1000,
):
    """Find the shortest path that satisfies given constraints using A* algorithm.

    This function implements the A* pathfinding algorithm with support for multi-agent
    collision avoidance through constraint-based planning. It considers turning costs
    and time constraints for autonomous ground vehicle (AGV) navigation.

    Args:
        mapdata (ndarray): 2D grid map matrix where 0 represents passable cells and
            non-zero values represent obstacles.
        start (Tuple[int, int]): x, y coordinates of the starting position (row index, column index).
        end (iTuple[int, int]): x, y coordinates of the destination position (row index, column index).
        startdirection (Direction): Initial direction of the AGV. Must be one of
            Direction.UP, Direction.DOWN, Direction.LEFT, or Direction.RIGHT.
        constraints (List[Dict], optional): List of constraint dictionaries for
            collision avoidance. Each constraint contains:
            - agent (int): AGV identifier
            - loc (List[Tuple[int, int]]): Constraint location(s)
            - timestep (int): Time step when constraint applies
            - type (str): Constraint type ('positive', 'negative', or 'additional')
            Defaults to empty list.

    Returns:
        List[Tuple[int, int, int, Direction]] or None: If a path exists, returns a list
            of tuples representing the path where each tuple contains (x, y, t, direction).
            Returns None if no valid path is found.

    Note:
        - The algorithm considers turning penalties based on TIME_OF_TURN90 and
          TIME_OF_TURN180 constants
        - Supports vertex collision constraints, edge collision constraints, and
          timing constraints for multi-agent coordination
        - Time complexity depends on the grid size and constraint complexity

    Example:
        >>> mapdata = np.array([[0, 0, 0], [0, 1, 0], [0, 0, 0]])
        >>> path = astar(mapdata, 0, 0, 2, 2, Direction.RIGHT, [])
        >>> print(path)
        [(0, 0, 0, Direction.RIGHT), (0, 1, 1, Direction.RIGHT), ...]
    """
    positive_constraints, negative_constraints, additional_constraint, earliest_stopping_time = resolve_constraints(
        constraints
    )
    startx, starty = int(start[0]), int(start[1])
    endx, endy = int(end[0]), int(end[1])

    startNode = Node(
        startx,
        starty,
        t=0,
        direction=startdirection,
        g=0,
        h=Node.get_h(startx, starty, endx, endy),
    )

    open_list: List[Node] = [startNode]
    open_list_info: Dict[Tuple[int, int, int], Node] = {(startx, starty, 0): startNode}
    closed_list: set[Tuple[int, int, int]] = set()
    movements = [
        (-1, 0, Direction.UP),  # Move up
        (1, 0, Direction.DOWN),  # Move down
        (0, -1, Direction.LEFT),  # Move left
        (0, 1, Direction.RIGHT),  # Move right
        (0, 0, None),  # Stay in place
    ]

    iter = 0
    cur = startNode
    while (cur.x, cur.y) != (endx, endy) or cur.t <= earliest_stopping_time:
        if iter == max_iter:
            logging.warning("A* exceeded maximum iterations")
            return None

        cur = heapq.heappop(open_list)  # pop the node with the smallest f value

        logging.debug(f"Current node: {cur}")
        if not cur.valid:
            continue
        closed_list.add((cur.x, cur.y, cur.t))

        # Generate movement neighbors
        for dx, dy, new_direction in movements:
            new_x, new_y = cur.x + dx, cur.y + dy
            if new_direction is None:
                new_direction = cur.direction
            dt = Node.get_dt(new_direction, cur.direction)
            new_t = cur.t + dt
            if not mapdata.is_valid(new_x, new_y) or (new_x, new_y, new_t) in closed_list:
                continue

            new_h = Node.get_h(new_x, new_y, endx, endy)

            neighbor = Node(
                new_x,
                new_y,
                new_t,
                new_direction,
                cur.g + TEMPORAL_COST * dt,
                new_h,
                cur,
            )

            constraints_to_be_considered = []
            for t in range(cur.t, neighbor.t + 1):
                if t in negative_constraints:
                    constraints_to_be_considered.extend(negative_constraints[t])

            if not neighbor.violate_constraints(constraints_to_be_considered, endx, endy, earliest_stopping_time):
                logging.debug(f"neighbor: {neighbor}")
                if (neighbor.x, neighbor.y, neighbor.t) in open_list_info:
                    # If the neighbor is already in the open list, check if we found a better path
                    neighbor_old: Node = open_list_info[neighbor.x, neighbor.y, neighbor.t]
                    if neighbor.g < neighbor_old.g:
                        neighbor_old.valid = False
                        open_list_info[neighbor.x, neighbor.y, neighbor.t] = neighbor

                else:
                    open_list_info[neighbor.x, neighbor.y, neighbor.t] = neighbor
                    heapq.heappush(open_list, neighbor)
        logging.debug(f"Open list: {open_list}")
        logging.debug(f"Closed list: {closed_list}")
        logging.debug("----------------------------------------------------------")
        iter += 1

    path = []
    while cur:
        path.append((cur.x, cur.y, cur.t, cur.direction))
        cur = cur.father
    path.reverse()
    path = fill_path(path)

    # If additional constraints exist, add waiting steps at the end of the path where the AGV stays in place
    # (position and direction unchanged, only time advances)
    if additional_constraint:
        last_x, last_y, last_t, last_direction = path[-1]
        for i in range(1, additional_constraint["timestep"] + 1):
            path.append((last_x, last_y, last_t + i, last_direction))
    return path


if __name__ == "__main__":
    # Example usage
    log_file = Path(__file__).parent.parent.parent / "logs" / "astar.log"
    log = logging.basicConfig(
        filename=log_file,
        filemode="w",
        format="%(asctime)s | %(levelname)-8s | %(filename)s:%(lineno)d | %(message)s",
        datefmt="%Y-%m-%d-%H:%M:%S",
        level=logging.DEBUG,
    )
    import numpy as np

    mapdata = [[0, 0, 0], [0, 1, 0], [0, 0, 0]]  # Simple grid map
    map = Map(np.array(mapdata))
    start = (0, 0)  # Starting position
    end = (2, 2)  # Destination position
    path = astar(map, start, end, Direction.RIGHT)
    print(path)  # Output the path found by A* algorithm
