from __future__ import annotations
import heapq
import logging
from copy import deepcopy
from collections import defaultdict
from token import OP
from typing import List, Optional, Tuple, Dict, TypedDict
from kiva_sim.a_star import astar
from kiva_sim.maps import Map
from kiva_sim.states import Direction


class SimpleCollision(TypedDict):
    loc: List[Tuple[int, int]]
    timestep: int


class Collision(TypedDict):
    a1: int
    a2: int
    loc: List[Tuple[int, int]]
    timestep: int


def detect_collision(path1: List[Tuple], path2: List[Tuple]) -> Optional[SimpleCollision]:
    """Detects collision between two moving paths.

    Checks if two given paths intersect at any point in space and time, considering
    their positions and directions at each timestep.

    Args:
        path1 (list[tuple]): First path as a list of tuples, where each tuple contains
            (x_coord, y_coord, timestep, direction). Represents the object's position
            and movement state at each timestep.
        path2 (list[tuple]): Second path with same format as path1.

    Returns:
        dict or None: If collision occurs, returns dictionary with these fields:
            - 'loc': list[float] - [x, y] coordinates of collision location
            - 'timestep': int - Timestep when collision occurred
            Returns None if no collision is detected.
    """
    n = min(len(path1), len(path2))
    for t in range(1, n):
        # vertex collision
        if path1[t][:2] == path2[t][:2]:
            collision: SimpleCollision = {"loc": [path1[t][:2]], "timestep": t}
            return collision
        # edge collision
        if path1[t][:2] == path2[t - 1][:2] and path1[t - 1][:2] == path2[t][:2]:
            collision: SimpleCollision = {"loc": [path1[t - 1][:2], path1[t][:2]], "timestep": t}
            return collision
    return None


def resolve_collision(i, j, collision: SimpleCollision) -> Collision:
    return {"a1": i, "a2": j, "loc": collision["loc"], "timestep": collision["timestep"]}


def detect_collisions(paths: List[List[Tuple]]) -> List[Collision]:
    """Detects all collisions between multiple paths.
    Compares each pair of paths to find any collisions based on their positions and
    movements at each timestep.
    Args:
        paths (list[list[tuple]]): List of paths, where each path is a list of tuples
            representing (x_coord, y_coord, timestep, direction) at each timestep.
    Returns:
        list[dict]: List of detected collisions, where each collision is represented
            as a dictionary with fields:
            - 'a1': int - ID of first AGV involved in collision
            - 'a2': int - ID of second AGV involved in collision
            - 'loc': list[tuple] - Coordinates of collision location
            - 'timestep': int - Timestep when collision occurred
    """
    n = len(paths)
    collisions = [
        resolve_collision(i, j, collision)
        for i in range(n - 1)
        for j in range(i + 1, n)
        if (collision := detect_collision(paths[i], paths[j]))
    ]
    return collisions


def standard_splitting(collision: Collision) -> List[Dict]:
    """Splits a collision into two constraints (one per AGV).

    Args:
        collision (dict): Collision details with keys:
            a1 (int): First AGV ID.
            a2 (int): Second AGV ID.
            loc (list[tuple]): Collision coordinates - single tuple for vertex collision,
                two tuples for edge collision.
            timestep (int): Timestep when collision occurs (post-swap for edge collisions).

    Returns:
        list: Two constraints (one per AGV) to resolve the collision.
    """
    loc = collision["loc"]
    loc2 = loc.copy() if len(loc) == 1 else loc[::-1]
    cons = [
        {"agent": collision["a1"], "loc": loc, "timestep": collision["timestep"], "type": "negative"},
        {"agent": collision["a2"], "loc": loc2, "timestep": collision["timestep"], "type": "negative"},
    ]
    return cons


class CbsNode:
    def __init__(
        self,
        id: int,
        constraints: Dict[str, List[Dict]],
        paths: List[List[Tuple]],
    ):
        self.id = id
        self.constraints = constraints
        self.paths = paths
        self._collosions: Optional[List[Collision]] = None

    @property
    def cost(self) -> int:
        """Calculate the cost of the node based on the paths."""
        return sum(len(path) - 1 for path in self.paths)

    @property
    def collisions(self) -> List[Collision]:
        """Get the collisions of the node."""
        if self._collosions is None:
            self._collosions = detect_collisions(self.paths)
        return self._collosions

    def __lt__(self, other: CbsNode) -> bool:
        # 理论上为取最优解应以cost为key，这里用碰撞数作为key是为了在短时间内优选算出可行解
        return (len(self.collisions), self.cost, self.id) < (len(other.collisions), other.cost, other.id)

    def generate_root_paths(
        self,
        root_paths: Dict[int, List[Tuple[int, int, int, Direction]]],
        arrived_at_start,
        maps: List[Map],
        starts: List[Tuple[int, int]],
        ends: List[Tuple[int, int]],
        directions: List[Direction],
        astar_max_iter: int = 1500,
    ) -> None:
        """Generate initial paths for the CBS node."""
        n = len(starts)
        for i in range(n):
            # 如果AGV i当前已有路径
            # 如果当前时刻有AGV刚刚回到起点，则地图上该AGV的起点变为不可通行，需要检查所保留的路径是否和该起点冲突
            if i in root_paths and (not arrived_at_start or all(maps[i][step[:2]] == 0 for step in root_paths[i])):
                path = root_paths[i]
            else:
                path = astar(maps[i], starts[i], ends[i], directions[i], self.constraints[str(i)], astar_max_iter)
            if not path:
                raise BaseException(f"No solution for AGV {i}")
            self.paths.append(path)


def cbs_reserve(
    maps: List[Map],
    arrived_at_start: bool,
    root_paths: Dict[int, List[Tuple[int, int, int, Direction]]],
    starts: List[Tuple[int, int]],
    ends: List[Tuple[int, int]],
    directions: List[Direction],
    constraints: List[Dict],
    max_iter: int = 1000,
    astar_max_iter: int = 1500,
) -> Optional[List[List[Tuple[int, int, int, Direction]]]]:
    """Resolves conflicts and returns collision-free paths using CBS algorithm.

    Implements Conflict-Based Search (CBS) to find paths for multiple AGVs while avoiding:
    - Obstacles (non-zero map values)
    - Inter-agent collisions
    - User-specified constraints

    Args:
        maps: List of 2D arrays where 0=traversable, non-zero=obstacle (per-AGV customized)
        arrived_at_start: Whether any AGV just returned to start (blocks its start point)
        root_paths: Existing partial paths to preserve [(x, y, t, direction), ...] per AGV
        starts: List of (x, y) start coordinates per AGV
        ends: List of (x, y) goal coordinates per AGV
        directions: Initial heading directions per AGV
        constraints: Additional movement constraints (e.g., shelf停留 requirements)
        max_iter: Maximum algorithm iterations before timeout

    Returns:
        List of collision-free paths if solution found within max_iter, else None.
        Each path is a list of (x, y, timestep, direction) tuples.

    Raises:
        BaseException: If any AGV has no valid initial path
    """
    constraint_table = defaultdict(list)
    for constraint in constraints:
        agent = constraint["agent"]
        constraint_table[agent].append(constraint)

    node_id: int = 0
    root: CbsNode = CbsNode(id=node_id, constraints=constraint_table, paths=[])
    root.generate_root_paths(root_paths, arrived_at_start, maps, starts, ends, directions, astar_max_iter)
    open_list: List[CbsNode] = [root]  # open list in cbs algorithm (list of nodes to be considered)
    node_id += 1  # increment node id for the next node

    # main loop of cbs algorithm
    iter = 0
    while open_list and iter < max_iter:
        node = heapq.heappop(open_list)
        logging.info(f"cbs iteration {iter}")
        logging.info(f"current node collisions:\n{node.collisions}")
        if not node.collisions:
            logging.info(f"solution found in iteration {iter}")
            return node.paths
        constraints = standard_splitting(node.collisions[0])
        for constraint in constraints:
            another_node = CbsNode(id=node_id, constraints={}, paths=[])
            another_node.constraints = deepcopy(node.constraints)
            agent = constraint["agent"]
            # todo: modify the data structure of constraints list to achieve O(1) searching
            if constraint not in another_node.constraints[agent]:
                another_node.constraints[agent].append(constraint)
            another_node.paths = deepcopy(node.paths)
            logging.info(f"another_node.constraints[agent]: {another_node.constraints[agent]}")
            path = astar(
                maps[agent],
                starts[agent],
                ends[agent],
                directions[agent],
                another_node.constraints[agent],
                astar_max_iter,
            )
            if path:
                another_node.paths[agent] = path
                heapq.heappush(open_list, another_node)
                node_id += 1
        logging.info("-" * 20)
        iter += 1
    logging.warning("cbs no solution")
    return None
