from typing import List, Tuple, Optional


def manhattan_distance(coord1: Tuple[int, int], coord2: Tuple[int, int]) -> int:
    return abs(coord1[0] - coord2[0]) + abs(coord1[1] - coord2[1])
