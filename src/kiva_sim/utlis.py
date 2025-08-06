from typing import List, Optional, Tuple
from kiva_sim.tables import Table, WorkCell


def manhattan_distance(coord1: Tuple[int, int], coord2: Tuple[int, int]) -> int:
    return abs(coord1[0] - coord2[0]) + abs(coord1[1] - coord2[1])


def get_available_workcells(tables: List[Table], table_id: Optional[int]) -> List[WorkCell]:
    if table_id is not None:
        table = tables[table_id]
        available_workcells = [work_cell for work_cell in table.work_cells if not work_cell.occupied]
    else:
        available_workcells = [
            work_cell for table in tables for work_cell in table.work_cells if not work_cell.occupied
        ]
    return available_workcells
