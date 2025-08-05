from typing import List
from kiva_sim.maps import MAP


class WorkCell:
    def __init__(self, id: int, table_id: int, loc: tuple[int, int]):
        self.id = id
        self.table_id = table_id  # 属于哪个工作台
        self.loc = loc
        self.occupied = False

    def __repr__(self):
        return f"WorkCell(id={self.id}, table_id={self.table_id}, loc={self.loc}, occupied={self.occupied})"


class Table:
    def __init__(self, id: int, loc: tuple[int, int]):
        self.id = id
        self.loc = loc
        self.work_cells: List[WorkCell] = []

    def __repr__(self):
        return f"Table(id={self.id}, loc={self.loc}, occupied={self.occupied})"

    @property
    def occupied(self) -> bool:
        """Check if the table is occupied based on its work cells."""
        return all(cell.occupied for cell in self.work_cells)


def init_tables(table_num) -> List[Table]:
    tables = []  # 工作台列表
    table_coords = MAP.table_coords
    for i in range(table_num):
        table = Table(id=i, loc=table_coords[i])
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        work_cell_id = 0
        for dx, dy in directions:
            x, y = table.loc[0] + dx, table.loc[1] + dy
            if MAP.is_valid(x, y):
                cell = WorkCell(id=work_cell_id, table_id=i, loc=(x, y))
                table.work_cells.append(cell)
                work_cell_id += 1
        tables.append(table)
    return tables
