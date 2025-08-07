import os
import numpy as np
import pandas as pd
import logging
from pathlib import Path
from typing import List, Tuple, cast, Optional

try:
    from rich import print
except ImportError:
    ...


class Map:
    """
    Class to represent the warehouse map.
    The map is read from an Excel file and contains information about shelves, tables, and charging stations.
    """

    def __init__(self, value: np.ndarray):
        self.data = value
        self._shelf_coords = None
        self._table_coords = None
        self._charging_station_coords = None

    def is_valid(self, x: int, y: int) -> bool:
        """
        Check if the coordinates (x, y) are valid within the map and not occupied.
        """
        return 0 <= x < self.data.shape[0] and 0 <= y < self.data.shape[1] and self.data[x, y] == 0

    def __getitem__(self, key):
        """supports map[x, y] or map[key] access"""
        if isinstance(key, tuple) and len(key) == 2:
            x, y = key
            return self.data[x, y]
        else:
            return self.data[key]

    def __setitem__(self, key, value):
        """supports map[x, y] = value or map[key] = value access"""
        if isinstance(key, tuple) and len(key) == 2:
            x, y = key
            self.data[x, y] = value
        else:
            self.data[key] = value

    @property
    def shape(self) -> Tuple[int, int]:
        """Return the shape of the map."""
        return cast(Tuple[int, int], self.data.shape)

    def _get_coords_by_value(self, value: int) -> List[Tuple[int, int]]:
        return [(int(x), int(y)) for x, y in np.argwhere(self.data == value)]

    @property
    def shelf_coords(self) -> List[Tuple[int, int]]:
        """Get coordinates of all shelves (cached)."""
        if self._shelf_coords is None:
            self._shelf_coords = self._get_coords_by_value(1)
        return self._shelf_coords

    @property
    def table_coords(self) -> List[Tuple[int, int]]:
        """Get coordinates of all tables (cached)."""
        if self._table_coords is None:
            self._table_coords = self._get_coords_by_value(2)
        return self._table_coords

    @property
    def charging_station_coords(self) -> List[Tuple[int, int]]:
        """Get coordinates of all charging stations (cached)."""
        if self._charging_station_coords is None:
            self._charging_station_coords = self._get_coords_by_value(3)
        return self._charging_station_coords


def load_map(map_name: Optional[str] = None):
    """Load map with proper logging."""
    map_folder = Path(__file__).resolve().parent / "maps"
    map_folder.mkdir(parents=True, exist_ok=True)
    map_name = map_name or os.environ.get("MAP_NAME", "map0.xlsx")
    map_path = map_folder / map_name
    logging.info(f"Loading map from {map_path}")
    df = pd.read_excel(map_path).fillna(0)
    return Map(df.iloc[0:-1, 1:-1].values)
