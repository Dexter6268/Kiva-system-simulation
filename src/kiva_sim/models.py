from dataclasses import dataclass
from typing import Tuple, Optional
from kiva_sim.states import OrderStatus


@dataclass
class SubOrder:
    id: int
    shelf_id: int
    status: OrderStatus = OrderStatus.TODO
    table_id: Optional[int] = None


@dataclass
class ChargingStation:
    id: int
    loc: Tuple[int, int]
    occupied: bool = False
