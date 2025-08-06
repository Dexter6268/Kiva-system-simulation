from dataclasses import dataclass
from typing import List, Optional
from kiva_sim.states import OrderStatus


@dataclass
class SubOrder:
    id: int
    shelf_id: int
    status: OrderStatus = OrderStatus.TODO
    table_id: Optional[int] = None
