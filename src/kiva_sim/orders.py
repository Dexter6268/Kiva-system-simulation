import os
import logging
import numpy as np
from pathlib import Path
from typing import List, Tuple, Optional
from kiva_sim.states import ShelfStatus
from kiva_sim.maps import MAP, Shelf
from kiva_sim.tables import Table
from kiva_sim.agv import AGV, AgvStatus, DeliveryMission

root_path = Path(__file__).parent.parent


class Order:
    def __init__(self, id: int, shelves: List[Shelf]):
        """
        Args:
            id (int): order id
            shelves (List[Shelf]): list of shelves
        """
        self.id = id
        self.shelves = shelves
        self.table: Optional[int] = None

    def __repr__(self):
        return f"Order(id={self.id}, shelves={self.shelves}), table={self.table})"


def init_orders(shelves: List[Shelf], order_num: int = int(np.random.normal(50, 5, 1)[0])) -> List[Order]:
    """Generate a list of random orders for the warehouse simulation.

    Args:
        seed: Random seed for reproducible order generation.
        order_num: Number of orders to generate. If None, defaults to a value
            sampled from a normal distribution with mean=50 and std=5.

    Returns:
        A list of Order objects, each containing randomly assigned shelves
        with TODO status.

    Note:
        Each order contains 1-4 shelves (uniform distribution) selected
        without replacement from available shelves.
    """
    orders = []
    shelf_num = len(shelves)
    for i in range(order_num):
        order_shelf_num = np.random.randint(1, 5)  # 1-4 shelves per order
        shelf_ids = np.random.choice(shelf_num, order_shelf_num, replace=False).tolist()
        shelves_subset = [shelves[shelf_id] for shelf_id in shelf_ids if shelves[shelf_id].status == ShelfStatus.TODO]
        orders.append(Order(i, shelves_subset))
    return orders


def manhattan_distance(coord1: Tuple[int, int], coord2: Tuple[int, int]) -> int:
    return abs(coord1[0] - coord2[0]) + abs(coord1[1] - coord2[1])


def find_nearest_available_vehicle(vehicles: List[AGV], shelf: Shelf) -> Optional[AGV]:
    """Find the nearest available vehicle to the given shelf."""
    available_vehicles = [v for v in vehicles if v.status == AgvStatus.AVAILABLE]
    if not available_vehicles:
        return None

    return min(
        available_vehicles,
        key=lambda vehicle: manhattan_distance((vehicle.x, vehicle.y), shelf.loc),
    )


def orderDistribute(order: Order, vehicles: List[AGV], shelves: List[Shelf], tabls: List[Table]) -> None:
    """Distributes order shelves to available AGV vehicles for delivery.
    Assigns each shelf in the order to the nearest available AGV. If the AGV
    previously handled the same shelf and is not charging, it directly assigns
    a work cell. Otherwise, the AGV is sent to collect the shelf first.
    Args:
        order: The order containing shelves to be distributed.
        vehicles: List of AGV vehicles available for assignment.
        shelves: List of all shelves in the system.
        tabls: List of tables containing work cells for order processing.
    """

    for i, shelf in enumerate(order.shelves):
        # 如果该货架尚未被分配且仍然在原位（没有被其它AGV运走）
        if shelf.status == ShelfStatus.TODO:
            nearest_vehicle = find_nearest_available_vehicle(vehicles, shelf)
            if nearest_vehicle is None:
                continue

            if (
                nearest_vehicle.delivery_missions
                and nearest_vehicle.delivery_missions[-1].shelf == shelf
                and nearest_vehicle.color_list[-1] == "y"
            ):  # 如果这辆车分配的货架和刚完成的一单一样且不是刚充完电
                logging.info(f"vehicle {nearest_vehicle.id} assigned to {shelf} again")

                available_workcells = [
                    work_cell
                    for table in tabls
                    for work_cell in table.work_cells
                    if not work_cell.occupied and (order.table is None or order.table == table.id)
                ]
                if available_workcells:
                    target_work_cell = min(
                        available_workcells,
                        key=lambda work_cell: manhattan_distance(
                            (nearest_vehicle.x, nearest_vehicle.y), work_cell.loc  # type: ignore
                        ),
                    )
                    # 分拣时间服从均值为10，标准差为2的正态分布
                    tsort = max(1, int(np.random.normal(10, 2, 1)[0]))
                    new_mission = DeliveryMission(
                        order_id=order.id,
                        sub_order_id=i,
                        shelf=shelf,
                        work_cell=target_work_cell,
                        tsort=tsort,
                    )
                    nearest_vehicle.delivery_missions.append(new_mission)
                    order.table = target_work_cell.table_id
                    target_work_cell.occupied = True
                    nearest_vehicle.status = AgvStatus.TO_SELECT
                    shelf.status = ShelfStatus.DOING
                    shelf.inplace = False
                    logging.info(f"vehicle {nearest_vehicle.id} going for {target_work_cell}")
                else:
                    nearest_vehicle.status = AgvStatus.WAITING_TO_SELECT
                    nearest_vehicle.delivery_missions[-1].shelf.inplace = False
                    nearest_vehicle.point = len(nearest_vehicle.path) - 1
                    logging.info(f"vehicle {nearest_vehicle.id} waiting to select")
            else:
                nearest_vehicle.status = AgvStatus.TO_SHELF
                # 分拣时间服从均值为10，标准差为2的正态分布
                tsort = max(1, int(np.random.normal(10, 2, 1)[0]))
                new_mission = DeliveryMission(
                    order_id=order.id,
                    sub_order_id=i,
                    shelf=shelf,
                    work_cell=None,
                    tsort=tsort,
                )
                nearest_vehicle.delivery_missions.append(new_mission)
                logging.info(f"vehicle {nearest_vehicle.id} assigned {shelf}")
                shelf.status = ShelfStatus.DOING
                shelf.inplace = False


if __name__ == "__main__":
    log = logging.basicConfig(
        # filename=log_file,
        # filemode="w",
        format="%(asctime)s | %(levelname)-8s | %(filename)s:%(lineno)d | %(message)s",
        datefmt="%Y-%m-%d-%H:%M:%S",
        level=logging.DEBUG,
    )
