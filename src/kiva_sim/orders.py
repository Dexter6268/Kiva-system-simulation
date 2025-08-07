import logging
import numpy as np
from typing import List, Optional
from kiva_sim.states import AgvStatus, OrderStatus
from kiva_sim.tables import Table
from kiva_sim.utlis import manhattan_distance, get_available_workcells
from kiva_sim.agv import AGV, Shelf, DeliveryMission, get_target_workcell
from kiva_sim.models import SubOrder


class Order:
    def __init__(self, id: int, sub_orders: List[SubOrder]):
        """
        Args:
            id (int): order id
            shelves (List[Shelf]): list of shelves
        """
        self.id = id
        self.sub_orders = sub_orders
        self.table_id: Optional[int] = None

    def __repr__(self):
        return f"Order(id={self.id}, table={self.table_id})"

    @property
    def status(self) -> OrderStatus:
        """Returns the status of the order based on the status of its shelves."""
        if all(sub_order.status == OrderStatus.DONE for sub_order in self.sub_orders):
            return OrderStatus.DONE
        elif all(sub_order.status == OrderStatus.TODO for sub_order in self.sub_orders):
            return OrderStatus.TODO
        else:
            return OrderStatus.DOING


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
        logging.info(f"Generating order {i} with {order_shelf_num} shelves")
        shelf_ids = np.random.choice(shelf_num, order_shelf_num, replace=False).tolist()
        logging.info(f"Order {i} shelf IDs: {shelf_ids}")
        # todo consider the case that some shelf isn't available
        sub_orders = [SubOrder(j, shelf_id) for j, shelf_id in enumerate(shelf_ids)]
        orders.append(Order(i, sub_orders))
    return orders


def find_nearest_available_vehicle(vehicles: List[AGV], shelf: Shelf) -> Optional[AGV]:
    """Find the nearest available vehicle to the given shelf."""
    available_vehicles = [v for v in vehicles if v.status == AgvStatus.AVAILABLE]
    if not available_vehicles:
        return None

    return min(
        available_vehicles,
        key=lambda vehicle: manhattan_distance(vehicle.loc, shelf.loc),
    )


def distribute_order(order: Order, vehicles: List[AGV], shelves: List[Shelf], tabls: List[Table]) -> None:
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
    for sub_order in order.sub_orders:
        # 如果该货架尚未被分配且仍然在原位（没有被其它AGV运走）
        shelf = shelves[sub_order.shelf_id]
        if sub_order.status == OrderStatus.TODO and shelf.available:
            nearest_vehicle = find_nearest_available_vehicle(vehicles, shelf)
            if nearest_vehicle is None:
                continue

            if (
                nearest_vehicle.delivery_missions
                and nearest_vehicle.delivery_missions[-1].shelf.id == shelf.id
                and nearest_vehicle.color_list[-1] == "y"
            ):  # 如果这辆车分配的货架和刚完成的一单一样且不是刚充完电
                logging.info(f"vehicle {nearest_vehicle.id} assigned to {shelf} again")
                available_workcells = get_available_workcells(tabls, order.table_id)
                if available_workcells:
                    target_work_cell = get_target_workcell(available_workcells, nearest_vehicle)
                    new_mission = DeliveryMission(order.id, sub_order, shelf, target_work_cell)
                    logging.info(f"new mission tsort: {new_mission.tsort}")
                    nearest_vehicle.assign_delivery_mission(new_mission)
                    order.table_id = target_work_cell.table_id
                    target_work_cell.occupied = True
                    nearest_vehicle.status = AgvStatus.TO_SELECT
                    logging.info(f"vehicle {nearest_vehicle.id} going for {target_work_cell}")
                else:
                    nearest_vehicle.status = AgvStatus.WAITING_TO_SELECT
                    nearest_vehicle.delivery_missions[-1].shelf.agv = nearest_vehicle
                    nearest_vehicle.point = len(nearest_vehicle.path) - 1
                    logging.info(f"vehicle {nearest_vehicle.id} waiting to select")
            else:
                nearest_vehicle.status = AgvStatus.TO_SHELF
                new_mission = DeliveryMission(order.id, sub_order, shelf)
                logging.info(f"new mission tsort: {new_mission.tsort}")
                nearest_vehicle.assign_delivery_mission(new_mission)
                logging.info(f"vehicle {nearest_vehicle.id} assigned {shelf}")


if __name__ == "__main__":
    log = logging.basicConfig(
        # filename=log_file,
        # filemode="w",
        format="%(asctime)s | %(levelname)-8s | %(filename)s:%(lineno)d | %(message)s",
        datefmt="%Y-%m-%d-%H:%M:%S",
        level=logging.DEBUG,
    )
    a = np.random.normal(50, 5, 1)
    print(a)
