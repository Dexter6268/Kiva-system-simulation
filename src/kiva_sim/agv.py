from __future__ import annotations
import os
import logging
import numpy as np
from copy import deepcopy
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from kiva_sim.states import AgvStatus, Direction, OrderStatus
from kiva_sim.maps import Map
from kiva_sim.tables import Table
from kiva_sim.utlis import manhattan_distance, get_available_workcells
from kiva_sim.tables import WorkCell
from kiva_sim.models import ChargingStation, SubOrder

FULL_CHARGE = int(os.getenv("FULL_CHARGE", "3600"))
BATTERY_CONSUMING_SPEED = int(os.getenv("BATTERY_CONSUMING_SPEED", "1"))
CHARGING_SPEED = 6 * BATTERY_CONSUMING_SPEED


@dataclass
class DeliveryMission:
    """Represents a delivery mission assigned to an AGV."""

    order_id: int
    sub_order: SubOrder
    shelf: Shelf
    work_cell: Optional[WorkCell] = None
    # 分拣时间服从均值为10，标准差为2的正态分布
    tsort: Optional[int] = field(default_factory=lambda: max(1, int(np.random.normal(10, 2, 1)[0])))


@dataclass
class ChargingMission:
    """Represents a charging mission."""

    charging_station_id: int
    loc: Tuple[int, int]


class Shelf:
    def __init__(self, id: int, loc: Tuple[int, int]):
        """
        Args:
            id (int): shelf id
            status (ShelfStatus): initial status of the shelf
        """
        self.id = id
        self.loc = loc
        self.agv: Optional[AGV] = None

    def __repr__(self):
        return f"Shelf(id={self.id}, available={self.available}, original_loc={self.loc}, cur_loc={self.cur_loc}"

    @property
    def cur_loc(self) -> Tuple[int, int]:
        """Current location of the shelf."""
        if not self.agv:
            return self.loc
        return self.agv.loc

    @property
    def available(self) -> bool:
        """Check if the shelf is available."""
        return self.agv is None


class AGV:
    def __init__(self, id: int, loc: Tuple[int, int], direction: Direction):
        """
        :param id:
        :param x: int，AGV当前的x坐标（位于栅格地图的第几行）
        :param y: int，AGV当前的y坐标（位于栅格地图的第几列）
        :param direction: string，AGV当前的朝向，'up', 'down', 'right, 'down' 中的一个
        :param status: AgvStatus，AGV当前的状态
        :param orders: list of dicts，订单列表，字段如下：
            'order_id': int，订单号
            'sub_order_id': int，该订单的第几个货架
            'shelf_id': int，货架编号
            'table_id': int，工作台编号
            'table_position': tuple，在目标工作台附近停留的位置（工作台坐标的上、左、右相邻坐标中的一个）
            'tsort'：int，分拣时间
        :param charge_mission: list of dict， 充电任务列表，字段如下：
            'id': int，充电桩id
            'loc': tuple，充电桩位置
        :param color: string，AGV当前的颜色（用来在仿真中表征AGV是否正搬运货架，如果在搬运在为黄色'y'，否则为黑色'k'）
        :param path: list of tuples，AGV当前的路径
        :param color_list: list of strings，AGV的颜色列表（对应当前路径）
        """
        self.id = id
        self.loc = loc
        self.direction = direction
        self.start = loc
        self.battery = FULL_CHARGE
        self.status = AgvStatus.AVAILABLE
        self.delivery_missions: List[DeliveryMission] = []
        self.charging_missions: List[ChargingMission] = []
        self.point = 0
        self.color = "k"
        self.path: List[Tuple[int, int, int, Direction]] = []
        self.color_list = []
        self.selecting_process = 0

    def move(self):
        """
        Move the AGV to the next point in its path.
        """
        if self.point < len(self.path) - 1 and self.status not in [
            AgvStatus.WAITING_TO_CHARGE,
            AgvStatus.WAITING_TO_SELECT,
            AgvStatus.SELECTING,
            AgvStatus.CHARGING,
        ]:
            self.point += 1
            x, y, _, self.direction = self.path[self.point]
            self.loc = (x, y)
            self.color = self.color_list[self.point]
            turning_coef = 2 if self.path[self.point - 1][:2] == self.path[self.point][:2] else 1
            self.battery -= BATTERY_CONSUMING_SPEED * turning_coef  # 转弯时耗电是匀速前进时的2倍
        self.battery = max(self.battery, 0)

    def check_battery(self, map_shape: Tuple) -> bool:
        """check if the AGV has enough battery to complete a mission"""
        return self.battery >= (map_shape[0] + map_shape[1]) * 6 * BATTERY_CONSUMING_SPEED

    def charge(self):
        self.battery += CHARGING_SPEED
        self.battery = min(self.battery, FULL_CHARGE)

    def goto_charge(self, available_stations: list[ChargingStation]) -> None:
        self.status = AgvStatus.TO_CHARGE
        logging.info(f"vehicle {self.id} to charge")
        self.delivery_missions[-1].shelf.agv = None
        target_station = min(available_stations, key=lambda s: manhattan_distance(self.loc, s.loc))
        self.charging_missions.append(ChargingMission(target_station.id, target_station.loc))
        target_station.occupied = True
        self.point = 0

    def assign_delivery_mission(self, mission: DeliveryMission):
        """Assign a delivery mission to the AGV."""
        self.delivery_missions.append(mission)
        mission.sub_order.status = OrderStatus.DOING
        mission.shelf.agv = self

    def __repr__(self):
        return (
            f"AGV(id={self.id}, loc={self.loc}, direction={self.direction}, "
            f"battery={self.battery}, status={self.status}, "
            f"delivery_missions={self.delivery_missions}, "
            f"charging_missions={self.charging_missions})\n"
        )

    @property
    def needs_path_renewal(self) -> bool:
        return (
            self.status
            not in [
                AgvStatus.AVAILABLE,
                AgvStatus.WAITING_TO_CHARGE,
                AgvStatus.WAITING_AT_START,
                AgvStatus.WAITING_TO_SELECT,
            ]
            and self.point == 0
        )

    def updates_map(self, map, coord_to_update) -> None:
        map = deepcopy(map)
        map[coord_to_update] = 0
        self.map = map
        self.end = coord_to_update

    def updates_path(self, new_path: List[Tuple[int, int, int, Direction]]) -> None:
        """Update the AGV's path."""
        self.point = 0
        self.path = new_path
        color = "y" if self.status in [AgvStatus.TO_SELECT, AgvStatus.RETURN_SHELF] else "k"
        self.color_list = [color] * len(new_path)

    def meta_updates(
        self,
        tables,
        orders,
        GLOBAL_AGV_MAP,
        charging_stations,
        is_last_to_return,
        allowed_to_return,
        revenue,
    ):
        num_suborders_unassigned = sum(
            sub_order.status == OrderStatus.TODO for order in orders for sub_order in order.sub_orders
        )
        # 如果AGV空闲且没有剩余的未指派订单，则令AGV返回起点
        if self.status == AgvStatus.AVAILABLE and num_suborders_unassigned == 0:
            if self.loc != self.start:
                # 分批返回起点（如果当前处于返程的AGV超过总数的一半则继续等待），防止一次性返回车数过多，造成拥堵s)
                if allowed_to_return:
                    logging.info(f"vehicle {self.id} back to start")
                    self.status = AgvStatus.BACK_TO_START
            else:
                self.status = AgvStatus.WAITING_AT_START
                GLOBAL_AGV_MAP[self.start] = 4
        elif self.status not in (AgvStatus.AVAILABLE, AgvStatus.WAITING_AT_START):
            self.move()
            # 当AGV完成一个阶段的任务，更新AGV对象状态参数
            if self.point == len(self.path) - 1:
                revenue = self.updates_status(
                    tables, orders, GLOBAL_AGV_MAP, charging_stations, is_last_to_return, revenue
                )

    def updates_status(
        self,
        tables: List[Table],
        orders,
        global_agv_map: Map,
        charging_stations: List[ChargingStation],
        is_last_to_return: bool,
        revenue: float,
    ) -> float:
        """Update the AGV's status."""

        status_config = {
            AgvStatus.TO_SHELF: (self._handle_to_shelf_status, [tables, orders, revenue]),
            AgvStatus.TO_SELECT: (self._handle_to_select_status, [revenue]),
            AgvStatus.RETURN_SHELF: (self._handle_return_shelf_status, [global_agv_map, charging_stations, revenue]),
            AgvStatus.TO_CHARGE: (self._handle_to_charge_status, [revenue]),
            AgvStatus.SELECTING: (self._handle_selecting_status, [revenue]),
            AgvStatus.CHARGING: (self._handle_charging_status, [charging_stations, revenue]),
            AgvStatus.WAITING_TO_SELECT: (self._handle_waiting_to_select_status, [tables, orders, revenue]),
            AgvStatus.WAITING_TO_CHARGE: (self._handle_waiting_to_charge_status, [charging_stations, revenue]),
            AgvStatus.BACK_TO_START: (self._handle_back_to_start_status, [global_agv_map, is_last_to_return, revenue]),
        }
        config = status_config.get(self.status)
        if config:
            handler, required_params = config
            return handler(*required_params)
        return revenue

    def _handle_to_shelf_status(self, tables, orders, revenue):
        last_delivery_mission = self.delivery_missions[-1]
        last_order = orders[last_delivery_mission.order_id]
        available_workcells = get_available_workcells(tables, last_order.table_id)
        logging.info(f"vehicle {self.id} reached {last_delivery_mission.shelf} for the first time")
        if available_workcells:
            target_work_cell = get_target_workcell(available_workcells, self)
            last_order.table_id = target_work_cell.table_id
            last_delivery_mission.work_cell = target_work_cell

            target_work_cell.occupied = True
            self.status = AgvStatus.TO_SELECT
            logging.info(f"vehicle {self.id} going for {target_work_cell}")
            self.point = 0
        else:
            self.status = AgvStatus.WAITING_TO_SELECT
            logging.info(f"vehicle {self.id} waiting to select at position {self.loc}")
        return revenue

    def _handle_to_select_status(self, revenue):
        self.status = AgvStatus.SELECTING
        logging.info(
            f"vehicle {self.id} has reached {self.delivery_missions[-1].work_cell} and starts selecting at position {self.loc}"
        )
        return revenue

    def _handle_return_shelf_status(self, global_agv_map, charging_stations, revenue):
        last_delivery_mission = self.delivery_missions[-1]
        logging.info(f"vehicle {self.id} reached {last_delivery_mission.shelf} for the second time")
        if self.check_battery(global_agv_map.shape):
            # 更新货架在位情况
            last_delivery_mission.shelf.agv = None
            self.status = AgvStatus.AVAILABLE
            self.point = 0
        else:
            available_stations = [station for station in charging_stations if not station.occupied]
            if available_stations:
                self.goto_charge(available_stations)
            else:
                self.status = AgvStatus.WAITING_TO_CHARGE
                logging.info(f"vehicle {self.id} waiting to charge")
        return revenue

    def _handle_to_charge_status(self, revenue):
        """Handle TO_CHARGE status transition."""
        self.status = AgvStatus.CHARGING
        logging.info(f"vehicle {self.id} start charging at position {self.loc}")
        return revenue

    def _handle_selecting_status(self, revenue):
        """Handle SELECTING status transition."""
        self.selecting_process += 1
        last_delivery_mission = self.delivery_missions[-1]
        assert last_delivery_mission.tsort is not None
        if self.selecting_process >= last_delivery_mission.tsort:
            logging.info(
                f"vehicle {self.id} has finished selecting at position f{self.loc}, now returning {last_delivery_mission.shelf}"
            )
            self.status = AgvStatus.RETURN_SHELF
            revenue += last_delivery_mission.tsort  # 结算分拣收益
            assert last_delivery_mission.work_cell is not None
            last_delivery_mission.work_cell.occupied = False
            last_delivery_mission.sub_order.status = OrderStatus.DONE
            self.selecting_process = 0
            self.point = 0
        return revenue

    def _handle_charging_status(self, charging_stations, revenue):
        """Handle CHARGING status transition."""
        self.charge()
        if self.battery == FULL_CHARGE:
            logging.info(f"vehicle {self.id} finished charging at position {self.loc}")
            self.status = AgvStatus.AVAILABLE
            charging_stations[self.charging_missions[-1].charging_station_id].occupied = False
            self.point = 0
        return revenue

    def _handle_waiting_to_select_status(self, tables, orders, revenue):
        """Handle WAITING_TO_SELECT status transition."""
        last_delivery_mission = self.delivery_missions[-1]
        last_order = orders[last_delivery_mission.order_id]
        available_workcells = get_available_workcells(tables, last_order.table_id)
        if not available_workcells:
            logging.info(f"vehicle {self.id} waiting to select at position {self.loc}")
            return revenue
        target_work_cell = get_target_workcell(available_workcells, self)
        last_order.table_id = target_work_cell.table_id
        last_delivery_mission.work_cell = target_work_cell
        target_work_cell.occupied = True
        self.status = AgvStatus.TO_SELECT
        logging.info(f"vehicle {self.id} to select at {target_work_cell}")
        self.point = 0
        return revenue

    def _handle_waiting_to_charge_status(self, charging_stations, revenue):
        """Handle WAITING_TO_CHARGE status transition."""
        logging.info(f"vehicle {self.id} waiting to charge at position {self.loc}")
        available_stations = [station for station in charging_stations if not station.occupied]
        if available_stations:
            self.goto_charge(available_stations)
        return revenue

    def _handle_back_to_start_status(self, global_agv_map, is_last_to_return, revenue):
        """Handle BACK_TO_START status transition."""
        global_agv_map[self.start] = 4
        if is_last_to_return:
            self.status = AgvStatus.WAITING_AT_START
            return revenue

        logging.info(f"vehicle {self.id} arrived at start")
        self.status = AgvStatus.ARRIVED_AT_START
        self.point = 0
        return revenue

    @property
    def target(self) -> str:
        """Get the target description for a vehicle based on its current status."""
        status_target_mapping = {
            AgvStatus.TO_SHELF: lambda v: f"shelf {v.delivery_missions[-1].shelf.id}",
            AgvStatus.RETURN_SHELF: lambda v: f"shelf {v.delivery_missions[-1].shelf.id}",
            AgvStatus.TO_SELECT: lambda v: f"table {v.delivery_missions[-1].work_cell.table_id}",
            AgvStatus.TO_CHARGE: lambda v: f"charging station {v.charging_missions[-1].charging_station_id}",
        }

        target_func = status_target_mapping.get(self.status)
        if target_func:
            try:
                return target_func(self)
            except (IndexError, AttributeError):
                logging.warning(f"Failed to get target for vehicle {self.id} with status {self.status}")
                return "None"

        return "None"


def get_target_workcell(available_workcells: List[WorkCell], nearest_vehicle: AGV) -> WorkCell:
    return min(available_workcells, key=lambda work_cell: manhattan_distance(nearest_vehicle.loc, work_cell.loc))


def init_agvs(agv_num: int, map: Map) -> List[AGV]:
    dy = map.shape[1] // (agv_num + 1)
    return [AGV(id=i, loc=(0, (i + 1) * dy), direction=Direction.DOWN) for i in range(agv_num)]


if __name__ == "__main__":
    sub_order = SubOrder(id=1, shelf_id=1)
    shelf = Shelf(id=1, loc=(0, 0))
    mission1 = DeliveryMission(1, sub_order, shelf)
    import time

    time.sleep(1)  # 模拟分拣时间
    mission2 = DeliveryMission(2, sub_order, shelf)
    mission3 = DeliveryMission(3, sub_order, shelf)
    print(f"Mission 1 tsort: {mission1.tsort}")  # 例如: 8
    print(f"Mission 2 tsort: {mission2.tsort}")  # 例如: 12
    print(f"Mission 3 tsort: {mission3.tsort}")  # 例如: 9
