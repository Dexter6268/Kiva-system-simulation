from __future__ import annotations
from copy import deepcopy
import os
import logging
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, TYPE_CHECKING
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
    tsort: Optional[int] = max(1, int(np.random.normal(10, 2, 1)[0]))


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
    def __init__(self, id: int, x: int, y: int, direction: Direction, battery):
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
        self.x = x
        self.y = y
        self.direction = direction
        self.battery = battery
        self.start = (x, y)
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
            AgvStatus.WAITING_AT_START,
            AgvStatus.WAITING_TO_SELECT,
            AgvStatus.SELECTING,
            AgvStatus.CHARGING,
        ]:
            self.point += 1
            self.x, self.y, _, self.direction = self.path[self.point]
            self.color = self.color_list[self.point]
            if self.path[self.point - 1][:2] == self.path[self.point][:2]:
                self.battery -= BATTERY_CONSUMING_SPEED * 2  # 转弯时耗电是匀速前进时的2倍
            else:
                self.battery -= BATTERY_CONSUMING_SPEED
        self.battery = max(self.battery, 0)

    def check_battery(self, map: Map):
        """check if the AGV has enough battery to complete a mission"""
        return self.battery >= (map.shape[0] + map.shape[1]) * 6 * BATTERY_CONSUMING_SPEED

    def charge(self):
        self.battery += CHARGING_SPEED
        self.battery = min(self.battery, FULL_CHARGE)

    def goto_charge(self, available_stations: list[ChargingStation], charging_stations: list[ChargingStation]) -> None:

        shelf = self.delivery_missions[-1].shelf
        shelf.agv = None
        self.status = AgvStatus.TO_CHARGE
        logging.info(f"vehicle {self.id} to charge")
        target_station = min(
            available_stations,
            key=lambda s: manhattan_distance(self.loc, s.loc),
        )
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
    def loc(self):
        return (self.x, self.y)

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

    # def updates_status(
    #     self, tables: List[Table], orders, GLOBAL_AGV_MAP, charging_stations: List[ChargingStation]
    # ) -> None:
    #     """Update the AGV's status."""
    #     last_delivery_mission = self.delivery_missions[-1]
    #     if self.status == AgvStatus.TO_SHELF:
    #         logging.info(f"vehicle {self.id} reached {last_delivery_mission.shelf} for the first time")
    #         available_workcells = get_available_workcells(tables, orders[last_delivery_mission.order_id])
    #         if available_workcells:
    #             target_work_cell = get_target_workcell(available_workcells, self)
    #             orders[last_delivery_mission.order_id].table_id = target_work_cell.table_id
    #             last_delivery_mission.work_cell = target_work_cell

    #             target_work_cell.occupied = True
    #             self.status = AgvStatus.TO_SELECT
    #             logging.info(f"vehicle {self.id} going for {target_work_cell}")
    #             self.point = 0
    #         else:
    #             self.status = AgvStatus.WAITING_TO_SELECT
    #             logging.info(f"vehicle {self.id} waiting to select at position {self.loc}")
    #     elif self.status == AgvStatus.TO_SELECT:
    #         logging.info(f"vehicle {self.id} reached {last_delivery_mission.work_cell}")
    #         self.status = AgvStatus.SELECTING
    #         logging.info(f"vehicle {self.id} start selecting at position {self.loc}")
    #     elif self.status == AgvStatus.RETURN_SHELF:
    #         logging.info(f"vehicle {self.id} reached {last_delivery_mission.shelf} for the second time")
    #         if self.check_battery(MAP):
    #             # 更新货架在位情况
    #             last_delivery_mission.shelf.agv = None
    #             self.status = AgvStatus.AVAILABLE
    #             self.point = 0
    #         else:
    #             available_stations = [station for station in charging_stations if not station.occupied]
    #             if available_stations:
    #                 self.goto_charge(available_stations, charging_stations)
    #             else:
    #                 self.status = AgvStatus.WAITING_TO_CHARGE
    #                 logging.info(f"vehicle {self.id} waiting to charge")
    #     elif self.status == AgvStatus.TO_CHARGE:
    #         self.status = AgvStatus.CHARGING
    #         logging.info(f"vehicle {self.id} start charging at position {self.loc}")
    #     elif self.status == AgvStatus.SELECTING:
    #         self.selecting_process += 1
    #         assert isinstance(last_delivery_mission.tsort, int)
    #         if self.selecting_process >= last_delivery_mission.tsort:
    #             logging.info(f"vehicle {self.id} finished selecting at position f{self.loc}")
    #             self.status = AgvStatus.RETURN_SHELF
    #             logging.info(f"vehicle {self.id} returning {last_delivery_mission.shelf}")
    #             revenue += last_delivery_mission.tsort  # 结算分拣收益
    #             assert last_delivery_mission.work_cell is not None
    #             last_delivery_mission.work_cell.occupied = False
    #             last_delivery_mission.sub_order.status = OrderStatus.DONE
    #             self.selecting_process = 0
    #             self.point = 0
    #     elif self.status == AgvStatus.CHARGING:
    #         self.charge()
    #         if self.battery == FULL_CHARGE:
    #             logging.info(f"vehicle {self.id} finished charging at position {self.loc}")
    #             self.status = AgvStatus.AVAILABLE
    #             charging_stations[self.charging_missions[-1].charging_station_id].occupied = False
    #             self.point = 0
    #     elif self.status == AgvStatus.WAITING_TO_SELECT:
    #         available_workcells = get_available_workcells(tables, orders[last_delivery_mission.order_id])
    #         if available_workcells:
    #             target_work_cell = get_target_workcell(available_workcells, self)
    #             orders[last_delivery_mission.order_id].table_id = target_work_cell.table_id
    #             last_delivery_mission.work_cell = target_work_cell
    #             target_work_cell.occupied = True
    #             self.status = AgvStatus.TO_SELECT
    #             logging.info(f"vehicle {self.id} to select at {target_work_cell}")
    #             self.point = 0
    #         else:
    #             logging.info(f"vehicle {self.id} waiting to select at position {self.loc}")
    #     elif self.status == AgvStatus.WAITING_TO_CHARGE:
    #         logging.info(f"vehicle {self.id} waiting to charge at position {self.loc}")
    #         available_stations = [station for station in charging_stations if not station.occupied]
    #         if available_stations:
    #             self.goto_charge(available_stations, charging_stations)
    #     elif self.status == AgvStatus.BACK_TO_START:
    #         GLOBAL_AGV_MAP[self.start] = 4
    #         num_idle_vehicles = sum(agv.status == AgvStatus.WAITING_AT_START for agv in vehicles)
    #         if num_idle_vehicles < agv_num - 1:
    #             logging.info(f"vehicle {self.id} arrived at start")
    #             self.status = AgvStatus.ARRIVED_AT_START
    #             self.point = 0
    #         else:
    #             self.status = AgvStatus.WAITING_AT_START


def get_target_workcell(available_workcells: List[WorkCell], nearest_vehicle: AGV) -> WorkCell:
    return min(available_workcells, key=lambda work_cell: manhattan_distance(nearest_vehicle.loc, work_cell.loc))


def init_agvs(agv_num: int, map: Map) -> List[AGV]:

    vehicles = [
        AGV(
            id=i,
            x=0,
            y=(i + 1) * (map.shape[1] // (agv_num + 1)),
            direction=Direction.DOWN,
            battery=FULL_CHARGE,
        )
        for i in range(agv_num)
    ]
    return vehicles


if __name__ == "__main__":
    pass
