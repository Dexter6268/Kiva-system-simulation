from __future__ import annotations
import os
import logging
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, TYPE_CHECKING
from kiva_sim.states import AgvStatus, Direction, OrderStatus
from kiva_sim.maps import MAP
from kiva_sim.utlis import manhattan_distance
from kiva_sim.tables import WorkCell
from kiva_sim.models import SubOrder

FULL_CHARGE = int(os.getenv("FULL_CHARGE", "3600"))
BATTERY_CONSUMING_SPEED = int(os.getenv("BATTERY_CONSUMING_SPEED", "1"))
CHARGING_SPEED = 6 * BATTERY_CONSUMING_SPEED
SHELF_COORDS = MAP.shelf_coords


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
    def __init__(self, id: int):
        """
        Args:
            id (int): shelf id
            status (ShelfStatus): initial status of the shelf
        """
        self.id = id
        self.loc = SHELF_COORDS[id]
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

    def check_battery(self, MAP):
        """check if the AGV has enough battery to complete a mission"""
        return self.battery >= (MAP.shape[0] + MAP.shape[1]) * 6 * BATTERY_CONSUMING_SPEED

    def charge(self):
        self.battery += CHARGING_SPEED
        self.battery = min(self.battery, FULL_CHARGE)

    def goto_charge(self, available_stations: list[dict], charging_stations: list[dict]) -> None:

        shelf = self.delivery_missions[-1].shelf
        shelf.agv = None
        self.status = AgvStatus.TO_CHARGE
        logging.info(f"vehicle {self.id} to charge")
        target_station = min(
            available_stations,
            key=lambda s: manhattan_distance(self.loc, s["loc"]),
        )
        self.charging_missions.append(ChargingMission(target_station["id"], target_station["loc"]))
        charging_stations[target_station["id"]]["occupied"] = True
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

    def updates_path(self, new_path: List[Tuple[int, int, int, Direction]]) -> None:
        """Update the AGV's path."""
        self.point = 0
        self.path = new_path
        color = "y" if self.status in [AgvStatus.TO_SELECT, AgvStatus.RETURN_SHELF] else "k"
        self.color_list = [color] * len(new_path)

    # def renew_path(self, AGV_MAP: Map, LIFTING_TIME: int) -> None:
    # logging.info(f"vehicle {vehicle.id} triggered renewing")
    # additional_constraints: List[Dict] = []
    # # 发生路径更新时已经抵达目标点的AGV（在抬起或放下货架，或在工作台处分拣）
    # staying_vehicles: List[int] = []
    # # 发生路径更新时未抵达目标点的AGV
    # moving_vehicles: List[int] = []
    # moving_vehicle_id: int = 0
    # maps = []
    # starts: List[Tuple[int, int]] = []
    # ends = []
    # root_paths: Dict = {}
    # directions = []
    # arrived_at_start = any(vehicle.status == AgvStatus.ARRIVED_AT_START for vehicle in vehicles)

    # if vehicle.status == AgvStatus.TO_SHELF or vehicle.status == AgvStatus.RETURN_SHELF:
    #     # 如果更新路径时该AGV已经运行至目标货架处（正在抬起或放下货架）
    #     last_delivery_mission = vehicle.delivery_missions[-1]
    #     if last_delivery_mission.shelf.loc == vehicle.loc:
    #         staying_vehicles.append(vehicle.id)
    #     else:
    #         grid = deepcopy(AGV_MAP)
    #         # 将目标货架处设为可通行
    #         grid[last_delivery_mission.shelf.loc] = 0
    #         maps.append(grid)
    #         starts.append(vehicle.loc)
    #         ends.append(last_delivery_mission.shelf.loc)
    #         if vehicle.point not in [0, len(vehicle.path) - 1]:
    #             root_paths[moving_vehicle_id] = vehicle.path[vehicle.point :]
    #         directions.append(vehicle.direction)
    #         moving_vehicles.append(vehicle.id)
    #         # 添加额外约束，让AGV在货架处停留一段时间表示在抬起或放下货架
    #         additional_constraints.append(
    #             {
    #                 "agent": moving_vehicle_id,
    #                 "timestep": LIFTING_TIME,
    #                 "type": "additional",
    #             }
    #         )
    #         moving_vehicle_id += 1
    # elif vehicle.status == AgvStatus.TO_SELECT:
    #     last_delivery_mission = vehicle.delivery_missions[-1]
    #     assert last_delivery_mission.work_cell is not None
    #     # 如果更新路径时该AGV已经运行至工作台（正在分拣）
    #     if last_delivery_mission.work_cell.loc == vehicle.loc:
    #         staying_vehicles.append(vehicle.id)
    #     else:
    #         grid = deepcopy(AGV_MAP)
    #         # 将目标工作台处设为可通行
    #         grid[last_delivery_mission.work_cell.loc] = 0
    #         maps.append(grid)
    #         starts.append(vehicle.loc)
    #         ends.append(last_delivery_mission.work_cell.loc)
    #         if vehicle.point not in [0, len(vehicle.path) - 1]:
    #             root_paths[moving_vehicle_id] = vehicle.path[vehicle.point :]
    #         grid[last_delivery_mission.work_cell.loc] = 0
    #         directions.append(vehicle.direction)
    #         moving_vehicles.append(vehicle.id)
    #         moving_vehicle_id += 1
    # elif vehicle.status == AgvStatus.TO_CHARGE:
    #     if len(vehicle.charging_missions) > 0 and vehicle.charging_missions[-1].loc == vehicle.loc:
    #         staying_vehicles.append(vehicle.id)
    #     else:
    #         grid = deepcopy(AGV_MAP)
    #         # 将目标充电桩处设为可通行
    #         grid[vehicle.charging_missions[-1].loc[0]][vehicle.charging_missions[-1].loc[1]] = 0
    #         starts.append(vehicle.loc)
    #         ends.append(vehicle.charging_missions[-1].loc)
    #         if vehicle.point not in [0, len(vehicle.path) - 1]:
    #             root_paths[moving_vehicle_id] = vehicle.path[vehicle.point :]
    #         grid[vehicle.charging_missions[-1].loc[0]][vehicle.charging_missions[-1].loc[1]] = 0
    #         maps.append(grid)
    #         directions.append(vehicle.direction)
    #         moving_vehicles.append(vehicle.id)
    #         moving_vehicle_id += 1
    # elif vehicle.status == AgvStatus.BACK_TO_START:
    #     if vehicle.loc == vehicle.start:
    #         staying_vehicles.append(vehicle.id)
    #     else:
    #         grid = deepcopy(AGV_MAP)
    #         maps.append(grid)
    #         starts.append(vehicle.loc)
    #         ends.append(vehicle.start)
    #         if vehicle.point not in [0, len(vehicle.path) - 1]:
    #             root_paths[moving_vehicle_id] = vehicle.path[vehicle.point :]
    #         directions.append(vehicle.direction)
    #         moving_vehicles.append(vehicle.id)
    #         moving_vehicle_id += 1
    # elif vehicle.status == AgvStatus.ARRIVED_AT_START:
    #     staying_vehicles.append(vehicle.id)
    #     vehicle.status = AgvStatus.WAITING_AT_START
    #         logging.info(f"vehicle {vehicle.id} waiting at start")
    # logging.info(f"staying_vehicles: {staying_vehicles}")
    # logging.info(f"moving_vehicles: {moving_vehicles}")
    # logging.info(f"starts: {starts}")
    # logging.info(f"ends: {ends}")
    # logging.info(f"directions: {directions}")
    # logging.info(f"cbs starts searching")


def init_agvs(agv_num: int) -> List[AGV]:

    vehicles = [
        AGV(
            id=i,
            x=0,
            y=(i + 1) * (MAP.shape[1] // (agv_num + 1)),
            direction=Direction.DOWN,
            battery=FULL_CHARGE,
        )
        for i in range(agv_num)
    ]
    return vehicles


if __name__ == "__main__":
    pass
