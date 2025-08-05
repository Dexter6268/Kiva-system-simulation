from ast import Tuple
import os
from enum import IntEnum
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
from kiva_sim.states import AgvStatus, Direction
from kiva_sim.orders import Shelf
from kiva_sim.maps import MAP
from kiva_sim.tables import WorkCell

FULL_CHARGE = int(os.getenv("FULL_CHARGE", "3600"))
BATTERY_CONSUMING_SPEED = int(os.getenv("BATTERY_CONSUMING_SPEED", "1"))
CHARGING_SPEED = 6 * BATTERY_CONSUMING_SPEED


@dataclass
class DeliveryMission:
    """Represents a delivery mission assigned to an AGV."""

    order_id: int
    sub_order_id: int
    shelf: Shelf
    work_cell: Optional[WorkCell] = None
    tsort: Optional[int] = None


@dataclass
class ChargingMission:
    """Represents a charging mission."""

    charging_station_id: int
    loc: Tuple[int, int]


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
