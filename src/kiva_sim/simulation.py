import os
import time
import logging
from webbrowser import get
import numpy as np
import seaborn as sns
from copy import deepcopy
import matplotlib.pyplot as plt
from typing import List, Tuple, Dict, Optional
from kiva_sim.states import AgvStatus, OrderStatus
from kiva_sim.agv import AGV, init_agvs, Shelf
from kiva_sim.cbs import cbs_reserve
from kiva_sim.visualization import create_animation
from kiva_sim.maps import Map, MAP
from kiva_sim.utlis import manhattan_distance
from kiva_sim.tables import Table, WorkCell, init_tables
from kiva_sim.orders import Order, init_orders, distribute_order, get_available_workcells, get_target_workcell


SHELF_COORDS = MAP.shelf_coords
TABLE_COORDS = MAP.table_coords
CHARGING_STATION_COORD = MAP.charging_station_coords
SHELF_NUM = len(SHELF_COORDS)
TABLE_NUM = len(TABLE_COORDS)
CHARGING_STATION_NUM = len(CHARGING_STATION_COORD)

FULL_CHARGE = int(os.getenv("FULL_CHARGE", 3600))
LIFTING_TIME = int(os.getenv("LIFTING_TIME", 4))  # AGV抬起及放下货架时长
AGV_COST = float(os.getenv("AGV_COST", 75000 * 1.05 / 5 / 365 / 24 / 3600))  # 每辆AGV的购入及维护成本
CHARGING_STATION_COST = float(os.getenv("CHARGING_STATION_COST", 20000 / 5 / 365 / 24 / 3600))
WORKER_COST = float(os.getenv("WORKER_COST", 34 / 3600))  # 分拣工人每秒薪资


def init_simu(
    agv_num: int, order_num: int
) -> Tuple[Map, List[Table], List[AGV], List[Shelf], List[dict], List[Order]]:
    AGV_MAP = deepcopy(MAP)  # map for AGVs
    tables = init_tables(TABLE_NUM)
    for table in tables:
        for cell in table.work_cells:
            AGV_MAP[cell.loc] = 5  # 将工作台附近禁止通行
    vehicles = init_agvs(agv_num)
    shelves = [Shelf(i) for i in range(SHELF_NUM)]
    charging_stations = [
        {"id": i, "loc": CHARGING_STATION_COORD[i], "occupied": False} for i in range(CHARGING_STATION_NUM)
    ]

    orders = init_orders(shelves=shelves, order_num=order_num)
    return AGV_MAP, tables, vehicles, shelves, charging_stations, orders


def simulation(
    agv_num: int,
    order_num: int,
    interval: int = 200,
    show: bool = False,
    save_fig: bool = False,
    heat_map: bool = False,
    astar_max_iter: int = 1500,
    cbs_max_iter: int = 1000,
    simu_max_iter: int = 500,
    random_seed: Optional[int] = None,
):
    """
    功能：运行仿真
    :param seed: int，仿真运行的随机数种子
    :param agv_num: int， AGV数量
    :param order_num: int，订单数量
    :param interval: int，动画每一帧的时间间隔
    :param show: Boolean，是否生成动画
    :param save_fig: Boolean，是否保存动画
    :param heat_map: Boolean，是否生成热力图
    :return: list， 仿真信息列表
    """
    if random_seed is not None:
        np.random.seed(random_seed)
        logging.info(f"random seed set to {random_seed}")

    # 主函数 仿真+可视化
    # 初始化
    # -------------------------------------------------------------------------------------------------------
    revenue = 0  # 订单完成收益
    # cbs算法迭代上限
    AGV_MAP, tables, vehicles, shelves, charging_stations, orders = init_simu(agv_num, order_num)
    t = 0  # 时间步
    sim_info = []  # 仿真信息，用来实现可视化
    orders_completed_time = float("inf")
    tbreak = -1

    iteration = 0
    time_start = time.time()
    # main loop
    # -------------------------------------------------------------------------------------------------------
    while iteration < simu_max_iter:
        iteration += 1
        agv_info = []  # 包括AGV的位置、方向、颜色（是否正托举货架）、电量
        # 分配订单
        for order in orders:
            distribute_order(order, vehicles, shelves, tables)
            logging.debug(f"{order}")
        for vehicle in vehicles:
            # 如果存在agv已分配任务且未启动或已完成任务
            if vehicle.needs_path_renewal():
                logging.info(f"vehicle {vehicle.id} triggered renewing")
                additional_constraints: List[Dict] = []
                # 发生路径更新时已经抵达目标点的AGV（在抬起或放下货架，或在工作台处分拣）
                staying_vehicles: List[int] = []
                # 发生路径更新时未抵达目标点的AGV
                moving_vehicle_ids: List[int] = []
                moving_vehicle_id: int = 0
                maps = []
                starts: List[Tuple[int, int]] = []
                ends = []
                root_paths: Dict = {}
                directions = []
                arrived_at_start = any(vehicle.status == AgvStatus.ARRIVED_AT_START for vehicle in vehicles)
                for vehicle in vehicles:
                    if vehicle.status == AgvStatus.TO_SHELF or vehicle.status == AgvStatus.RETURN_SHELF:
                        # 如果更新路径时该AGV已经运行至目标货架处（正在抬起或放下货架）
                        last_delivery_mission = vehicle.delivery_missions[-1]
                        if last_delivery_mission.shelf.loc == vehicle.loc:
                            staying_vehicles.append(vehicle.id)
                        else:
                            grid = deepcopy(AGV_MAP)
                            # 将目标货架处设为可通行
                            grid[last_delivery_mission.shelf.loc] = 0
                            maps.append(grid)
                            starts.append(vehicle.loc)
                            ends.append(last_delivery_mission.shelf.loc)
                            if vehicle.point not in [0, len(vehicle.path) - 1]:
                                root_paths[moving_vehicle_id] = vehicle.path[vehicle.point :]
                            directions.append(vehicle.direction)
                            moving_vehicle_ids.append(vehicle.id)
                            # 添加额外约束，让AGV在货架处停留一段时间表示在抬起或放下货架
                            additional_constraints.append(
                                {
                                    "agent": moving_vehicle_id,
                                    "timestep": LIFTING_TIME,
                                    "type": "additional",
                                }
                            )
                            moving_vehicle_id += 1
                    elif vehicle.status == AgvStatus.TO_SELECT:
                        last_delivery_mission = vehicle.delivery_missions[-1]
                        assert last_delivery_mission.work_cell is not None
                        # 如果更新路径时该AGV已经运行至工作台（正在分拣）
                        if last_delivery_mission.work_cell.loc == vehicle.loc:
                            staying_vehicles.append(vehicle.id)
                        else:
                            grid = deepcopy(AGV_MAP)
                            # 将目标工作台处设为可通行
                            grid[last_delivery_mission.work_cell.loc] = 0
                            maps.append(grid)
                            starts.append(vehicle.loc)
                            ends.append(last_delivery_mission.work_cell.loc)
                            if vehicle.point not in [0, len(vehicle.path) - 1]:
                                root_paths[moving_vehicle_id] = vehicle.path[vehicle.point :]
                            grid[last_delivery_mission.work_cell.loc] = 0
                            directions.append(vehicle.direction)
                            moving_vehicle_ids.append(vehicle.id)
                            moving_vehicle_id += 1
                    elif vehicle.status == AgvStatus.TO_CHARGE:
                        if len(vehicle.charging_missions) > 0 and vehicle.charging_missions[-1].loc == vehicle.loc:
                            staying_vehicles.append(vehicle.id)
                        else:
                            grid = deepcopy(AGV_MAP)
                            # 将目标充电桩处设为可通行
                            grid[vehicle.charging_missions[-1].loc[0]][vehicle.charging_missions[-1].loc[1]] = 0
                            starts.append(vehicle.loc)
                            ends.append(vehicle.charging_missions[-1].loc)
                            if vehicle.point not in [0, len(vehicle.path) - 1]:
                                root_paths[moving_vehicle_id] = vehicle.path[vehicle.point :]
                            grid[vehicle.charging_missions[-1].loc[0]][vehicle.charging_missions[-1].loc[1]] = 0
                            maps.append(grid)
                            directions.append(vehicle.direction)
                            moving_vehicle_ids.append(vehicle.id)
                            moving_vehicle_id += 1
                    elif vehicle.status == AgvStatus.BACK_TO_START:
                        if vehicle.loc == vehicle.start:
                            staying_vehicles.append(vehicle.id)
                        else:
                            grid = deepcopy(AGV_MAP)
                            maps.append(grid)
                            starts.append(vehicle.loc)
                            ends.append(vehicle.start)
                            if vehicle.point not in [0, len(vehicle.path) - 1]:
                                root_paths[moving_vehicle_id] = vehicle.path[vehicle.point :]
                            directions.append(vehicle.direction)
                            moving_vehicle_ids.append(vehicle.id)
                            moving_vehicle_id += 1
                    elif vehicle.status == AgvStatus.ARRIVED_AT_START:
                        staying_vehicles.append(vehicle.id)
                        vehicle.status = AgvStatus.WAITING_AT_START
                        logging.info(f"vehicle {vehicle.id} waiting at start")
                logging.info(f"staying_vehicles: {staying_vehicles}")
                logging.info(f"moving_vehicles: {moving_vehicle_ids}")
                logging.info(f"starts: {starts}")
                logging.info(f"ends: {ends}")
                logging.info(f"directions: {directions}")
                logging.info(f"cbs starts searching")
                if not starts:
                    break

                paths = cbs_reserve(
                    maps,
                    arrived_at_start,
                    root_paths,
                    starts,
                    ends,
                    directions,
                    additional_constraints,
                    cbs_max_iter,
                    astar_max_iter,
                )
                # 将生成的路径分配给moving vehicles
                if paths is None:
                    raise BaseException("cbs no solution!")

                for id, path in zip(moving_vehicle_ids, paths):
                    vehicles[id].updates_path(path)
                break

        shelf_info = ["y"] * SHELF_NUM  # 货架颜色信息
        for vehicle in vehicles:
            # 录入AGV信息
            target = "None"
            if vehicle.status in [AgvStatus.TO_SHELF, AgvStatus.RETURN_SHELF]:
                last_delivery_mission = vehicle.delivery_missions[-1]
                target = f"shelf {last_delivery_mission.shelf.id}"
            elif vehicle.status == AgvStatus.TO_SELECT:
                last_delivery_mission = vehicle.delivery_missions[-1]
                assert last_delivery_mission.work_cell is not None
                target = f"table {last_delivery_mission.work_cell.table_id}"
            elif vehicle.status == AgvStatus.TO_CHARGE:
                target = f"charging station {vehicle.charging_missions[-1].charging_station_id}"
            agv_info.append(
                {
                    "id": vehicle.id,
                    "x": vehicle.x,
                    "y": vehicle.y,
                    "direction": repr(vehicle.direction),
                    "color": vehicle.color,
                    "status": repr(vehicle.status),
                    "battery": f"{vehicle.battery / FULL_CHARGE: .1%}",
                    "target": target,
                }
            )
            num_suborders_unassigned = sum(
                sub_order.status == OrderStatus.TODO for order in orders for sub_order in order.sub_orders
            )

            num_agv_back_to_start = sum(agv.status == AgvStatus.BACK_TO_START for agv in vehicles)

            # 如果AGV空闲且没有剩余的未指派订单，则令AGV返回起点
            if vehicle.status == AgvStatus.AVAILABLE and num_suborders_unassigned == 0:
                if vehicle.loc != vehicle.start:
                    # 分批返回起点（如果当前处于返程的AGV超过总数的一半则继续等待），防止一次性返回车数过多，造成拥堵
                    if num_agv_back_to_start <= agv_num // 2:
                        logging.info(f"vehicle {vehicle.id} back to start")
                        vehicle.status = AgvStatus.BACK_TO_START
                else:
                    vehicle.status = AgvStatus.WAITING_AT_START
                    AGV_MAP[vehicle.start] = 4
            elif vehicle.status != AgvStatus.AVAILABLE and vehicle.status != AgvStatus.WAITING_AT_START:
                vehicle.move()
                shelf_color = "w" if vehicle.color == "y" else "y"
                last_delivery_mission = vehicle.delivery_missions[-1]
                shelf_info[last_delivery_mission.shelf.id] = shelf_color

                # 当AGV完成一个阶段的任务，更新AGV对象状态参数
                if vehicle.point == len(vehicle.path) - 1:
                    if vehicle.status == AgvStatus.TO_SHELF:
                        logging.info(f"vehicle {vehicle.id} reached {last_delivery_mission.shelf} for the first time")
                        available_workcells = get_available_workcells(tables, orders[last_delivery_mission.order_id])
                        if available_workcells:
                            target_work_cell = get_target_workcell(available_workcells, vehicle)
                            orders[last_delivery_mission.order_id].table_id = target_work_cell.table_id
                            last_delivery_mission.work_cell = target_work_cell

                            target_work_cell.occupied = True
                            vehicle.status = AgvStatus.TO_SELECT
                            logging.info(f"vehicle {vehicle.id} going for {target_work_cell}")
                            vehicle.point = 0
                        else:
                            vehicle.status = AgvStatus.WAITING_TO_SELECT
                            logging.info(f"vehicle {vehicle.id} waiting to select at position {vehicle.loc}")
                    elif vehicle.status == AgvStatus.TO_SELECT:
                        logging.info(f"vehicle {vehicle.id} reached {last_delivery_mission.work_cell}")
                        vehicle.status = AgvStatus.SELECTING
                        logging.info(f"vehicle {vehicle.id} start selecting at position {vehicle.loc}")
                    elif vehicle.status == AgvStatus.RETURN_SHELF:
                        logging.info(f"vehicle {vehicle.id} reached {last_delivery_mission.shelf} for the second time")
                        if vehicle.check_battery(MAP):
                            # 更新货架在位情况
                            last_delivery_mission.shelf.agv = None
                            vehicle.status = AgvStatus.AVAILABLE
                            vehicle.point = 0
                        else:
                            available_stations = [station for station in charging_stations if not station["occupied"]]
                            if available_stations:
                                vehicle.goto_charge(available_stations, charging_stations)
                            else:
                                vehicle.status = AgvStatus.WAITING_TO_CHARGE
                                logging.info(f"vehicle {vehicle.id} waiting to charge")
                    elif vehicle.status == AgvStatus.TO_CHARGE:
                        vehicle.status = AgvStatus.CHARGING
                        logging.info(f"vehicle {vehicle.id} start charging at position {vehicle.loc}")
                    elif vehicle.status == AgvStatus.SELECTING:
                        vehicle.selecting_process += 1
                        assert isinstance(last_delivery_mission.tsort, int)
                        if vehicle.selecting_process >= last_delivery_mission.tsort:
                            logging.info(f"vehicle {vehicle.id} finished selecting at position f{vehicle.loc}")
                            vehicle.status = AgvStatus.RETURN_SHELF
                            logging.info(f"vehicle {vehicle.id} returning {last_delivery_mission.shelf}")
                            revenue += last_delivery_mission.tsort  # 结算分拣收益
                            assert last_delivery_mission.work_cell is not None
                            last_delivery_mission.work_cell.occupied = False
                            last_delivery_mission.sub_order.status = OrderStatus.DONE
                            vehicle.selecting_process = 0
                            vehicle.point = 0
                    elif vehicle.status == AgvStatus.CHARGING:
                        vehicle.charge()
                        if vehicle.battery == FULL_CHARGE:
                            logging.info(f"vehicle {vehicle.id} finished charging at position {vehicle.loc}")
                            vehicle.status = AgvStatus.AVAILABLE
                            charging_stations[vehicle.charging_missions[-1].charging_station_id]["occupied"] = False
                            vehicle.point = 0
                    elif vehicle.status == AgvStatus.WAITING_TO_SELECT:
                        available_workcells = get_available_workcells(tables, orders[last_delivery_mission.order_id])
                        if available_workcells:
                            target_work_cell = get_target_workcell(available_workcells, vehicle)
                            orders[last_delivery_mission.order_id].table_id = target_work_cell.table_id
                            last_delivery_mission.work_cell = target_work_cell
                            target_work_cell.occupied = True
                            vehicle.status = AgvStatus.TO_SELECT
                            logging.info(f"vehicle {vehicle.id} to select at {target_work_cell}")
                            vehicle.point = 0
                        else:
                            logging.info(f"vehicle {vehicle.id} waiting to select at position {vehicle.loc}")
                    elif vehicle.status == AgvStatus.WAITING_TO_CHARGE:
                        logging.info(f"vehicle {vehicle.id} waiting to charge at position {vehicle.loc}")
                        available_stations = [station for station in charging_stations if not station["occupied"]]
                        if available_stations:
                            vehicle.goto_charge(available_stations, charging_stations)
                    elif vehicle.status == AgvStatus.BACK_TO_START:
                        AGV_MAP[vehicle.start] = 4
                        num_idle_vehicles = sum(agv.status == AgvStatus.WAITING_AT_START for agv in vehicles)
                        if num_idle_vehicles < agv_num - 1:
                            logging.info(f"vehicle {vehicle.id} arrived at start")
                            vehicle.status = AgvStatus.ARRIVED_AT_START
                            vehicle.point = 0
                        else:
                            vehicle.status = AgvStatus.WAITING_AT_START
        num_orders_completed = sum(order.status == OrderStatus.DONE for order in orders)

        sim_info.append(
            {
                "AGVInfo": agv_info,
                "shelfInfo": shelf_info,
                "orders_completed": num_orders_completed,
                "t": t,
                "revenue": revenue,
            }
        )

        if t == tbreak:
            break
        num_idle_vehicles = sum(vehicle.status == AgvStatus.WAITING_AT_START for vehicle in vehicles)
        logging.info(f"orders_completed: {num_orders_completed}")
        logging.info("-" * 80)
        if num_orders_completed == order_num:
            orders_completed_time = min(orders_completed_time, t)
            if num_idle_vehicles == agv_num:
                tbreak = t + 1
        t += 1
    # -------------------------------------------------------------------------------------------------------
    time_end = time.time()
    time_sum = time_end - time_start  # 计算的时间差为程序的执行时间，单位为秒/s

    total_net_revenue = (
        revenue * 0.5
        - (AGV_COST * agv_num + CHARGING_STATION_COST * CHARGING_STATION_NUM) * t
        - WORKER_COST * TABLE_NUM * orders_completed_time
    )
    revenue_per_hour = total_net_revenue / t * 3600
    logging.info(f"run time: {time_sum}")
    logging.info(f"{order_num} orders completed with {agv_num} agvs with {orders_completed_time} seconds")
    logging.info(
        f"AGV_COST: {AGV_COST * agv_num * t:.2f}, CHARGING_STATION_COST: {CHARGING_STATION_COST * CHARGING_STATION_NUM * t:.2f}, WORKER_COST: {WORKER_COST * TABLE_NUM * orders_completed_time:.1f}, revenue: {revenue * 0.5:.1f}"
    )
    logging.info(f"total net revenue: {total_net_revenue:.1f}")
    logging.info(f"revenue_per_hour: {revenue_per_hour:.1f}")
    heat_map_data = np.zeros(MAP.shape)  # 热力图矩阵
    utilized_time = [0] * agv_num  # 各AGV被利用的时间步数
    for info in sim_info:
        for i, vehicle in enumerate(info["AGVInfo"]):
            heat_map_data[vehicle["x"]][vehicle["y"]] += 1
            if vehicle["status"] in [
                AgvStatus.TO_SHELF,
                AgvStatus.TO_SELECT,
                AgvStatus.SELECTING,
                AgvStatus.RETURN_SHELF,
            ]:
                utilized_time[i] += 1
    mean_utility = sum(utilized_time) / (t * agv_num)
    for i, ut in enumerate(utilized_time):
        logging.info(f"vehicle {i} utility {ut / t:.2%}")
    logging.info(f"mean agv utility: {mean_utility * 100:.2%}")

    if heat_map:
        sns.set_context({"figure.figsize": (MAP.shape[1], MAP.shape[0])})
        sns.heatmap(
            data=heat_map_data,
            square=True,
            cmap="RdBu_r",
            linewidths=0.3,
            cbar_kws={"shrink": 0.8},
        )
        plt.savefig(f"heat_map_{order_num}_orders_{agv_num}_AGVs.png", dpi=300)
        plt.show()

    if show:
        # 货架颜色信息由于主循环内更新先后顺序原因出现1时间步错位，在此矫正。
        for i in range(len(sim_info) - 1, 0, -1):
            sim_info[i]["shelfInfo"] = sim_info[i - 1]["shelfInfo"]
        ani, fps = create_animation(
            MAP,
            sim_info,
            order_num,
            AGV_COST,
            CHARGING_STATION_COST,
            WORKER_COST,
            TABLE_NUM,
            orders_completed_time,
            interval=interval,
            SAVE_GIF=save_fig,
        )
        if save_fig:
            ani.save(
                f"gifs/map1_{order_num}_orders_{agv_num}_AGVs.gif",
                fps=fps,
                writer="pillow",
            )
        plt.show()
    return np.array(
        [
            agv_num,
            order_num,
            orders_completed_time,  # 订单完成时间
            revenue_per_hour,  # 每小时的净收益
            mean_utility,  # AGV的利用率
            total_net_revenue,  # 总净收益
            revenue * 0.5,  # 订单完成利润
            AGV_COST * agv_num * t,  # AGV成本
            CHARGING_STATION_COST * CHARGING_STATION_NUM * t,  # 充电桩成本
            WORKER_COST * TABLE_NUM * orders_completed_time,
        ]
    )  # 工作台工人成本
