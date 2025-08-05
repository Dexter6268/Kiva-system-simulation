import os
import time
import logging
import numpy as np
import seaborn as sns
from copy import deepcopy
import matplotlib.pyplot as plt
from kiva_sim.agv import AGV, AgvStatus, init_agvs, ChargingMission
from kiva_sim.cbs import cbs_reserve
from kiva_sim.visualization import create_animation
from kiva_sim.orders import (
    MAP,
    TABLE_COORDS,
    ShelfStatus,
    Shelf,
    init_orders,
    orderDistribute,
    manhattan_distance,
)

from kiva_sim.maps import Map, MAP
from kiva_sim.tables import init_tables


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


def init_simu(agv_num, order_num):
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


def go_to_charge(vehicle: AGV, shelf: Shelf, available_stations: list[dict], charging_stations: list[dict]) -> None:

    shelf.inplace = True
    vehicle.status = AgvStatus.TO_CHARGE
    logging.info(f"vehicle {vehicle.id} to charge")
    target = min(
        available_stations,
        key=lambda s: manhattan_distance(vehicle.loc, s["loc"]),
    )
    vehicle.charging_missions.append(ChargingMission(target["id"], target["loc"]))
    charging_stations[target["id"]]["occupied"] = True
    vehicle.point = 0


def simulation(
    seed: int,
    agv_num: int,
    order_num: int,
    interval: int = 200,
    show: bool = False,
    save_fig: bool = False,
    heat_map: bool = False,
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

    np.random.seed(seed)

    # 主函数 仿真+可视化
    # 初始化
    # -------------------------------------------------------------------------------------------------------
    revenue = 0  # 订单完成收益
    maxIteration = 1500  # cbs算法迭代上限
    AGV_MAP, tables, vehicles, shelves, charging_stations, orders = init_simu(agv_num, order_num)
    t = 0  # 时间步
    simInfo = []  # 仿真信息，用来实现可视化
    time_start = time.time()  # 主循环开始运行时间
    orders_completed_time = float("inf")
    tbreak = -1
    maxiter = 500
    iteration = 0
    # main loop
    # -------------------------------------------------------------------------------------------------------
    while iteration < maxiter:
        iteration += 1
        AGVInfo = []  # 包括AGV的位置、方向、颜色（是否正托举货架）、电量
        # 分配订单
        for order in orders:
            orderDistribute(order, vehicles, shelves, tables)
            logging.debug(order)
        for vehicle in vehicles:
            # 如果存在agv已分配任务且未启动或已完成任务
            if (
                vehicle.status
                not in [
                    AgvStatus.AVAILABLE,
                    AgvStatus.WAITING_TO_CHARGE,
                    AgvStatus.WAITING_AT_START,
                    AgvStatus.WAITING_TO_SELECT,
                ]
                and vehicle.point == 0
            ):
                logging.info("vehicle %d triggered renewing" % vehicle.id)
                cons = []  # 约束
                staying_vehicles = []  # 发生路径更新时已经抵达目标点的AGV（在抬起或放下货架，或在工作台处分拣）
                moving_vehicles = []  # 发生路径更新时未抵达目标点的AGV
                moving_vehicle_id = 0
                maps = []
                starts = []
                ends = []
                root_paths = {}
                directions = []
                arrived_at_start = any(vehicle.status == AgvStatus.ARRIVED_AT_START for vehicle in vehicles)
                for vehicle in vehicles:
                    # 如果AGV处于to shelf（从起点至目标货架）或return shelf（从工作台处送还货架）
                    if vehicle.status == AgvStatus.TO_SHELF or vehicle.status == AgvStatus.RETURN_SHELF:
                        # 如果更新路径时该AGV已经运行至目标货架处（正在抬起或放下货架）
                        if shelves[vehicle.delivery_missions[-1].shelf_id].loc == vehicle.loc:
                            staying_vehicles.append(vehicle.id)
                        else:
                            grid = deepcopy(AGV_MAP)
                            # 将目标货架处设为可通行
                            grid[SHELF_COORDS[vehicle.delivery_missions[-1].shelf_id][0]][
                                SHELF_COORDS[vehicle.delivery_missions[-1].shelf_id][1]
                            ] = 0
                            maps.append(grid)
                            starts.append((vehicle.x, vehicle.y))
                            ends.append(SHELF_COORDS[vehicle.delivery_missions[-1].shelf_id])
                            if vehicle.point not in [0, len(vehicle.path) - 1]:
                                root_paths[moving_vehicle_id] = vehicle.path[vehicle.point :]
                            directions.append(vehicle.direction)
                            moving_vehicles.append(vehicle.id)
                            # 添加额外约束，让AGV在货架处停留一段时间表示在抬起或放下货架
                            cons.append(
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
                        if last_delivery_mission.work_cell.loc == (vehicle.x, vehicle.y):
                            staying_vehicles.append(vehicle.id)
                        else:
                            grid = deepcopy(AGV_MAP)
                            # 将目标工作台处设为可通行
                            grid[last_delivery_mission.work_cell.loc] = 0
                            maps.append(grid)
                            starts.append((vehicle.x, vehicle.y))
                            ends.append(last_delivery_mission.work_cell.loc)
                            if vehicle.point not in [0, len(vehicle.path) - 1]:
                                root_paths[moving_vehicle_id] = vehicle.path[vehicle.point :]
                            grid[last_delivery_mission.work_cell.loc] = 0
                            directions.append(vehicle.direction)
                            moving_vehicles.append(vehicle.id)
                            moving_vehicle_id += 1
                    elif vehicle.status == AgvStatus.TO_CHARGE:
                        if len(vehicle.charging_missions) > 0 and vehicle.charging_missions[-1].loc == (
                            vehicle.x,
                            vehicle.y,
                        ):
                            staying_vehicles.append(vehicle.id)
                        else:
                            grid = deepcopy(AGV_MAP)
                            # 将目标充电桩处设为可通行
                            grid[vehicle.charging_missions[-1].loc[0]][vehicle.charging_missions[-1].loc[1]] = 0
                            starts.append((vehicle.x, vehicle.y))
                            ends.append(vehicle.charging_missions[-1].loc)
                            if vehicle.point not in [0, len(vehicle.path) - 1]:
                                root_paths[moving_vehicle_id] = vehicle.path[vehicle.point :]
                            grid[vehicle.charging_missions[-1].loc[0]][vehicle.charging_missions[-1].loc[1]] = 0
                            maps.append(grid)
                            directions.append(vehicle.direction)
                            moving_vehicles.append(vehicle.id)
                            moving_vehicle_id += 1
                    elif vehicle.status == AgvStatus.BACK_TO_START:
                        if (vehicle.x, vehicle.y) == vehicle.start:
                            staying_vehicles.append(vehicle.id)
                        else:
                            grid = deepcopy(AGV_MAP)
                            maps.append(grid)
                            starts.append((vehicle.x, vehicle.y))
                            ends.append(vehicle.start)
                            if vehicle.point not in [0, len(vehicle.path) - 1]:
                                root_paths[moving_vehicle_id] = vehicle.path[vehicle.point :]
                            directions.append(vehicle.direction)
                            moving_vehicles.append(vehicle.id)
                            moving_vehicle_id += 1
                    elif vehicle.status == AgvStatus.ARRIVED_AT_START:
                        staying_vehicles.append(vehicle.id)
                        vehicle.status = AgvStatus.WAITING_AT_START
                        logging.info(f"vehicle {vehicle.id} waiting at start")
                logging.info(f"staying_vehicles: {staying_vehicles}")
                logging.info(f"moving_vehicles: {moving_vehicles}")
                logging.info(f"starts: {starts}")
                logging.info(f"ends: {ends}")
                logging.info(f"directions: {directions}")
                logging.info(f"cbs starts searching")
                if starts:
                    # 用cbs算法为所有moving vehicles生成无冲突路径
                    paths = cbs_reserve(
                        maps,
                        arrived_at_start,
                        root_paths,
                        starts,
                        ends,
                        directions,
                        cons,
                        maxIteration,
                    )
                    # 将生成的路径分配给moving vehicles
                    if paths:
                        for i in moving_vehicles:
                            vehicles[i].point = 0
                            vehicles[i].path = paths.pop(0)
                            # 更新AGV颜色列表
                            if vehicles[i].status == AgvStatus.TO_SHELF:
                                vehicles[i].color_list = ["k"] * len(vehicles[i].path)
                            elif vehicles[i].status in [
                                AgvStatus.TO_SELECT,
                                AgvStatus.RETURN_SHELF,
                            ]:
                                vehicles[i].color_list = ["y"] * len(vehicles[i].path)
                            elif vehicles[i].status == AgvStatus.TO_CHARGE:
                                vehicles[i].color_list = ["k"] * len(vehicles[i].path)
                            elif vehicles[i].status == AgvStatus.BACK_TO_START:
                                vehicles[i].color_list = ["k"] * len(vehicles[i].path)
                    else:
                        raise BaseException("cbs no solution!")
                break

        shelfInfo = ["y"] * SHELF_NUM  # 货架颜色信息
        for vehicle in vehicles:
            # 录入AGV信息
            target = "None"
            if vehicle.status in [AgvStatus.TO_SHELF, AgvStatus.RETURN_SHELF]:
                last_delivery_mission = vehicle.delivery_missions[-1]
                target = f"shelf {last_delivery_mission.shelf_id}"
            elif vehicle.status == AgvStatus.TO_SELECT:
                last_delivery_mission = vehicle.delivery_missions[-1]
                assert last_delivery_mission.work_cell is not None
                target = f"table {last_delivery_mission.work_cell.table_id}"
            elif vehicle.status == AgvStatus.TO_CHARGE:
                target = f"charging station {vehicle.charging_missions[-1].charging_station_id}"
            AGVInfo.append(
                {
                    "id": vehicle.id,
                    "x": vehicle.x,
                    "y": vehicle.y,
                    "direction": vehicle.direction,
                    "color": vehicle.color,
                    "status": vehicle.status,
                    "battery": "%.1f%%" % (vehicle.battery / FULL_CHARGE * 100),
                    "target": target,
                }
            )
            order_left_to_assign = sum(shelf.status == ShelfStatus.TODO for order in orders for shelf in order.shelves)

            numOfBackToStart = sum(agv.status == AgvStatus.BACK_TO_START for agv in vehicles)

            # 如果AGV空闲且没有剩余的未指派订单，则令AGV返回起点
            if vehicle.status == AgvStatus.AVAILABLE and order_left_to_assign == 0:
                if (vehicle.x, vehicle.y) != vehicle.start:
                    # 分批返回起点（如果当前处于返程的AGV超过总数的一半则继续等待），防止一次性返回车数过多，造成拥堵
                    if numOfBackToStart <= agv_num // 2:
                        logging.info(f"vehicle {vehicle.id} back to start")
                        vehicle.status = AgvStatus.BACK_TO_START
                else:
                    vehicle.status = AgvStatus.WAITING_AT_START
                    AGV_MAP[vehicle.start[0]][vehicle.start[1]] = 4
            elif vehicle.status != AgvStatus.AVAILABLE and vehicle.status != AgvStatus.WAITING_AT_START:
                vehicle.move()
                if vehicle.color == "y":
                    shelf_color = "w"
                elif vehicle.color == "k":
                    shelf_color = "y"
                last_delivery_mission = vehicle.delivery_missions[-1]
                shelfInfo[last_delivery_mission.shelf_id] = shelf_color

                # 当AGV完成一个阶段的任务，更新AGV对象状态参数
                if vehicle.point == len(vehicle.path) - 1:
                    if vehicle.status == AgvStatus.TO_SHELF:
                        logging.info(
                            f"vehicle {vehicle.id} reached {shelves[last_delivery_mission.shelf_id]} for the first time"
                        )
                        available_workcells = [
                            work_cell
                            for table in tables
                            for work_cell in table.work_cells
                            if not work_cell.occupied
                            and (
                                (orders[last_delivery_mission.order_id].table is None)
                                or (orders[last_delivery_mission.order_id].table == table.id)
                            )
                        ]

                        if available_workcells:
                            target_work_cell = min(
                                available_workcells,
                                key=lambda wc: manhattan_distance(vehicle.loc, wc.loc),
                            )
                            orders[last_delivery_mission.order_id].table = target_work_cell.table_id
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
                        logging.info(
                            f"vehicle {vehicle.id} reached {shelves[last_delivery_mission.shelf_id]} for the second time"
                        )
                        if vehicle.check_battery(MAP):
                            # 更新货架在位情况
                            shelves[last_delivery_mission.shelf_id].inplace = True
                            vehicle.status = AgvStatus.AVAILABLE
                            vehicle.point = 0
                        else:
                            available_stations = [station for station in charging_stations if not station["occupied"]]
                            if available_stations:
                                go_to_charge(
                                    vehicle,
                                    shelves[last_delivery_mission.shelf_id],
                                    available_stations,
                                    charging_stations,
                                )
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
                            logging.info(f"vehicle {vehicle.id} returning {shelves[last_delivery_mission.shelf_id]}")
                            revenue += last_delivery_mission.tsort  # 结算分拣收益
                            assert last_delivery_mission.work_cell is not None
                            last_delivery_mission.work_cell.occupied = False

                            shelves[last_delivery_mission.shelf_id].status = ShelfStatus.DONE
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
                        available_workcells = [
                            work_cell
                            for table in tables
                            for work_cell in table.work_cells
                            if not work_cell.occupied
                            and (
                                (orders[last_delivery_mission.order_id].table is None)
                                or (orders[last_delivery_mission.order_id].table == table.id)
                            )
                        ]
                        if available_workcells:
                            target_work_cell = min(
                                available_workcells,
                                key=lambda wc: manhattan_distance(vehicle.loc, wc.loc),
                            )
                            orders[last_delivery_mission.order_id].table = target_work_cell.table_id

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
                            go_to_charge(
                                vehicle,
                                shelves[last_delivery_mission.shelf_id],
                                available_stations,
                                charging_stations,
                            )
                    elif vehicle.status == AgvStatus.BACK_TO_START:
                        AGV_MAP[vehicle.start] = 4
                        numOfIdle = sum(agv.status == AgvStatus.WAITING_AT_START for agv in vehicles)
                        if numOfIdle < agv_num - 1:
                            logging.info(f"vehicle {vehicle.id} arrived at start")
                            vehicle.status = AgvStatus.ARRIVED_AT_START
                            vehicle.point = 0
                        else:
                            vehicle.status = AgvStatus.WAITING_AT_START
        orders_completed = sum(all(shelf.status == ShelfStatus.DONE for shelf in order.shelves) for order in orders)

        simInfo.append(
            {
                "AGVInfo": AGVInfo,
                "shelfInfo": shelfInfo,
                "orders_completed": orders_completed,
                "t": t,
                "revenue": revenue,
            }
        )

        if t == tbreak:
            break
        numOfIdle = sum(vehicle.status == AgvStatus.WAITING_AT_START for vehicle in vehicles)
        logging.info(f"orders_completed: {orders_completed}")
        logging.info("-" * 80)
        if orders_completed == order_num:
            orders_completed_time = min(orders_completed_time, t)
            if numOfIdle == agv_num:
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
    for info in simInfo:
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
        logging.info(f"vehicle {i} utility {utilized_time[i] / t:.2%}")
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
        for i in range(len(simInfo) - 1, 0, -1):
            simInfo[i]["shelfInfo"] = simInfo[i - 1]["shelfInfo"]
        ani, fps = create_animation(
            MAP,
            simInfo,
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
