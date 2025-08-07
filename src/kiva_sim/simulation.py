import os
import time
import logging
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
from kiva_sim.tables import Table, init_tables
from kiva_sim.orders import Order, init_orders, distribute_order
from kiva_sim.models import ChargingStation


SHELF_COORDS = MAP.shelf_coords
CHARGING_STATION_COORD = MAP.charging_station_coords
TABLE_NUM = len(MAP.table_coords)
SHELF_NUM = len(SHELF_COORDS)
CHARGING_STATION_NUM = len(CHARGING_STATION_COORD)

FULL_CHARGE = int(os.getenv("FULL_CHARGE", 3600))
LIFTING_TIME = int(os.getenv("LIFTING_TIME", 4))  # AGV抬起及放下货架时长
AGV_COST = float(os.getenv("AGV_COST", 75000 * 1.05 / 5 / 365 / 24 / 3600))  # 每辆AGV的购入及维护成本
CHARGING_STATION_COST = float(os.getenv("CHARGING_STATION_COST", 20000 / 5 / 365 / 24 / 3600))
WORKER_COST = float(os.getenv("WORKER_COST", 34 / 3600))  # 分拣工人每秒薪资


def init_simu(
    agv_num: int, order_num: int
) -> Tuple[Map, List[Table], List[AGV], List[Shelf], List[ChargingStation], List[Order]]:
    """Initialize the simulation environment with AGVs, orders, and other components."""
    GLOBAL_AGV_MAP = deepcopy(MAP)  # map for all AGVs
    tables = init_tables(TABLE_NUM, MAP)
    for table in tables:
        for cell in table.work_cells:
            GLOBAL_AGV_MAP[cell.loc] = 5  # 将工作台附近禁止通行
    vehicles = init_agvs(agv_num, MAP)
    shelves = [Shelf(i, SHELF_COORDS[i]) for i in range(SHELF_NUM)]
    charging_stations = [ChargingStation(i, CHARGING_STATION_COORD[i]) for i in range(CHARGING_STATION_NUM)]
    orders = init_orders(shelves=shelves, order_num=order_num)
    return GLOBAL_AGV_MAP, tables, vehicles, shelves, charging_stations, orders


def get_moving_vehicles(vehicles: List[AGV], AGV_MAP: Map) -> Tuple[List[AGV], List[Dict]]:
    additional_constraints: List[Dict] = []
    # 发生路径更新时未抵达目标点的AGV
    moving_vehicles: List[AGV] = []
    for vehicle in vehicles:
        if (
            vehicle.status in [AgvStatus.TO_SHELF, AgvStatus.RETURN_SHELF]
            and vehicle.delivery_missions[-1].shelf.loc != vehicle.loc
        ):
            # 如果更新路径时该AGV已经运行至目标货架处（正在抬起或放下货架）
            vehicle.updates_map(AGV_MAP, vehicle.delivery_missions[-1].shelf.loc)
            # 添加额外约束，让AGV在货架处停留一段时间表示在抬起或放下货架
            additional_constraints.append(
                {"agent": len(moving_vehicles), "timestep": LIFTING_TIME, "type": "additional"}
            )
            moving_vehicles.append(vehicle)
        elif (
            vehicle.status == AgvStatus.TO_SELECT
            and vehicle.delivery_missions[-1].work_cell.loc != vehicle.loc  # type: ignore
        ):
            vehicle.updates_map(AGV_MAP, vehicle.delivery_missions[-1].work_cell.loc)  # type: ignore
            moving_vehicles.append(vehicle)
        elif vehicle.status == AgvStatus.TO_CHARGE and (
            len(vehicle.charging_missions) == 0 or vehicle.charging_missions[-1].loc != vehicle.loc
        ):
            vehicle.updates_map(AGV_MAP, vehicle.charging_missions[-1].loc)
            moving_vehicles.append(vehicle)
        elif vehicle.status == AgvStatus.BACK_TO_START and vehicle.loc != vehicle.start:
            vehicle.updates_map(AGV_MAP, vehicle.start)
            moving_vehicles.append(vehicle)
        elif vehicle.status == AgvStatus.ARRIVED_AT_START:
            vehicle.status = AgvStatus.WAITING_AT_START
            logging.info(f"vehicle {vehicle.id} waiting at start")
    return moving_vehicles, additional_constraints


def extract_path_planning_data(
    moving_vehicles: List[AGV],
) -> Tuple[List, List[Tuple[int, int]], List, Dict, List]:
    """
    从移动车辆列表中提取路径规划所需的数据

    Args:
        moving_vehicles: 需要重新规划路径的车辆列表

    Returns:
        Tuple containing: vehicle_ids, maps, starts, ends, root_paths, directions
    """
    if not moving_vehicles:
        return [], [], [], {}, []

    vehicle_ids = []
    maps = []
    starts = []
    ends = []
    root_paths = {}
    directions = []

    for i, vehicle in enumerate(moving_vehicles):
        vehicle_ids.append(vehicle.id)
        maps.append(vehicle.map)
        starts.append(vehicle.loc)
        ends.append(vehicle.end)
        directions.append(vehicle.direction)
        if vehicle.point not in [0, len(vehicle.path) - 1]:
            root_paths[i] = vehicle.path[vehicle.point :]

    # 记录调试信息
    logging.info(f"moving_vehicles: {vehicle_ids}")
    logging.info(f"starts: {starts}")
    logging.info(f"ends: {ends}")
    logging.info(f"directions: {directions}")

    return maps, starts, ends, root_paths, directions


def update_frame_states(vehicle: AGV, agv_states: List[Dict], shelf_states: List[str]) -> None:
    """Record the states of AGVs and shelves for visualization."""
    agv_states.append(
        {
            "id": vehicle.id,
            "x": vehicle.loc[0],
            "y": vehicle.loc[1],
            "direction": repr(vehicle.direction),
            "color": vehicle.color,
            "status": repr(vehicle.status),
            "battery": f"{vehicle.battery / FULL_CHARGE:.1%}",
            "target": vehicle.target,
        }
    )
    if vehicle.delivery_missions:
        shelf_color = "w" if vehicle.color == "y" else "y"
        last_delivery_mission = vehicle.delivery_missions[-1]
        shelf_states[last_delivery_mission.shelf.id] = shelf_color


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
    """Run warehouse automation simulation with AGVs and order fulfillment.

    Simulates a Kiva-style warehouse system where Automated Guided Vehicles (AGVs)
    navigate through the warehouse to fulfill orders by picking up shelves, moving
    them to work stations for sorting, and returning them to their original positions.
    The simulation uses Conflict-Based Search (CBS) for multi-agent path planning
    and tracks various performance metrics.

    Args:
        agv_num: Number of AGVs to deploy in the simulation.
        order_num: Number of orders to be processed during the simulation.
        interval: Time interval between animation frames in milliseconds. Used only
            when show=True. Defaults to 200.
        show: Whether to display the animated visualization of the simulation.
            Defaults to False.
        save_fig: Whether to save the animation as a GIF file. Only effective when
            show=True. Defaults to False.
        heat_map: Whether to generate and display a heat map showing AGV movement
            patterns. Defaults to False.
        astar_max_iter: Maximum number of iterations for the A* pathfinding algorithm
            used within CBS. Defaults to 1500.
        cbs_max_iter: Maximum number of iterations for the Conflict-Based Search
            algorithm for multi-agent path planning. Defaults to 1000.
        simu_max_iter: Maximum number of simulation time steps before termination.
            Defaults to 500.
        random_seed: Random seed for reproducible simulation results. If None,
            uses system time. Defaults to None.

    Returns:
        numpy.ndarray: Array containing simulation performance metrics:
            - agv_num: Number of AGVs used
            - order_num: Number of orders processed
            - orders_completed_time: Time steps required to complete all orders
            - revenue_per_hour: Net revenue per hour in simulation currency
            - mean_utility: Average AGV utilization rate (0.0 to 1.0)
            - total_net_revenue: Total net revenue after costs
            - order_revenue: Gross revenue from completed orders
            - agv_cost: Total AGV operational costs
            - charging_station_cost: Total charging station costs
            - worker_cost: Total worker costs at work stations

    Raises:
        BaseException: If the CBS algorithm fails to find a valid path solution
            for the AGVs, indicating an unsolvable conflict situation.

    Example:
        >>> # Run a basic simulation with 5 AGVs and 20 orders
        >>> results = simulation(agv_num=5, order_num=20, show=True)
        >>> print(f"Completion time: {results[2]} seconds")
        >>> print(f"AGV utilization: {results[4]:.2%}")

        >>> # Run simulation with animation and heat map
        >>> results = simulation(
        ...     agv_num=8,
        ...     order_num=50,
        ...     show=True,
        ...     save_fig=True,
        ...     heat_map=True,
        ...     random_seed=42
        ... )

    Note:
        The simulation terminates when either:
        - All orders are completed and all AGVs return to start positions
        - The maximum simulation iterations (simu_max_iter) is reached

        Performance metrics are calculated based on configurable cost parameters
        including AGV purchase/maintenance costs, charging station costs, and
        worker wages.
    """
    if random_seed is not None:
        np.random.seed(random_seed)
        logging.info(f"random seed set to {random_seed}")

    # 初始化
    # -------------------------------------------------------------------------------------------------------
    revenue = 0  # 订单完成收益
    GLOBAL_AGV_MAP, tables, vehicles, shelves, charging_stations, orders = init_simu(agv_num, order_num)
    all_orders = set(orders)
    completed_orders = set()
    t = 0  # 时间步
    animation_frames = []  # 仿真信息，用来实现可视化
    orders_completed_time = float("inf")
    simulation_complete = False  # 仿真是否完成

    time_start = time.time()
    # main loop
    # -------------------------------------------------------------------------------------------------------
    while not simulation_complete and t < simu_max_iter:
        agv_states = []  # 包括AGV的位置、方向、颜色（是否正托举货架）、电量
        # 分配订单
        unfinished_orders = all_orders - completed_orders
        for order in unfinished_orders:
            distribute_order(order, vehicles, shelves, tables)

        for vehicle in vehicles:
            # 如果存在agv已分配任务且未启动或已完成任务
            if vehicle.needs_path_renewal:
                logging.info(f"vehicle {vehicle.id} triggered renewing")
                arrived_at_start = any(v.status == AgvStatus.ARRIVED_AT_START for v in vehicles)
                moving_vehicles, additional_constraints = get_moving_vehicles(vehicles, GLOBAL_AGV_MAP)
                maps, starts, ends, root_paths, directions = extract_path_planning_data(moving_vehicles)
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

                for v, path in zip(moving_vehicles, paths):
                    v.updates_path(path)
                break

        shelf_states = ["y"] * SHELF_NUM  # 货架颜色信息
        for vehicle in vehicles:
            num_idle_vehicles = sum(agv.status == AgvStatus.WAITING_AT_START for agv in vehicles)
            is_last_to_return = num_idle_vehicles >= agv_num - 1
            num_agv_back_to_start = sum(agv.status == AgvStatus.BACK_TO_START for agv in vehicles)
            allowed_to_return = num_agv_back_to_start <= agv_num // 2

            vehicle.meta_updates(
                tables, orders, GLOBAL_AGV_MAP, charging_stations, is_last_to_return, allowed_to_return, revenue
            )

            update_frame_states(vehicle, agv_states, shelf_states)

        completed_orders = set(order for order in orders if order.status == OrderStatus.DONE)
        num_orders_completed = len(completed_orders)

        frame_data = {
            "agv_states": agv_states,
            "shelf_states": shelf_states,
            "orders_completed": num_orders_completed,
            "t": t,
            "revenue": revenue,
        }
        animation_frames.append(frame_data)
        logging.info(f"orders_completed: {completed_orders}")
        logging.info("-" * 80)
        if num_orders_completed == order_num:
            orders_completed_time = min(orders_completed_time, t)

        simulation_complete = (num_orders_completed == order_num) and all(
            vehicle.status == AgvStatus.WAITING_AT_START for vehicle in vehicles
        )
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
    logging.info(f"revenue per hour: {revenue_per_hour:.1f}")
    heat_map_data = np.zeros(MAP.shape)  # 热力图矩阵
    utilized_time = [0] * agv_num  # 各AGV被利用的时间步数
    for frame in animation_frames:
        for id, agv_state in enumerate(frame["agv_states"]):
            heat_map_data[agv_state["x"], agv_state["y"]] += 1
            if agv_state["status"] in [
                AgvStatus.TO_SHELF,
                AgvStatus.TO_SELECT,
                AgvStatus.SELECTING,
                AgvStatus.RETURN_SHELF,
            ]:
                utilized_time[id] += 1
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
        ani, fps = create_animation(
            MAP,
            animation_frames,
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
            WORKER_COST * TABLE_NUM * orders_completed_time,  # 工作台工人成本
        ]
    )
