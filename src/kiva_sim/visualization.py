import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from kiva_sim.maps import MAP

SHELF_COORDS = MAP.shelf_coords
TABLE_COORDS = MAP.table_coords
CHARGING_STATION_COORD = MAP.charging_station_coords
TABLE_NUM = len(TABLE_COORDS)
CHARGING_STATION_NUM = len(CHARGING_STATION_COORD)


def create_animation(
    map_grid,
    simInfo,
    order_num,
    cost_agv,
    cost_charging_station,
    cost_worker,
    table_num,
    order_complete_time,
    interval,
    SAVE_GIF=False,
):
    """
    功能：生成仿真动画
    :param map_grid: 2d-np.array，栅格地图矩阵（0代表可通行，1代表货架，2代表工作台）
    :param simInfo: list of lists of tuples， 仿真信息列表
    :param order_num: int，订单数量
    :param agv_num: int，AGV数量
    :param cost_agv: int，AGV的单台购入及维护成本
    :param cost_worker: int，分拣工人的时薪
    :param table_num: int，工作台数量
    :param revenue: float，订单收益
    :return: ani
    """
    if SAVE_GIF:
        BLOCK_SIZE = 2500  # 用来调节货架和AGV在地上的大小
        FONT_SIZE = 18
    else:
        BLOCK_SIZE = 400
        FONT_SIZE = 12

    fig, ax = plt.subplots(dpi=100, figsize=(map_grid.shape[1], map_grid.shape[0]))
    table_x = []  # 工作台x坐标列表
    table_y = []  # 工作台y坐标列表
    charging_station_x = []
    charging_station_y = []
    for i in range(TABLE_NUM):
        table_x.append(TABLE_COORDS[i][0] - 0.5)  # 减去0.5是为了将散点显示在栅格中间
        table_y.append(TABLE_COORDS[i][1] - 0.5)
        ax.scatter(table_y[i], table_x[i], s=80, c="white", marker=(f"${i}$"), zorder=2)
    for i in range(CHARGING_STATION_NUM):
        charging_station_x.append(CHARGING_STATION_COORD[i][0] - 0.5)
        charging_station_y.append(CHARGING_STATION_COORD[i][1] - 0.5)
        ax.scatter(charging_station_y[i], charging_station_x[i], s=80, c="white", marker=(f"${i}$"), zorder=2)

    plt.xlim(-1, map_grid.shape[1] - 1)  # 将map的列数作为图中的x坐标
    plt.ylim(map_grid.shape[0] - 1, -1)  # 将map的行数作为图中的y坐标
    my_x_ticks = np.arange(0, map_grid.shape[1], 1)  # x轴刻度
    my_y_ticks = np.arange(0, map_grid.shape[0], 1)  # y轴刻度
    plt.xticks(my_x_ticks)
    plt.yticks(my_y_ticks)
    plt.grid(True)  # 开启栅格
    plt.scatter(table_y, table_x, s=BLOCK_SIZE, c="r", marker="s", label="table")
    plt.scatter(charging_station_y, charging_station_x, s=BLOCK_SIZE, c="green", marker="s", label="charging_station")

    # 动画初始化
    # -------------------------------------------------------------------------------------------------------
    t = len(simInfo)
    agv_num = len(simInfo[0]["AGVInfo"])
    ax.set_title(
        f"{order_num} orders completed with {agv_num} agvs with {order_complete_time} seconds",
        fontsize=FONT_SIZE,
    )
    # 货架
    x_shelf = [coord[0] - 0.5 for coord in SHELF_COORDS]
    y_shelf = [coord[1] - 0.5 for coord in SHELF_COORDS]
    sc_shelf = ax.scatter(y_shelf, x_shelf, s=BLOCK_SIZE, c="y", marker="s", label="shelf")
    # 货架id
    for i in range(len(SHELF_COORDS)):
        ax.text(y_shelf[i], x_shelf[i], str(i), ha="center", va="center", c="white", zorder=2, fontweight="bold")

    # AGV本体
    x_init = [row["x"] - 0.5 for row in simInfo[0]["AGVInfo"]]
    y_init = [row["y"] - 0.5 for row in simInfo[0]["AGVInfo"]]
    sc_position = ax.scatter(y_init, x_init, s=BLOCK_SIZE, c="k", marker="s", label="AGV", zorder=3)
    # AGV方向标识
    x_direction_init = [row["x"] - 0.5 + 0.2 for row in simInfo[0]["AGVInfo"]]
    sc_direction = ax.scatter(y_init, x_direction_init, s=80, c="r", marker="s", zorder=4)
    # AGV id
    sc_markers = [
        ax.text(y_init[i], x_init[i], str(i), ha="center", va="center", c="white", zorder=5, fontweight="bold")
        for i in range(agv_num)
    ]

    # 图标
    ax.legend(bbox_to_anchor=(1, 1), loc="upper left", markerscale=0.3, fontsize=FONT_SIZE)
    # 计时
    timestep = ax.text(
        map_grid.shape[1] - 0.8, 2, "time step: 0", ha="left", va="top", fontsize=FONT_SIZE, fontweight="bold"
    )
    # AGV状态
    status_head = ax.text(
        -8,
        4,
        f"{'AGV info':>21}\n{'id':^7}{'status': ^16}{'target':^16}",
        ha="left",
        va="top",
        fontsize=FONT_SIZE,
        fontweight="bold",
    )
    lines = (f"{id:^7}{simInfo[0]['AGVInfo'][id]['status']:^16}" for id in range(agv_num))
    text = "\n".join(lines) + "\n"
    status = ax.text(-8, 5, text, ha="left", va="top", fontsize=FONT_SIZE)

    # AGV目标
    lines = (f"{simInfo[0]['AGVInfo'][i]['target']: ^16}" for i in range(agv_num))
    text = "\n".join(lines) + "\n"
    target = ax.text(-4, 5, text, ha="left", va="top", fontsize=FONT_SIZE)
    # 电量
    charge_head = ax.text(
        47.2, 4, f"AGV info\n{'id':^7} battery", ha="left", va="top", fontsize=FONT_SIZE, fontweight="bold"
    )
    lines = (f"{i:^7}{simInfo[0]['AGVInfo'][i]['battery']}" for i in range(agv_num))
    text = "\n".join(lines) + "\n"
    battery = ax.text(map_grid.shape[1] - 0.8, 5, text, ha="left", va="top", fontsize=FONT_SIZE)
    # 订单完成数量
    orders_completed = ax.text(
        map_grid.shape[1] - 0.8,
        3,
        f"order completed: {simInfo[0]["orders_completed"]} / {order_num}",
        ha="left",
        va="top",
        fontsize=FONT_SIZE,
        fontweight="bold",
    )
    # 成本
    cost_head = ax.text(-8, -1, "revenue and cost", ha="left", va="top", fontsize=FONT_SIZE, fontweight="bold")
    text_cost = "revenue: 0\ncost_agv: 0\ncost_charging_station: 0\ncost_workers: 0\ntotal net revenue: 0"
    cost = ax.text(-8, 0, text_cost, ha="left", va="top", fontsize=FONT_SIZE)

    # -------------------------------------------------------------------------------------------------------
    # 更新函数
    def update(sim_info):
        x = [row["x"] - 0.5 for row in sim_info["AGVInfo"]]
        y = [row["y"] - 0.5 for row in sim_info["AGVInfo"]]
        t = sim_info["t"]  # 时间步
        # 更新时间步信息
        timestep.set_text(f"time step: {t}")
        # 更新AGV状态信息
        agv_info = sim_info["AGVInfo"]
        lines = (f"{i: ^7}{agv_info[i]['status']: ^16}" for i in range(agv_num))
        text = "\n".join(lines) + "\n"
        status.set_text(text)
        lines = (f"{i: ^7}{agv_info[i]['battery']}" for i in range(agv_num))
        text = "\n".join(lines) + "\n"
        battery.set_text(text)
        # 更新AGV目标
        lines = (f"{agv_info[i]['target']: ^16}" for i in range(agv_num))
        text = "\n".join(lines) + "\n"
        target.set_text(text)

        # 更新订单完成数量
        orders_completed.set_text(f"order completed: {sim_info["orders_completed"]} / {order_num}")

        # 更新成本信息
        total_net_revenue = (
            sim_info["revenue"] * 0.5
            - (cost_agv * agv_num + cost_charging_station * CHARGING_STATION_NUM) * t
            - cost_worker * table_num * min(t, order_complete_time)
        )
        text_cost = "revenue: {revenue:.1f}\ncost_agv: {cost_agv:.2f}\ncost_charging_station: {cost_charging_station:.2f}\ncost_workers: {cost_workers:.1f}\ntotal net revenue: {total_net_revenue:.1f}".format(
            revenue=sim_info["revenue"] * 0.5,
            cost_agv=cost_agv * agv_num * t,
            cost_charging_station=cost_charging_station * CHARGING_STATION_NUM * t,
            cost_workers=cost_worker * table_num * min(t, order_complete_time),
            total_net_revenue=total_net_revenue,
        )
        cost.set_text(text_cost)

        AGV_color = [row["color"] for row in sim_info["AGVInfo"]]
        shelf_color = sim_info["shelfInfo"]

        x_direction, y_direction = x.copy(), y.copy()
        direction2offset = {"up": (-1, 0), "right": (0, 1), "down": (1, 0), "left": (0, -1)}
        offset_value = 0.18
        for i in range(agv_num):
            agv_direction = sim_info["AGVInfo"][i]["direction"]
            dx, dy = direction2offset[agv_direction]
            x_direction[i] += dx * offset_value
            y_direction[i] += dy * offset_value
            # 更新AGV id
            sc_markers[i].set_position((y[i], x[i]))
        sc_position.set_offsets(np.c_[y, x])  # 更新AGV位置
        sc_position.set_color(AGV_color)  # 更新AVG颜色
        sc_shelf.set_color(shelf_color)  # 更新货架颜色
        sc_direction.set_offsets(np.c_[y_direction, x_direction])  # 更新AGV方向

    ani = FuncAnimation(fig, update, frames=simInfo, interval=interval, repeat=False, cache_frame_data=False)  # type: ignore
    fps = 1000 / interval
    return ani, fps


if __name__ == "__main__":
    i = 5
    print(f"{i:<10}sdf")
