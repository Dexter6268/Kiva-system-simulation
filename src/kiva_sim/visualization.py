import numpy as np
import tkinter as tk
from typing import List, Dict, Tuple
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from kiva_sim.maps import Map


def get_optimal_figsize(map_shape, save_gif=False, margin_factor=0.9):
    """
    根据地图比例和屏幕尺寸计算最优的figure尺寸

    Args:
        map: 地图网格
        save_gif: 是否为保存GIF（影响质量设置）
        margin_factor: 屏幕利用率（0.9表示使用90%的屏幕空间）

    Returns:
        (width, height): figure尺寸，单位英寸
    """
    try:
        # 获取屏幕尺寸
        root = tk.Tk()
        screen_width_px = root.winfo_screenwidth()
        screen_height_px = root.winfo_screenheight()
        screen_dpi = root.winfo_fpixels("1i")  # 获取屏幕DPI
        root.destroy()

        # 转换为英寸
        screen_width_inch = screen_width_px / screen_dpi
        screen_height_inch = screen_height_px / screen_dpi

        # 考虑边距（为菜单栏、工具栏等留空间）
        usable_width = screen_width_inch * margin_factor
        usable_height = screen_height_inch * margin_factor

    except:
        print("无法获取屏幕信息，使用默认值")
        usable_width = 12
        usable_height = 8

    # 地图的宽高比
    map_width, map_height = map_shape
    map_aspect_ratio = map_width / map_height

    # 根据地图比例计算最优尺寸
    if map_aspect_ratio > (usable_width / usable_height):
        # 地图较宽，以宽度为准
        fig_width = usable_width
        fig_height = fig_width / map_aspect_ratio
    else:
        # 地图较高，以高度为准
        fig_height = usable_height
        fig_width = fig_height * map_aspect_ratio

    # 限制最小和最大尺寸
    min_size = 6 if not save_gif else 8
    max_size = 20 if save_gif else 16

    fig_width = max(min_size, min(fig_width, max_size))
    fig_height = max(min_size * 0.75, min(fig_height, max_size))

    return (fig_width, fig_height)


def get_precise_block_size(ax):
    """
    基于实际坐标轴范围计算精确的 block_size

    Args:
        ax: matplotlib 轴对象
        map_shape: (width, height) 地图形状

    Returns:
        int: 精确的 block_size
    """
    # 获取轴的边界框（以像素为单位）
    bbox = ax.get_window_extent()

    # 轴的像素尺寸
    axis_width_px = bbox.width
    axis_height_px = bbox.height

    # 数据坐标范围
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    data_width = xlim[1] - xlim[0]
    data_height = ylim[0] - ylim[1]  # ylim 是反向的

    # 每个数据单位对应的像素数
    pixels_per_unit_x = axis_width_px / data_width
    pixels_per_unit_y = axis_height_px / data_height

    # 每个网格的像素大小（取较小值保证正方形）
    pixels_per_grid = min(pixels_per_unit_x, pixels_per_unit_y)

    # 转换为 matplotlib 的 points²
    # 在 matplotlib 中，1 个数据单位 = 72 points
    fig = ax.get_figure()
    dpi = fig.dpi
    points_per_pixel = 72 / dpi
    points_per_grid = pixels_per_grid * points_per_pixel

    # 散点大小是面积
    block_size = points_per_grid**2

    return int(block_size)


def create_animation_old(
    map: Map,
    sim_info: List[Dict],
    order_num: int,
    cost_agv: float,
    cost_charging_station: float,
    cost_worker: float,
    table_num: int,
    order_complete_time: int,
    interval: int,
    SAVE_GIF=False,
) -> Tuple[FuncAnimation, int]:
    """
    功能：生成仿真动画
    :param map: 2d-np.array，栅格地图矩阵（0代表可通行，1代表货架，2代表工作台）
    :param sim_info: list of lists of tuples， 仿真信息列表
    :param order_num: int，订单数量
    :param agv_num: int，AGV数量
    :param cost_agv: int，AGV的单台购入及维护成本
    :param cost_worker: int，分拣工人的时薪
    :param table_num: int，工作台数量
    :param revenue: float，订单收益
    :return: ani
    """
    FONT_SIZE = 18 if SAVE_GIF else 12

    shelf_coords = np.array(map.shelf_coords)
    table_coords = map.table_coords
    charging_station_coords = map.charging_station_coords
    table_num = len(table_coords)
    shelf_num = len(shelf_coords)
    charging_station_num = len(charging_station_coords)

    figsize = get_optimal_figsize(map.shape[::-1], save_gif=SAVE_GIF)
    fig, ax = plt.subplots(dpi=100, figsize=figsize)
    table_x = []  # 工作台x坐标列表
    table_y = []  # 工作台y坐标列表
    charging_station_x = []
    charging_station_y = []
    for i in range(table_num):
        table_x.append(table_coords[i][0] - 0.5)  # 减去0.5是为了将散点显示在栅格中间
        table_y.append(table_coords[i][1] - 0.5)
        ax.scatter(table_y[i], table_x[i], s=80, c="white", marker=(f"${i}$"), zorder=2)
    for i in range(charging_station_num):
        charging_station_x.append(charging_station_coords[i][0] - 0.5)
        charging_station_y.append(charging_station_coords[i][1] - 0.5)
        ax.scatter(charging_station_y[i], charging_station_x[i], s=80, c="white", marker=(f"${i}$"), zorder=2)

    plt.xlim(-1, map.shape[1] - 1)  # 将map的列数作为图中的x坐标
    plt.ylim(map.shape[0] - 1, -1)  # 将map的行数作为图中的y坐标
    my_x_ticks = np.arange(0, map.shape[1], 1)  # x轴刻度
    my_y_ticks = np.arange(0, map.shape[0], 1)  # y轴刻度
    plt.xticks(my_x_ticks)
    plt.yticks(my_y_ticks)
    plt.grid(True)  # 开启栅格
    plt.gca().set_aspect("equal", adjustable="box")  # 设置x轴和y轴比例相同

    # 渲染一次以获取精确的轴尺寸
    fig.canvas.draw()
    BLOCK_SIZE = get_precise_block_size(ax)

    sc_table = plt.scatter(table_y, table_x, s=BLOCK_SIZE, c="r", marker="s", label="table")
    sc_charging = plt.scatter(
        charging_station_y, charging_station_x, s=BLOCK_SIZE, c="green", marker="s", label="charging_station"
    )

    # 动画初始化
    # -------------------------------------------------------------------------------------------------------
    agv_num = len(sim_info[0]["agv_states"])
    ax.set_title(
        f"{order_num} orders completed with {agv_num} agvs with {order_complete_time} seconds",
        fontsize=FONT_SIZE,
    )
    # 货架
    x_shelf = shelf_coords[:, 0] - 0.5
    y_shelf = shelf_coords[:, 1] - 0.5
    sc_shelf = ax.scatter(y_shelf, x_shelf, s=BLOCK_SIZE, c="y", marker="s", label="shelf")
    # 货架id
    for i in range(shelf_num):
        ax.text(y_shelf[i], x_shelf[i], str(i), ha="center", va="center", c="white", zorder=2, fontweight="bold")

    # AGV本体
    x_init = [row["x"] - 0.5 for row in sim_info[0]["agv_states"]]
    y_init = [row["y"] - 0.5 for row in sim_info[0]["agv_states"]]
    sc_position = ax.scatter(y_init, x_init, s=BLOCK_SIZE, c="k", marker="s", label="AGV", zorder=3)
    # AGV方向标识
    offset_value = 0.2
    x_direction_init = [row["x"] - 0.5 + offset_value for row in sim_info[0]["agv_states"]]
    sc_direction = ax.scatter(y_init, x_direction_init, s=BLOCK_SIZE * 0.25, c="r", marker="s", zorder=4)
    # AGV id
    sc_markers = [
        ax.text(y_init[i], x_init[i], str(i), ha="center", va="center", c="white", zorder=5, fontweight="bold")
        for i in range(agv_num)
    ]

    # 图标
    ax.legend(bbox_to_anchor=(1, 1), loc="upper left", markerscale=0.3, fontsize=FONT_SIZE)
    # 计时
    timestep = ax.text(
        map.shape[1] - 0.8, 2, "time step: 0", ha="left", va="top", fontsize=FONT_SIZE, fontweight="bold"
    )
    # AGV状态
    agv_status_title = f"{'AGV info':>21}\n{'id':^7}{'status': ^16}{'target':^16}"
    ax.text(-8, 4, agv_status_title, ha="left", va="top", fontsize=FONT_SIZE, fontweight="bold")

    lines = (f"{id:^7}{sim_info[0]['agv_states'][id]['status']:^16}" for id in range(agv_num))
    text = "\n".join(lines) + "\n"
    status = ax.text(-8, 5, text, ha="left", va="top", fontsize=FONT_SIZE)

    # AGV目标
    lines = (f"{sim_info[0]['agv_states'][i]['target']: ^16}" for i in range(agv_num))
    text = "\n".join(lines) + "\n"
    target = ax.text(-4, 5, text, ha="left", va="top", fontsize=FONT_SIZE)

    # 电量
    ax.text(47.2, 4, f"AGV info\n{'id':^7} battery", ha="left", va="top", fontsize=FONT_SIZE, fontweight="bold")
    lines = (f"{i:^7}{sim_info[0]['agv_states'][i]['battery']}" for i in range(agv_num))
    text = "\n".join(lines) + "\n"
    battery = ax.text(map.shape[1] - 0.8, 5, text, ha="left", va="top", fontsize=FONT_SIZE)

    # 订单完成数量
    order_completed_text = f"order completed: {sim_info[0]["orders_completed"]} / {order_num}"
    orders_completed = ax.text(
        map.shape[1] - 0.8, 3, order_completed_text, ha="left", va="top", fontsize=FONT_SIZE, fontweight="bold"
    )

    # 成本
    ax.text(-8, -1, "revenue and cost", ha="left", va="top", fontsize=FONT_SIZE, fontweight="bold")
    text_cost = "revenue: 0\ncost_agv: 0\ncost_charging_station: 0\ncost_workers: 0\ntotal net revenue: 0"
    cost = ax.text(-8, 0, text_cost, ha="left", va="top", fontsize=FONT_SIZE)

    # 添加窗口大小变化事件监听
    def on_resize(event):
        """窗口大小变化时更新 block_size"""
        if event.canvas == fig.canvas:
            # 重新计算 block_size
            new_block_size = get_precise_block_size(ax)
            new_direction_size = new_block_size * 0.25
            # 更新所有散点图的大小
            sc_table.set_sizes([new_block_size] * table_num)
            sc_charging.set_sizes([new_block_size] * charging_station_num)
            sc_shelf.set_sizes([new_block_size] * shelf_num)
            sc_position.set_sizes([new_block_size] * agv_num)
            sc_direction.set_sizes([new_direction_size] * agv_num)
            # 重新绘制
            fig.canvas.draw_idle()

    # 连接事件
    fig.canvas.mpl_connect("resize_event", on_resize)

    # -------------------------------------------------------------------------------------------------------
    # 更新函数
    def update(sim_info):
        x = [row["x"] - 0.5 for row in sim_info["agv_states"]]
        y = [row["y"] - 0.5 for row in sim_info["agv_states"]]
        t = sim_info["t"]  # 时间步
        # 更新时间步信息
        timestep.set_text(f"time step: {t}")
        # 更新AGV状态信息
        agv_states = sim_info["agv_states"]
        lines = (f"{i: ^7}{agv_states[i]['status']: ^16}" for i in range(agv_num))
        text = "\n".join(lines) + "\n"
        status.set_text(text)
        lines = (f"{i: ^7}{agv_states[i]['battery']}" for i in range(agv_num))
        text = "\n".join(lines) + "\n"
        battery.set_text(text)
        # 更新AGV目标
        lines = (f"{agv_states[i]['target']: ^16}" for i in range(agv_num))
        text = "\n".join(lines) + "\n"
        target.set_text(text)

        # 更新订单完成数量
        orders_completed.set_text(f"order completed: {sim_info["orders_completed"]} / {order_num}")

        # 更新成本信息
        total_net_revenue = (
            sim_info["revenue"] * 0.5
            - (cost_agv * agv_num + cost_charging_station * charging_station_num) * t
            - cost_worker * table_num * min(t, order_complete_time)
        )
        text_cost = "revenue: {revenue:.1f}\ncost_agv: {cost_agv:.2f}\ncost_charging_station: {cost_charging_station:.2f}\ncost_workers: {cost_workers:.1f}\ntotal net revenue: {total_net_revenue:.1f}".format(
            revenue=sim_info["revenue"] * 0.5,
            cost_agv=cost_agv * agv_num * t,
            cost_charging_station=cost_charging_station * charging_station_num * t,
            cost_workers=cost_worker * table_num * min(t, order_complete_time),
            total_net_revenue=total_net_revenue,
        )
        cost.set_text(text_cost)

        AGV_color = [row["color"] for row in sim_info["agv_states"]]
        shelf_color = sim_info["shelf_states"]

        x_direction, y_direction = x.copy(), y.copy()
        direction2offset = {"up": (-1, 0), "right": (0, 1), "down": (1, 0), "left": (0, -1)}
        for i in range(agv_num):
            agv_direction = sim_info["agv_states"][i]["direction"]
            dx, dy = direction2offset[agv_direction]
            x_direction[i] += dx * offset_value
            y_direction[i] += dy * offset_value
            # 更新AGV id
            sc_markers[i].set_position((y[i], x[i]))
        sc_position.set_offsets(np.c_[y, x])  # 更新AGV位置
        sc_position.set_color(AGV_color)  # 更新AVG颜色
        sc_shelf.set_color(shelf_color)  # 更新货架颜色
        sc_direction.set_offsets(np.c_[y_direction, x_direction])  # 更新AGV方向

    ani = FuncAnimation(fig, update, frames=sim_info, interval=interval, repeat=False, cache_frame_data=False)  # type: ignore
    fps = 1000 // interval
    return ani, fps


def create_animation(
    map: Map,
    sim_info: List[Dict],
    order_num: int,
    cost_agv: float,
    cost_charging_station: float,
    cost_worker: float,
    table_num: int,
    order_complete_time: int,
    interval: int,
    SAVE_GIF=False,
) -> Tuple[FuncAnimation, int]:
    """生成仿真动画"""
    # 提取地图信息
    coords_info = _extract_map_coordinates(map)

    # 创建图形和轴
    fig, ax = _setup_figure_and_axes(map, SAVE_GIF)

    # 绘制静态元素
    scatter_objects = _draw_static_elements(ax, coords_info, SAVE_GIF)

    # 初始化动态元素
    agv_num = len(sim_info[0]["agv_states"])
    dynamic_elements = _initialize_dynamic_elements(
        ax,
        map,
        coords_info["shelf_coords"],
        sim_info[0],
        agv_num,
        order_num,
        order_complete_time,
        scatter_objects["block_size"],
        SAVE_GIF,
    )

    # 设置窗口调整事件
    _setup_resize_handler(fig, ax, scatter_objects, dynamic_elements, coords_info, agv_num)

    # 创建更新函数
    update_func = _create_update_function(
        dynamic_elements,
        cost_agv,
        cost_charging_station,
        cost_worker,
        table_num,
        order_complete_time,
        agv_num,
        order_num,
    )

    # 创建动画
    ani = FuncAnimation(fig, update_func, frames=sim_info, interval=interval, repeat=False, cache_frame_data=False)  # type: ignore
    fps = 1000 // interval
    return ani, fps


def _extract_map_coordinates(map: Map) -> Dict:
    """提取地图坐标信息"""
    shelf_coords = np.array(map.shelf_coords)
    table_coords = np.array(map.table_coords)
    charging_station_coords = np.array(map.charging_station_coords)

    return {
        "shelf_coords": shelf_coords,
        "table_coords": table_coords,
        "charging_station_coords": charging_station_coords,
        "shelf_num": len(shelf_coords),
        "table_num": len(table_coords),
        "charging_station_num": len(charging_station_coords),
    }


def _setup_figure_and_axes(map: Map, save_gif: bool) -> Tuple:
    """设置图形和坐标轴"""
    figsize = get_optimal_figsize(map.shape[::-1], save_gif=save_gif)
    fig, ax = plt.subplots(dpi=100, figsize=figsize)

    # 设置坐标轴
    plt.xlim(-1, map.shape[1] - 1)
    plt.ylim(map.shape[0] - 1, -1)

    # 设置刻度
    my_x_ticks = np.arange(0, map.shape[1], 1)
    my_y_ticks = np.arange(0, map.shape[0], 1)
    plt.xticks(my_x_ticks)
    plt.yticks(my_y_ticks)
    plt.grid(True)
    plt.gca().set_aspect("equal", adjustable="box")

    # 渲染一次以获取精确的轴尺寸
    fig.canvas.draw()

    return fig, ax


def _draw_static_elements(ax, coords_info: Dict, save_gif: bool) -> Dict:
    """绘制静态元素（工作台、充电站、货架）"""
    block_size = get_precise_block_size(ax)
    font_size = 18 if save_gif else 12

    # 绘制工作台
    table_coords = coords_info["table_coords"] - 0.5
    table_x, table_y = table_coords[:, 0], table_coords[:, 1]
    sc_table = ax.scatter(table_y, table_x, s=block_size, c="r", marker="s", label="table")

    # 添加工作台标签
    for i, (x, y) in enumerate(zip(table_x, table_y)):
        ax.scatter(y, x, s=80, c="white", marker=(f"${i}$"), zorder=2)

    # 绘制充电站
    charging_coords = coords_info["charging_station_coords"] - 0.5
    charging_x, charging_y = charging_coords[:, 0], charging_coords[:, 1]
    sc_charging = ax.scatter(charging_y, charging_x, s=block_size, c="green", marker="s", label="charging_station")

    # 添加充电站标签
    for i, (x, y) in enumerate(zip(charging_x, charging_y)):
        ax.scatter(y, x, s=80, c="white", marker=(f"${i}$"), zorder=2)

    return {"sc_table": sc_table, "sc_charging": sc_charging, "block_size": block_size}


def _initialize_dynamic_elements(
    ax,
    map: Map,
    shelf_coords: np.ndarray,
    initial_sim_info: Dict,
    agv_num: int,
    order_num: int,
    order_complete_time: int,
    block_size: int,
    save_gif: bool,
) -> Dict:
    """初始化动态元素（AGV、文本信息等）"""
    font_size = 18 if save_gif else 12

    # 设置标题
    ax.set_title(
        f"{order_num} orders completed with {agv_num} agvs with {order_complete_time} seconds",
        fontsize=font_size,
    )

    # 初始化AGV位置
    agv_positions = np.array([[row["x"], row["y"]] for row in initial_sim_info["agv_states"]])
    x_init = agv_positions[:, 0] - 0.5
    y_init = agv_positions[:, 1] - 0.5

    # AGV散点图
    sc_position = ax.scatter(y_init, x_init, s=block_size, c="k", marker="s", label="AGV", zorder=3)

    # AGV方向标识
    offset_value = 0.2
    x_direction_init = x_init + offset_value
    sc_direction = ax.scatter(y_init, x_direction_init, s=block_size * 0.25, c="r", marker="s", zorder=4)

    # AGV ID标记
    sc_markers = [
        ax.text(y_init[i], x_init[i], str(i), ha="center", va="center", c="white", zorder=5, fontweight="bold")
        for i in range(agv_num)
    ]

    # 绘制货架
    shelf_coords = shelf_coords - 0.5
    x_shelf, y_shelf = shelf_coords[:, 0], shelf_coords[:, 1]
    sc_shelf = ax.scatter(y_shelf, x_shelf, s=block_size, c="y", marker="s", label="shelf")

    # 添加货架ID
    for i, (x, y) in enumerate(zip(x_shelf, y_shelf)):
        ax.text(y, x, str(i), ha="center", va="center", c="white", zorder=2, fontweight="bold")

    # 图例
    ax.legend(bbox_to_anchor=(1, 1), loc="upper left", markerscale=0.3, fontsize=font_size)

    # 创建文本元素
    text_elements = _create_text_elements(ax, map, initial_sim_info, agv_num, order_num, font_size)

    return {
        "sc_position": sc_position,
        "sc_direction": sc_direction,
        "sc_markers": sc_markers,
        "sc_shelf": sc_shelf,
        "offset_value": offset_value,
        **text_elements,
    }


def _create_text_elements(ax, map: Map, initial_sim_info: Dict, agv_num: int, order_num: int, font_size: int) -> Dict:
    """创建所有文本元素"""
    # 时间步显示
    timestep = ax.text(
        map.shape[1] - 0.8, 2, "time step: 0", ha="left", va="top", fontsize=font_size, fontweight="bold"
    )

    # AGV状态信息
    agv_status_title = f"{'AGV info':>21}\n{'id':^7}{'status': ^16}{'target':^16}"
    ax.text(-8, 4, agv_status_title, ha="left", va="top", fontsize=font_size, fontweight="bold")

    # AGV状态文本
    status_lines = (f"{i:^7}{initial_sim_info['agv_states'][i]['status']:^16}" for i in range(agv_num))
    status_text = "\n".join(status_lines) + "\n"
    status = ax.text(-8, 5, status_text, ha="left", va="top", fontsize=font_size)

    # AGV目标文本
    target_lines = (f"{initial_sim_info['agv_states'][i]['target']: ^16}" for i in range(agv_num))
    target_text = "\n".join(target_lines) + "\n"
    target = ax.text(-4, 5, target_text, ha="left", va="top", fontsize=font_size)

    # 电量信息
    ax.text(47.2, 4, f"AGV info\n{'id':^7} battery", ha="left", va="top", fontsize=font_size, fontweight="bold")
    battery_lines = (f"{i:^7}{initial_sim_info['agv_states'][i]['battery']}" for i in range(agv_num))
    battery_text = "\n".join(battery_lines) + "\n"
    battery = ax.text(map.shape[1] - 0.8, 5, battery_text, ha="left", va="top", fontsize=font_size)

    # 订单完成数量
    order_completed_text = f"order completed: {initial_sim_info['orders_completed']} / {order_num}"
    orders_completed = ax.text(
        map.shape[1] - 0.8, 3, order_completed_text, ha="left", va="top", fontsize=font_size, fontweight="bold"
    )

    # 成本信息
    ax.text(-8, -1, "revenue and cost", ha="left", va="top", fontsize=font_size, fontweight="bold")
    text_cost = "revenue: 0\ncost_agv: 0\ncost_charging_station: 0\ncost_workers: 0\ntotal net revenue: 0"
    cost = ax.text(-8, 0, text_cost, ha="left", va="top", fontsize=font_size)

    return {
        "timestep": timestep,
        "status": status,
        "target": target,
        "battery": battery,
        "orders_completed": orders_completed,
        "cost": cost,
    }


def _setup_resize_handler(fig, ax, scatter_objects: Dict, dynamic_elements: Dict, coords_info: Dict, agv_num: int):
    """设置窗口大小变化事件处理"""

    def on_resize(event):
        if event.canvas == fig.canvas:
            new_block_size = get_precise_block_size(ax)
            new_direction_size = new_block_size * 0.25

            # 更新散点图大小
            scatter_objects["sc_table"].set_sizes([new_block_size] * coords_info["table_num"])
            scatter_objects["sc_charging"].set_sizes([new_block_size] * coords_info["charging_station_num"])
            dynamic_elements["sc_shelf"].set_sizes([new_block_size] * coords_info["shelf_num"])
            dynamic_elements["sc_position"].set_sizes([new_block_size] * agv_num)
            dynamic_elements["sc_direction"].set_sizes([new_direction_size] * agv_num)

            fig.canvas.draw_idle()

    fig.canvas.mpl_connect("resize_event", on_resize)


def _create_update_function(
    dynamic_elements: Dict,
    cost_agv: float,
    cost_charging_station: float,
    cost_worker: float,
    table_num: int,
    order_complete_time: int,
    agv_num: int,
    order_num: int,
):
    """创建动画更新函数"""

    def update(sim_info):
        # 更新AGV位置
        agv_positions = np.array([[row["x"], row["y"]] for row in sim_info["agv_states"]])
        x = agv_positions[:, 0] - 0.5
        y = agv_positions[:, 1] - 0.5

        t = sim_info["t"]

        # 更新文本信息
        _update_text_info(
            dynamic_elements,
            sim_info,
            t,
            agv_num,
            order_num,
            cost_agv,
            cost_charging_station,
            cost_worker,
            table_num,
            order_complete_time,
        )

        # 更新AGV视觉元素
        _update_agv_visuals(dynamic_elements, sim_info, x, y, agv_num)

    return update


def _update_text_info(
    dynamic_elements: Dict,
    sim_info: Dict,
    t: int,
    agv_num: int,
    order_num: int,
    cost_agv: float,
    cost_charging_station: float,
    cost_worker: float,
    table_num: int,
    order_complete_time: int,
):
    """更新所有文本信息"""
    # 更新时间步
    dynamic_elements["timestep"].set_text(f"time step: {t}")

    # 更新AGV状态
    agv_states = sim_info["agv_states"]
    status_lines = (f"{i: ^7}{agv_states[i]['status']: ^16}" for i in range(agv_num))
    dynamic_elements["status"].set_text("\n".join(status_lines) + "\n")

    # 更新AGV目标
    target_lines = (f"{agv_states[i]['target']: ^16}" for i in range(agv_num))
    dynamic_elements["target"].set_text("\n".join(target_lines) + "\n")

    # 更新电量
    battery_lines = (f"{i: ^7}{agv_states[i]['battery']}" for i in range(agv_num))
    dynamic_elements["battery"].set_text("\n".join(battery_lines) + "\n")

    # 更新订单完成数量
    dynamic_elements["orders_completed"].set_text(f"order completed: {sim_info['orders_completed']} / {order_num}")

    # 更新成本信息
    charging_station_num = 4  # 需要从外部传入或计算
    total_net_revenue = (
        sim_info["revenue"] * 0.5
        - (cost_agv * agv_num + cost_charging_station * charging_station_num) * t
        - cost_worker * table_num * min(t, order_complete_time)
    )

    cost_text = (
        f"revenue: {sim_info['revenue'] * 0.5:.1f}\n"
        f"cost_agv: {cost_agv * agv_num * t:.2f}\n"
        f"cost_charging_station: {cost_charging_station * charging_station_num * t:.2f}\n"
        f"cost_workers: {cost_worker * table_num * min(t, order_complete_time):.1f}\n"
        f"total net revenue: {total_net_revenue:.1f}"
    )
    dynamic_elements["cost"].set_text(cost_text)


def _update_agv_visuals(dynamic_elements: Dict, sim_info: Dict, x: np.ndarray, y: np.ndarray, agv_num: int):
    """更新AGV视觉元素"""
    # 更新AGV颜色
    agv_colors = [row["color"] for row in sim_info["agv_states"]]
    shelf_colors = sim_info["shelf_states"]

    # 计算方向偏移
    offset_value = dynamic_elements["offset_value"]
    direction2offset = {"up": (-1, 0), "right": (0, 1), "down": (1, 0), "left": (0, -1)}

    x_direction, y_direction = x.copy(), y.copy()
    for i in range(agv_num):
        agv_direction = sim_info["agv_states"][i]["direction"]
        dx, dy = direction2offset[agv_direction]
        x_direction[i] += dx * offset_value
        y_direction[i] += dy * offset_value

        # 更新AGV ID位置
        dynamic_elements["sc_markers"][i].set_position((y[i], x[i]))

    # 更新散点图
    dynamic_elements["sc_position"].set_offsets(np.c_[y, x])
    dynamic_elements["sc_position"].set_color(agv_colors)
    dynamic_elements["sc_shelf"].set_color(shelf_colors)
    dynamic_elements["sc_direction"].set_offsets(np.c_[y_direction, x_direction])


# ...existing code...
if __name__ == "__main__":
    from kiva_sim.maps import load_map

    map = load_map()
    shelf_coords = map.shelf_coords
    print(np.array(shelf_coords))
