import matplotlib.pyplot as plt
import numpy as np
from mapAndOrder import SHELF_COORD, TABLE_COORD, CHARGING_STATION_COORD, TABLE_NUM, CHARGING_STATION_NUM
from matplotlib.animation import FuncAnimation

def create_animation(map_grid, simInfo, order_num, cost_agv, cost_charging_station, cost_worker, table_num, order_complete_time, interval, SAVE_GIF = False):
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
        table_x.append(TABLE_COORD[i][0] - 0.5)  # 减去0.5是为了将散点显示在栅格中间，更美观
        table_y.append(TABLE_COORD[i][1] - 0.5)
        ax.scatter(table_y[i], table_x[i], s=80, c='white', marker=('$' + str(i) + '$'), zorder=2)
    for i in range(CHARGING_STATION_NUM):
        charging_station_x.append(CHARGING_STATION_COORD[i][0] - 0.5)  # 减去0.5是为了将散点显示在栅格中间，更美观
        charging_station_y.append(CHARGING_STATION_COORD[i][1] - 0.5)
        ax.scatter(charging_station_y[i], charging_station_x[i], s=80, c='white', marker=('$' + str(i) + '$'), zorder=2)

    plt.xlim(-1, map_grid.shape[1] - 1)  # 将map的列数作为图中的x坐标
    plt.ylim(map_grid.shape[0]-1, -1)  # 将map的行数作为图中的y坐标
    my_x_ticks = np.arange(0, map_grid.shape[1], 1)  # x轴刻度
    my_y_ticks = np.arange(0, map_grid.shape[0], 1)  # y轴刻度
    plt.xticks(my_x_ticks)
    plt.yticks(my_y_ticks)
    plt.grid(True)  # 开启栅格
    plt.scatter(table_y, table_x, s=BLOCK_SIZE, c='r', marker='s', label='table')
    plt.scatter(charging_station_y, charging_station_x, s=BLOCK_SIZE, c='green', marker='s', label='charging_station')




    # 动画初始化
    # -------------------------------------------------------------------------------------------------------
    t = len(simInfo)
    agv_num = len(simInfo[0]['AGVInfo'])
    ax.set_title('%d orders completed with %d agvs with %d seconds' % (order_num, agv_num, order_complete_time), fontsize=FONT_SIZE)  # 标题
    # 货架
    x_shelf = [coord[0] - 0.5 for coord in SHELF_COORD]
    y_shelf = [coord[1] - 0.5 for coord in SHELF_COORD]
    sc_shelf = ax.scatter(y_shelf, x_shelf, s=BLOCK_SIZE, c='y', marker='s', label='shelf')
    # 货架id
    for i in range(len(SHELF_COORD)):
        ax.text(y_shelf[i], x_shelf[i], str(i), ha='center', va='center', c='white', zorder=2, fontweight='bold')


    # AGV本体
    x_init = [row['x'] - 0.5 for row in simInfo[0]['AGVInfo']]
    y_init = [row['y'] - 0.5 for row in simInfo[0]['AGVInfo']]
    sc_position = ax.scatter(y_init, x_init, s=BLOCK_SIZE, c='k', marker='s', label='AGV', zorder=3)
    # AGV方向标识
    x_direction_init = [row['x'] - 0.5 + 0.2 for row in simInfo[0]['AGVInfo']]
    sc_direction = ax.scatter(y_init, x_direction_init, s=80, c='r', marker='s', zorder=4)
    # AGV id
    sc_markers = []
    for i in range(agv_num):
        sc_markers.append(ax.text(y_init[i], x_init[i], str(i), ha='center', va='center', c='white', zorder=5, fontweight='bold'))

    # 图标
    ax.legend(bbox_to_anchor=(1, 1), loc="upper left", markerscale=0.3, fontsize=FONT_SIZE)
    # 计时
    timestep = ax.text(map_grid.shape[1] - 0.8, 2, 'time step: 0', ha='left', va='top', fontsize=FONT_SIZE, fontweight='bold')
    # AGV状态
    status_head = ax.text(-8, 4, '             AGV info\nid     status                    target', ha='left', va='top', fontsize=FONT_SIZE, fontweight='bold')
    text = ""
    for i in range(agv_num):
        text += "%d"%i + (6 - len("%d"%i)) * " " + simInfo[0]['AGVInfo'][i]['status'] + "\n"
    status = ax.text(-8, 5, text, ha='left', va='top', fontsize=FONT_SIZE)

    # AGV目标
    text = ""
    for i in range(agv_num):
        text += simInfo[0]['AGVInfo'][i]['target'] + "\n"
    target = ax.text(-4, 5, text, ha='left', va='top', fontsize=FONT_SIZE)
    # 电量
    charge_head = ax.text(47.2, 4, 'AGV info\nid battery', ha='left', va='top', fontsize=FONT_SIZE, fontweight='bold')
    text = ""
    for i in range(agv_num):
        text += "%d"%i + (5 - len("%d"%i)) * " " + simInfo[0]['AGVInfo'][i]['battery'] + "\n"
    battery = ax.text(map_grid.shape[1] - 0.8, 5, text, ha='left', va='top', fontsize=FONT_SIZE)
    # 订单完成数量
    order_complete = ax.text(map_grid.shape[1] - 0.8, 3, 'order completed: %d / %d'%(simInfo[0]['order_complete'], order_num), ha='left', va='top', fontsize=FONT_SIZE, fontweight='bold')
    # 成本
    cost_head = ax.text(-8, -1, 'revenue and cost', ha='left', va='top', fontsize=FONT_SIZE, fontweight='bold')
    text_cost = 'revenue: 0\ncost_agv: 0\ncost_charging_station: 0\ncost_workers: 0\ntotal net revenue: 0'
    cost = ax.text(-8, 0, text_cost, ha='left', va='top', fontsize=FONT_SIZE)
    # -------------------------------------------------------------------------------------------------------
    # 更新函数
    def update(simInfo):
        x = [row['x'] - 0.5 for row in simInfo['AGVInfo']]
        y = [row['y'] - 0.5 for row in simInfo['AGVInfo']]
        t = simInfo['t']  # 时间步
        # 更新时间步信息
        timestep.set_text('time step: ' + str(t))
        # 更新AGV状态信息
        text = ""
        for i in range(agv_num):
            text += "%d" % i + (6 - len("%d" % i)) * " " + simInfo['AGVInfo'][i]['status'] + "\n"
        status.set_text(text)
        text = ""
        for i in range(agv_num):
            text += "%d"%i+ (5 - len("%d"%i)) * " " + simInfo['AGVInfo'][i]['battery'] + "\n"
        battery.set_text(text)
        # 更新AGV目标
        text = ""
        for i in range(agv_num):
            text += simInfo['AGVInfo'][i]['target'] + "\n"
        target.set_text(text)

        # 更新订单完成数量
        order_complete.set_text('order completed: %d / %d' % (simInfo['order_complete'], order_num))


        # 更新成本信息
        total_net_revenue = simInfo['revenue'] * 0.5 - (cost_agv * agv_num + cost_charging_station * CHARGING_STATION_NUM) * t - cost_worker * table_num * min(t, order_complete_time)
        text_cost = 'revenue: %.1f\ncost_agv: %.2f\ncost_charging_station: %.2f\ncost_workers: %.1f\ntotal net revenue: %.1f' % (
        simInfo['revenue'] * 0.5, cost_agv * agv_num * t, cost_charging_station * CHARGING_STATION_NUM * t, cost_worker * table_num * min(t, order_complete_time),
        total_net_revenue)
        cost.set_text(text_cost)


        AGV_color = [row['color'] for row in simInfo['AGVInfo']]
        shelf_color = simInfo['shelfInfo']

        x_direction = x.copy()
        y_direction = y.copy()

        for i in range(len(simInfo['AGVInfo'])):
            if simInfo['AGVInfo'][i]['direction'] == 'up':
                x_direction[i] -= 0.2
            if simInfo['AGVInfo'][i]['direction'] == 'down':
                x_direction[i] += 0.2
            if simInfo['AGVInfo'][i]['direction'] == 'left':
                y_direction[i] -= 0.2
            if simInfo['AGVInfo'][i]['direction'] == 'right':
                y_direction[i] += 0.2
        sc_position.set_offsets(np.c_[y, x])  # 更新AGV位置
        sc_position.set_color(AGV_color)  # 更新AVG颜色
        sc_shelf.set_color(shelf_color)  # 更新货架颜色
        sc_direction.set_offsets(np.c_[y_direction, x_direction])  # 更新AGV方向
        # 更新AGV id
        for i in range(agv_num):
            sc_markers[i].set_position((y[i], x[i]))

    ani = FuncAnimation(fig, update, frames=simInfo, interval=interval, repeat=False, cache_frame_data=False)  # 创建动画效果
    fps = 1000 / interval
    return ani, fps

