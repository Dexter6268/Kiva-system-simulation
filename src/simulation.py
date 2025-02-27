from mapAndOrder import SHELF_COORD, CHARGING_STATION_COORD, MAP, \
    orderDistribute, shift_table, TABLE_NUM, SHELF_NUM, CHARGING_STATION_NUM
from agv import AGV, FULL_CHARGE
from cbs import cbs, cbs_reserve
import numpy as np
import time
from visualization import create_animation
import matplotlib.pyplot as plt
import seaborn as sns


LIFTING_TIME = 4  # AGV抬起及放下货架时长
AGV_COST = 75000 * 1.05 / 5 / 365 / 24 / 3600  # 每辆AGV的购入及维护成本(极智嘉M200) * 1.05是假设维护成本为购入价格的5%，然后均摊到每秒
CHARGING_STATION_COST = 20000 / 5 / 365 / 24 / 3600  # 每个充电桩的成本为20000元
WORKER_COST = 34 / 3600  # 分拣工人每秒薪资


def simulation(seed, AGV_NUM, ORDER_NUM, order_list, interval=200, show=False, save_fig=False, heat_map=False):
    """
    功能：运行仿真
    :param seed: int，仿真运行的随机数种子
    :param AGV_NUM: int， AGV数量
    :param ORDER_NUM: int，订单数量
    :param order_list: list of Order objects，订单对象列表
    :param interval: int，动画每一帧的时间间隔
    :param show: Boolean，是否生成动画
    :param save_fig: Boolean，是否保存动画
    :param heat_map: Boolean，是否生成热力图
    :return: list， 仿真信息列表
    """

    np.random.seed(seed)
    all_shelf_status = [1] * SHELF_NUM  # 货架状态，在原位为1，否则为0
    # 主函数 仿真+可视化
    # 初始化
    # -------------------------------------------------------------------------------------------------------
    revenue = 0  # 订单完成收益
    maxIteration = 1500  # cbs算法迭代上限
    AGV_MAP = MAP.copy()  # AGV所用地图
    charging_stations = []  # 充电桩列表
    for i in range(CHARGING_STATION_NUM):
        charging_stations.append({'id': i, 'loc': CHARGING_STATION_COORD[i], 'occupied': False})
    table_list = []  # 工作台列表
    for i in range(3 * TABLE_NUM):
        if i % 3 == 2:
            table_list.append(({'id': i, 'table_id': i // 3, 'shift': -1, 'loc': shift_table(i // 3, -1), 'occupied': False}))
        else:
            table_list.append(({'id': i, 'table_id': i // 3, 'shift': i % 3, 'loc': shift_table(i // 3, i % 3), 'occupied': False}))
        AGV_MAP[table_list[i]['loc'][0]][table_list[i]['loc'][1]] = 5  # 将工作台附近禁止通行
    # 初始化AGV对象列表
    vehicles = []
    for i in range(AGV_NUM):
        vehicles.append(AGV(id=i, x=0, y=(i + 1) * (MAP.shape[1] // (AGV_NUM + 1)), direction='down', battery=FULL_CHARGE))

    t = 0  # 时间步
    simInfo = []  # 仿真信息，用来实现可视化
    time_start = time.time()  # 主循环开始运行时间
    order_complete_time = 1000000000  # 订单完成所用时间
    tbreak = -1
    # 仿真主循环
    # -------------------------------------------------------------------------------------------------------
    while True:
        AGVInfo = []  # 包括AGV的位置、方向、颜色（是否正托举货架）、电量
        # 分配订单
        for order in order_list:
            orderDistribute(order, vehicles, all_shelf_status, table_list)
            # order.show()
        for car in vehicles:
            # 如果存在agv已分配任务且未启动或已完成任务
            if car.status not in ['available', 'waiting to charge', 'waiting at start', 'waiting to select'] and car.point == 0:
                print('car %d triggered renewing'%car.id)
                cons = []  # 约束
                staying_cars = []  # 发生路径更新时已经抵达目标点的AGV（在抬起或放下货架，或在工作台处分拣）
                moving_cars = []  # 发生路径更新时未抵达目标点的AGV
                moving_car_id = 0
                maps = []
                starts = []
                ends = []
                root_paths = {}
                directions = []
                arrived_at_start = False
                for car in vehicles:
                    if car.status == 'arrived at start':
                        arrived_at_start = True
                        break
                for car in vehicles:
                    # 如果AGV处于to shelf（从起点至目标货架）或return shelf（从工作台处送还货架）
                    if car.status == 'to shelf' or car.status == 'return shelf':
                        # 如果更新路径时该AGV已经运行至目标货架处（正在抬起或放下货架）
                        if SHELF_COORD[car.orders[-1]['shelf_id']] == (car.x, car.y):
                            staying_cars.append(car.id)
                        else:
                            grid = AGV_MAP.copy()
                            # 将目标货架处设为可通行
                            grid[SHELF_COORD[car.orders[-1]['shelf_id']][0]][
                                SHELF_COORD[car.orders[-1]['shelf_id']][1]] = 0
                            maps.append(grid)
                            starts.append((car.x, car.y))
                            ends.append(SHELF_COORD[car.orders[-1]['shelf_id']])
                            if car.point not in [0, len(car.path) - 1]:
                                root_paths[moving_car_id] = car.path[car.point:]
                            directions.append(car.direction)
                            moving_cars.append(car.id)
                            # 添加额外约束，让AGV在货架处停留一段时间表示在抬起或放下货架
                            cons.append({'agent': moving_car_id, 'timestep': LIFTING_TIME, 'type': 'additional'})
                            moving_car_id += 1
                    elif car.status == 'to select':
                        # 如果更新路径时该AGV已经运行至工作台（正在分拣）
                        if car.orders[-1]['table_position'] == (car.x, car.y):
                            staying_cars.append(car.id)
                        else:
                            grid = AGV_MAP.copy()
                            # 将目标工作台处设为可通行
                            grid[car.orders[-1]['table_position'][0]][
                                car.orders[-1]['table_position'][1]] = 0
                            maps.append(grid)
                            starts.append((car.x, car.y))
                            ends.append(car.orders[-1]['table_position'])
                            if car.point not in [0, len(car.path) - 1]:
                                root_paths[moving_car_id] = car.path[car.point:]
                            grid[car.orders[-1]['table_position'][0]][car.orders[-1]['table_position'][1]] = 0
                            directions.append(car.direction)
                            moving_cars.append(car.id)
                            moving_car_id += 1
                    elif car.status == 'to charge':
                        if len(car.charge_mission) > 0 and car.charge_mission[-1]['loc'] == (car.x, car.y):
                            staying_cars.append(car.id)
                        else:
                            grid = AGV_MAP.copy()
                            # 将目标充电桩处设为可通行
                            grid[car.charge_mission[-1]['loc'][0]][
                                car.charge_mission[-1]['loc'][1]] = 0
                            starts.append((car.x, car.y))
                            ends.append(car.charge_mission[-1]['loc'])
                            if car.point not in [0, len(car.path) - 1]:
                                root_paths[moving_car_id] = car.path[car.point:]
                            grid[car.charge_mission[-1]['loc'][0]][car.charge_mission[-1]['loc'][1]] = 0
                            maps.append(grid)
                            directions.append(car.direction)
                            moving_cars.append(car.id)
                            moving_car_id += 1
                    elif car.status == 'back to start':
                        if (car.x, car.y) == car.start:
                            staying_cars.append(car.id)
                        else:
                            grid = AGV_MAP.copy()
                            maps.append(grid)
                            starts.append((car.x, car.y))
                            ends.append(car.start)
                            if car.point not in [0, len(car.path) - 1]:
                                root_paths[moving_car_id] = car.path[car.point:]
                            directions.append(car.direction)
                            moving_cars.append(car.id)
                            moving_car_id += 1
                    elif car.status == 'arrived at start':
                        staying_cars.append(car.id)
                        car.status = 'waiting at start'
                        print('car %d waiting at start'%car.id)
                print('staying_cars: ', staying_cars)
                print('moving_cars: ', moving_cars)
                print('starts: ', starts)
                print('ends: ', ends)
                print('directions: ', directions)
                print('cbs starts searching')
                if starts:
                    # 用cbs算法为所有moving cars生成无冲突路径
                    paths = cbs_reserve(maps, arrived_at_start, root_paths, starts, ends, directions, cons, maxIteration)
                    # 将生成的路径分配给moving cars
                    if paths:
                        for i in moving_cars:
                            vehicles[i].point = 0
                            vehicles[i].path = paths.pop(0)
                            # 更新AGV颜色列表
                            if vehicles[i].status == 'to shelf':
                                vehicles[i].color_list = ['k'] * len(vehicles[i].path)
                            elif vehicles[i].status in ['to select', 'return shelf']:
                                vehicles[i].color_list = ['y'] * len(vehicles[i].path)
                            elif vehicles[i].status == 'to charge':
                                vehicles[i].color_list = ['k'] * len(vehicles[i].path)
                            elif vehicles[i].status == 'back to start':
                                vehicles[i].color_list = ['k'] * len(vehicles[i].path)
                    else:
                        raise BaseException('cbs no solution!')
                break

        shelfInfo = ['y'] * SHELF_NUM  # 货架颜色信息
        for car in vehicles:
            # 录入AGV信息
            target = 'None'
            if car.status in ['to shelf', 'return shelf']:
                target = 'shelf %d'%car.orders[-1]['shelf_id']
            elif car.status == 'to select':
                target = 'table %d'%car.orders[-1]['table_id']
            elif car.status == 'to charge':
                target = 'charging station %d'%car.charge_mission[-1]['id']
            AGVInfo.append({'id': car.id, 'x': car.x, 'y': car.y, 'direction': car.direction, 'color': car.color,
                            'status': car.status, 'battery': '%.1f%%'%(car.battery / FULL_CHARGE * 100), 'target': target})
            order_left_to_assign = 0  # 未指派的订单
            for order in order_list:
                for shelf_status in order.shelf_status:
                    if shelf_status == 'todo':
                        order_left_to_assign += 1
            numOfBackToStart = 0
            for agv in vehicles:
                if agv.status == 'back to start':
                    numOfBackToStart += 1
            # 如果AGV空闲且没有剩余的未指派订单，则令AGV返回起点
            if car.status == 'available' and order_left_to_assign == 0:
                if (car.x, car.y) != car.start:
                    # 分批返回起点（如果当前处于返程的AGV超过总数的一半则继续等待），防止一次性返回车数过多，造成拥堵
                    if numOfBackToStart <= AGV_NUM // 2:
                        print('car %d back to start'%car.id)
                        car.status = 'back to start'
                else:
                    car.status = 'waiting at start'
                    AGV_MAP[car.start[0]][car.start[1]] = 4
            elif car.status != 'available' and car.status != 'waiting at start':
                car.move()
                if car.color == 'y':
                    shelf_color = 'w'
                elif car.color == 'k':
                    shelf_color = 'y'
                shelfInfo[car.orders[-1]['shelf_id']] = shelf_color
                # 当AGV完成一个阶段的任务，更新AGV对象状态参数
                if car.point == len(car.path) - 1:
                    if car.status == 'to shelf':
                        print('car %d reached shelf %d at position (%d,%d) for the first time' % (
                            car.id, car.orders[-1]['shelf_id'], SHELF_COORD[car.orders[-1]['shelf_id']][0],
                            SHELF_COORD[car.orders[-1]['shelf_id']][1]))
                        available_tables = []
                        for table in table_list:
                            if order_list[car.orders[-1]['order_id']].table == None:
                                if not table['occupied']:
                                    available_tables.append(table)
                            else:
                                if order_list[car.orders[-1]['order_id']].table == table['table_id'] and not table['occupied']:
                                    available_tables.append(table)
                        if available_tables:
                            target = available_tables[0]
                            for table in available_tables:
                                if abs(car.x - table['loc'][0]) + abs(car.y - table['loc'][1]) < abs(
                                        car.x - target['loc'][0]) + abs(car.y - target['loc'][1]):
                                    target = table
                            order_list[car.orders[-1]['order_id']].table = target['table_id']
                            car.orders[-1]['table_id'] = target['table_id']
                            car.orders[-1]['table_position'] = target['loc']
                            car.orders[-1]['table_position_id'] = target['id']
                            car.orders[-1]['shift'] = target['shift']
                            table_list[target['id']]['occupied'] = True
                            car.status = 'to select'
                            print('car %d going for table %d at position (%d,%d)'% (car.id, car.orders[-1]['table_id'],
                            car.orders[-1]['table_position'][0], car.orders[-1]['table_position'][1]))
                            car.point = 0
                        else:
                            car.status = 'waiting to select'
                            print('car %d waiting to select at position (%d, %d)' % (car.id, car.x, car.y))
                        continue
                    if car.status == 'to select':
                        print('car %d reached table %d at position (%d,%d)' % (
                        car.id, car.orders[-1]['table_id'], car.orders[-1]['table_position'][0],
                        car.orders[-1]['table_position'][1]))
                        car.status = 'selecting'
                        print('car %d start selecting at position (%d, %d)' % (car.id, car.x, car.y))
                        continue

                    if car.status == 'return shelf':
                        print('car %d reached shelf %d at position (%d,%d) for the second time' % (
                            car.id, car.orders[-1]['shelf_id'], SHELF_COORD[car.orders[-1]['shelf_id']][0],
                            SHELF_COORD[car.orders[-1]['shelf_id']][1]))
                        if car.check_battery():
                            all_shelf_status[car.orders[-1]['shelf_id']] = 1  # 更新货架在位情况
                            car.status = 'available'
                            car.point = 0
                            continue
                        else:
                            available_stations = []
                            for station in charging_stations:
                                if not station['occupied']:
                                    available_stations.append(station)
                            if available_stations:
                                all_shelf_status[car.orders[-1]['shelf_id']] = 1  # 更新货架在位情况
                                car.status = 'to charge'
                                print('car %d to charge' % car.id)
                                target = available_stations[0]
                                for station in available_stations:
                                    if abs(car.x - station['loc'][0]) + abs(car.y - station['loc'][1]) < abs(car.x - target['loc'][0]) + abs(car.y - target['loc'][1]):
                                        target = station
                                car.charge_mission.append({'id': target['id'], 'loc': target['loc']})
                                charging_stations[target['id']]['occupied'] = True
                                car.point = 0
                            else:
                                car.status = 'waiting to charge'
                                print('car %d waiting to charge'%car.id)
                            continue
                    if car.status == 'to charge':
                        car.status = 'charging'
                        print('car %d start charging at position (%d, %d)' % (car.id, car.x, car.y))
                        continue
                    if car.status == 'selecting':
                        car.selecting_process += 1
                        if car.selecting_process >= car.orders[-1]['tsort']:
                            print('car %d finished selecting at position (%d, %d)' % (car.id, car.x, car.y))
                            car.status = 'return shelf'
                            print('car %d returning shelf %d at position (%d,%d)' % (
                                car.id, car.orders[-1]['shelf_id'], SHELF_COORD[car.orders[-1]['shelf_id']][0],
                                SHELF_COORD[car.orders[-1]['shelf_id']][1]))
                            revenue += car.orders[-1]['tsort']  # 结算分拣收益
                            table_list[car.orders[-1]['table_position_id']]['occupied'] = False
                            order_list[car.orders[-1]['order_id']].shelf_status[car.orders[-1]['sub_order_id']] = 'done'
                            car.selecting_process = 0
                            car.point = 0
                        continue
                    if car.status == 'charging':
                        car.charge()
                        if car.battery == FULL_CHARGE:
                            print('car %d finished charging at position (%d, %d)' % (car.id, car.x, car.y))
                            car.status = 'available'
                            charging_stations[car.charge_mission[-1]['id']]['occupied'] = False
                            car.point = 0
                        continue
                    if car.status == 'waiting to select':
                        available_tables = []
                        for table in table_list:
                            if order_list[car.orders[-1]['order_id']].table == None:
                                if not table['occupied']:
                                    available_tables.append(table)
                            else:
                                if order_list[car.orders[-1]['order_id']].table == table['table_id'] and not table['occupied']:
                                    available_tables.append(table)
                        if available_tables:
                            target = available_tables[0]
                            for table in available_tables:
                                if abs(car.x - table['loc'][0]) + abs(car.y - table['loc'][1]) < abs(
                                        car.x - target['loc'][0]) + abs(car.y - target['loc'][1]):
                                    target = table
                            order_list[car.orders[-1]['order_id']].table = target['table_id']
                            car.orders[-1]['table_id'] = target['table_id']
                            car.orders[-1]['table_position'] = target['loc']
                            car.orders[-1]['table_position_id'] = target['id']
                            car.orders[-1]['shift'] = target['shift']
                            table_list[target['id']]['occupied'] = True
                            car.status = 'to select'
                            print('car %d to select at table %d at (%d, %d)' % (car.id, target['table_id'], target['loc'][0], target['loc'][1]))
                            car.point = 0
                        else:
                            print('car %d waiting to select at position (%d, %d)' % (car.id, car.x, car.y))
                        continue
                    if car.status == 'waiting to charge':
                        print('car %d waiting to charge at position (%d, %d)' % (car.id, car.x, car.y))
                        available_stations = []
                        for station in charging_stations:
                            if not station['occupied']:
                                available_stations.append(station)
                        if available_stations:
                            all_shelf_status[car.orders[-1]['shelf_id']] = 1  # 更新货架在位情况
                            car.status = 'to charge'
                            print('car %d to charge' % car.id)
                            target = available_stations[0]
                            for station in available_stations:
                                if abs(car.x - station['loc'][0]) + abs(car.y - station['loc'][1]) < abs(
                                        car.x - target['loc'][0]) + abs(car.y - target['loc'][1]):
                                    target = station
                            car.charge_mission.append({'id': target['id'], 'loc': target['loc']})
                            charging_stations[target['id']]['occupied'] = True
                            car.point = 0
                        continue
                    if car.status == 'back to start':
                        AGV_MAP[car.start[0]][car.start[1]] = 4
                        numOfIdle = 0
                        for i in range(AGV_NUM):
                            if vehicles[i].status == 'waiting at start':
                                numOfIdle += 1
                        if numOfIdle < AGV_NUM - 1:
                            print('car %d arrived at start'%car.id)
                            car.status = 'arrived at start'
                            car.point = 0
                        else:
                            car.status = 'waiting at start'
                        continue

        order_complete = 0  # 完成的订单数
        for order in order_list:
            if order.shelf_status == ['done'] * len(order.shelf):
                order_complete += 1
        simInfo.append({'AGVInfo': AGVInfo, 'shelfInfo': shelfInfo, 'order_complete': order_complete, 't': t, 'revenue': revenue})


        if t == tbreak:
            break
        numOfIdle = 0
        for car in vehicles:
            print(car.id, car.status)
            if car.status == 'waiting at start':
                numOfIdle += 1
        print('order_complete: ', order_complete)
        print('=====================================================================================================================')
        if order_complete == ORDER_NUM:
            if t < order_complete_time:
                order_complete_time = t
            if numOfIdle == AGV_NUM:
                tbreak = t + 1
        t += 1
    # -------------------------------------------------------------------------------------------------------
    time_end = time.time()
    time_sum = time_end - time_start  # 计算的时间差为程序的执行时间，单位为秒/s

    total_net_revenue = revenue * 0.5 - (AGV_COST * AGV_NUM + CHARGING_STATION_COST * CHARGING_STATION_NUM) * t - WORKER_COST * TABLE_NUM * order_complete_time
    revenue_per_hour = total_net_revenue / t * 3600
    print('run time:', time_sum)
    print('%d orders completed with %d agvs with %d seconds' % (ORDER_NUM, AGV_NUM, order_complete_time))
    print('AGV_COST: %.2f, CHARGING_STATION_COST: %.2f, WORKER_COST: %.1f, revenue: %.1f' % (
    AGV_COST * AGV_NUM * t, CHARGING_STATION_COST * CHARGING_STATION_NUM * t, WORKER_COST * TABLE_NUM * order_complete_time, revenue * 0.5))
    print('total net revenue: %.1f' % total_net_revenue)
    print('revenue_per_hour: %.1f' % revenue_per_hour)
    heat_map_data = np.zeros(MAP.shape)  # 热力图矩阵
    utilized_time = [0] * AGV_NUM  # 各AGV被利用的时间步数
    for info in simInfo:
        for i, car in enumerate(info['AGVInfo']):
            heat_map_data[car['x']][car['y']] += 1
            if car['status'] in ['to shelf', 'to select', 'selecting', 'return shelf']:
                utilized_time[i] += 1
    mean_utility = sum(utilized_time) / (t * AGV_NUM)
    for i, ut in enumerate(utilized_time):
        print('car %d utility %.2f%%' % (i, utilized_time[i]/t*100))
    print('mean agv utility: %.2f%%' % (mean_utility * 100))

    if heat_map:
        sns.set_context({"figure.figsize": (MAP.shape[1], MAP.shape[0])})
        sns.heatmap(data=heat_map_data, square=True, cmap="RdBu_r", linewidths=0.3, cbar_kws={"shrink": 0.8})
        plt.savefig('heat_map for %d orders, %d AGVs .png' % (ORDER_NUM, AGV_NUM), dpi=300)
        plt.show()

    if show:
        # 货架颜色信息由于主循环内更新先后顺序原因出现1时间步错位，在此矫正。
        for i in range(len(simInfo) - 1, 0, -1):
            simInfo[i]['shelfInfo'] = simInfo[i - 1]['shelfInfo']
        ani, fps = create_animation(MAP, simInfo, ORDER_NUM, AGV_COST, CHARGING_STATION_COST, WORKER_COST, TABLE_NUM, order_complete_time, interval=interval, SAVE_GIF=save_fig)
        if save_fig:
            ani.save("gifs//map1 %d orders, %d AGVs.gif" % (ORDER_NUM, AGV_NUM), fps=fps, writer="pillow")
        plt.show()
    return np.array([AGV_NUM,
                     ORDER_NUM,
                     order_complete_time,  # 订单完成时间
                     revenue_per_hour,  # 每小时的净收益
                     mean_utility,  # AGV的利用率
                     total_net_revenue,  # 总净收益
                     revenue * 0.5,  # 订单完成利润
                     AGV_COST * AGV_NUM * t,  # AGV成本
                     CHARGING_STATION_COST * CHARGING_STATION_NUM * t,  # 充电桩成本
                     WORKER_COST * TABLE_NUM * order_complete_time])  # 工作台工人成本


