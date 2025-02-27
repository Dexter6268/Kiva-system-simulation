import numpy as np
import pandas as pd

df = pd.read_excel('maps//map1(3table).xlsx').fillna(0)
MAP = df.iloc[0:-1, 1: -1].values
SHELF_COORD = []
TABLE_COORD = []
CHARGING_STATION_COORD = []
for x in range(MAP.shape[0]):
    for y in range(MAP.shape[1]):
        if MAP[x][y] == 1:
            SHELF_COORD.append((x, y))
        if MAP[x][y] == 2:
            TABLE_COORD.append((x, y))
        if MAP[x][y] == 3:
            CHARGING_STATION_COORD.append((x, y))
SHELF_NUM = len(SHELF_COORD)
TABLE_NUM = len(TABLE_COORD)
CHARGING_STATION_NUM = len(CHARGING_STATION_COORD)

class Order:
    def __init__(self, id, shelf, shelf_status):
        """
        :param id: int，订单id
        :param shelf: list，货架id列表
        :param shelf_status: list of strings，货架状态列表，状态为 'done'/'doing'/'todo'
        """
        self.id = id
        self.shelf = shelf
        self.shelf_status = shelf_status
        self.table = None
    def show(self):
        """
        print订单信息
        """
        print('shelf = ', self.shelf, ', shelf_status = ', self.shelf_status, ', table = ', self.table)

def generate_orders(seed, numOfOrders=int(np.random.normal(50, 5, 1)[0])):
    """
    功能：生成订单列表
    :param numOfOrders: int，生成订单的数量，默认服从均值为50，方差标准差5为正态分布
    :return: list，返回订单列表
    """
    np.random.seed(seed)
    order = []
    for i in range(numOfOrders):
        numOfShelves = np.random.randint(1, 5)  # 假设每单涉及的货架数量服从1到4的均匀分布
        shelf = np.random.choice(SHELF_NUM, numOfShelves, replace=False)
        shelf_status = ['todo'] * numOfShelves
        order.append(Order(i, shelf, shelf_status))
    return order


def shift_table(table_id, shift):
    """
    功能：返回工作台附近的坐标（AGV分拣时处于工作台的上、左、右位置）
    :param table_id: int，工作台id
    :param shift: int，偏移参数，0，-1，1分别代表工作台上方，左方，右方
    :return: tuple，工作台附近的坐标
    """
    if shift == 0:
        table_co = (TABLE_COORD[table_id][0] - 1, TABLE_COORD[table_id][1])
    else:
        table_co = (TABLE_COORD[table_id][0], TABLE_COORD[table_id][1] + shift)
    return table_co


def orderDistribute(order, vehicle, all_shelf_status, table_list):
    """
    功能：将一个订单中的货架分配给若干个AGV
    原则：选择距离（道路距离）货架最近的空闲车辆，订单涉及多个货架则每个货架分配一辆车
    :param order: Order object
    :param vehicle: list of AGV objects，AGV对象的列表
    """
    for i in range(len(order.shelf)):
        # 如果该货架尚未被分配且仍然在原位（没有被其它AGV运走）
        if order.shelf_status[i] == 'todo' and all_shelf_status[order.shelf[i]] == 1:
            distance = 10000  # 该货架距离AGV的最近距离
            min_index = len(vehicle)   # 距离该货架最近的AGV的id
            for j in range(len(vehicle)):
                # 如果该AGV空闲
                if vehicle[j].status == 'available':
                    # AGV与货架之间的曼哈顿距离
                    temp = abs(SHELF_COORD[order.shelf[i]][0] - vehicle[j].x) + abs(
                        SHELF_COORD[order.shelf[i]][1] - vehicle[j].y)
                    if temp < distance:
                        distance = temp
                        min_index = j
            # 如果该货架被分配给了某AGV
            if distance < 10000:
                if vehicle[min_index].orders and SHELF_COORD[vehicle[min_index].orders[-1]['shelf_id']] == SHELF_COORD[order.shelf[i]] and vehicle[min_index].color_list[-1] == 'y':  # 如果这辆车分配的货架和刚完成的一单一样且不是刚充完电
                    available_tables = []
                    for table in table_list:
                        if order.table == None:
                            if not table['occupied']:
                                available_tables.append(table)
                        else:
                            if order.table == table['table_id'] and not table['occupied']:
                                available_tables.append(table)
                    if available_tables:
                        target = available_tables[0]
                        for table in available_tables:
                            if abs(vehicle[min_index].x - table['loc'][0]) + abs(vehicle[min_index].y - table['loc'][1]) < abs(
                                    vehicle[min_index].x - target['loc'][0]) + abs(vehicle[min_index].y - target['loc'][1]):
                                target = table
                        tsort = int(np.random.normal(10, 2, 1)[0])  # 分拣时间服从均值为10，标准差为2的正态分布
                        vehicle[min_index].orders.append(
                            {'order_id': order.id,
                             'sub_order_id': i,
                             'shelf_id': order.shelf[i],
                             'table_id': target['table_id'],
                             'table_position': target['loc'],
                             'table_position_id': target['id'],
                             'shift': target['shift'],
                             'tsort': tsort})
                        print('car %d assigned shelf %d again at (%d, %d) and table %d at (%d, %d)' % (
                            min_index, order.shelf[i], SHELF_COORD[order.shelf[i]][0], SHELF_COORD[order.shelf[i]][1],
                            target['table_id'], target['loc'][0], target['loc'][1]))
                        order.table = target['table_id']
                        table_list[target['id']]['occupied'] = True
                        vehicle[min_index].status = 'to select'
                        order.shelf_status[i] = 'doing'
                        all_shelf_status[order.shelf[i]] = 0  # 更新该货架状态
                        print('car %d going for table %d at position (%d,%d)' % (vehicle[min_index].id, vehicle[min_index].orders[-1]['table_id'],
                                                                                 vehicle[min_index].orders[-1]['table_position'][0],
                                                                                 vehicle[min_index].orders[-1]['table_position'][1]))
                    else:
                        print('car %d assigned shelf %d again at (%d, %d)' % (
                            min_index, order.shelf[i], SHELF_COORD[order.shelf[i]][0], SHELF_COORD[order.shelf[i]][1]))
                        vehicle[min_index].status = 'waiting to select'
                        all_shelf_status[vehicle[min_index].orders[-1]['shelf_id']] = 0
                        vehicle[min_index].point = len(vehicle[min_index].path) - 1
                        print('car %d waiting to select' % vehicle[min_index].id)
                else:
                    vehicle[min_index].status = 'to shelf'
                    tsort = int(np.random.normal(10, 2, 1)[0])  # 分拣时间服从均值为10，标准差为2的正态分布
                    vehicle[min_index].orders.append(
                        {'order_id': order.id,
                         'sub_order_id': i,
                         'shelf_id': order.shelf[i],
                         'table_id': None,
                         'table_position': None,
                         'table_position_id': None,
                         'shift': None,
                         'tsort': tsort})
                    print('car %d assigned shelf %d at (%d, %d)' % (
                    min_index, order.shelf[i], SHELF_COORD[order.shelf[i]][0], SHELF_COORD[order.shelf[i]][1]))
                    order.shelf_status[i] = 'doing'
                    all_shelf_status[order.shelf[i]] = 0  # 更新该货架状态
