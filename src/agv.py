from mapAndOrder import MAP
FULL_CHARGE = 3600  # 按充10分钟跑1小时来算，满电是3600，代表可以跑大概3600秒（粗略估计，直线行驶按1秒消耗1电，转弯按1秒消耗2电）
BATTERY_CONSUMING_SPEED = 1  # 电量消耗速度/s
CHARGING_SPEED = 6 * BATTERY_CONSUMING_SPEED
class AGV:
    def __init__(self, id, x, y, direction, battery):
        """
        :param id: 
        :param x: int，AGV当前的x坐标（位于栅格地图的第几行）
        :param y: int，AGV当前的y坐标（位于栅格地图的第几列）
        :param direction: string，AGV当前的朝向，'up', 'down', 'right, 'down' 中的一个
        :param status: string，AGV当前的状态
            'available'：未指派订单
            'to shelf': 从起点到目标货架的过程中（包括将目标货架抬起）
            'to select': 从目标货架到目标工作台的过程中（包括在工作台处等待分拣）
            'wait to select': 目标工作台已被占用，在货架处原地等待的过程
            'selecting': 在工作台处等待分拣
            'return shelf': 从目标工作台将目标货架返还的过程中（包括将目标货架放下）
            'to charge': 完成订单后（位于某货架处）前往充电桩的过程
            'wait to charge': 所有充电桩已被占用，在货架处原地等待的过程
            'charging': 在充电桩处充电
            'back to start': 订单完成后或充电完成后返回起点的过程
            'arrived at start': 到达起点，为了刷新路径，将该起点设为不可通行
            'waiting at start': 无订单且电量高于阈值，在起点处待机
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
        self.status = 'available'
        self.orders = []
        self.charge_mission = []
        self.point = 0
        self.color = 'k'
        self.path = []
        self.color_list = []
        self.selecting_process = 0
    def move(self):
        """
        功能：移动AGV，更新其状态
        """
        if self.point < len(self.path) - 1 and self.status not in ['waiting to charge', 'waiting at start', 'waiting to select', 'selecting', 'charging']:
            self.point += 1
            self.x = self.path[self.point][0]
            self.y = self.path[self.point][1]
            self.direction = self.path[self.point][3]
            self.color = self.color_list[self.point]
            if self.path[self.point - 1][:2] == self.path[self.point][:2]:
                self.battery -= BATTERY_CONSUMING_SPEED * 2  # 转弯时耗电是匀速前进时的2倍
            else:
                self.battery -= BATTERY_CONSUMING_SPEED
        if self.battery < 0:
            self.battery = 0

    def check_battery(self):
        # 检查电量是否足够
        return self.battery >= (MAP.shape[0] + MAP.shape[1]) * 6 * BATTERY_CONSUMING_SPEED
    def charge(self):
        self.battery += CHARGING_SPEED
        if self.battery > FULL_CHARGE:
            self.battery = FULL_CHARGE
