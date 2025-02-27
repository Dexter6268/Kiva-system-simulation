# AGV转90°的时间为2s，转180°的时间为3s
TIME_OF_TURN90 = 2
TIME_OF_TURN180 = 3
class Node(object):
    """
    A*算法中的结点对象
    """

    def __init__(self, x, y, t, direction, g, h, father):
        """
        :param x: int，该结点的x坐标（地图矩阵中的行数）
        :param y: int，该结点的y坐标（地图矩阵中的列数）
        :param t: int，该结点的时间坐标
        :param direction: int，该结点的朝向
        :param g: int，A*算法中的g值，即该结点距离起点的时空距离
        :param h: int，A*算法中的h值，即该结点距离终点的曼哈顿距离
        :param father: Node，该结点的父结点
        """
        self.x = x
        self.y = y
        self.t = t
        self.direction = direction
        self.g = g
        self.h = h
        self.father = father

    def getNeighbor(self, mapdata, endx, endy):
        """
        功能：获取结点的邻近结点
        :param mapdata: 2d-np.array，栅格地图矩阵（0代表可通行，非0代表障碍物）
        :param endx: int，终点x坐标
        :param endy: int，终点y坐标
        :return: result: list，临近结点列表
        """
        x = self.x
        y = self.y
        result = []
        # 如果该结点不在地图上边界（x==0）并且上方无障碍物
        if (x != 0 and mapdata[x - 1][y] == 0):
            # 如果该结点朝上
            if self.direction == 'up':
                timestep = 1
            # 如果该结点朝左或朝右
            elif self.direction == 'left' or self.direction == 'right':
                timestep = 1 + TIME_OF_TURN90
            # 如果该结点朝下
            
            else:
                timestep = 1 + TIME_OF_TURN180
            # 上方临近结点
            upNode = Node(x - 1, y, self.t + timestep, 'up', self.g + 10 * timestep,
                          (abs(x - 1 - endx) + abs(y - endy)) * 10, self)
            result.append(upNode)
        # 如果该结点不在地图下边界（x==len(mapdata) - 1）并且下方无障碍物
        if (x != len(mapdata) - 1 and mapdata[x + 1][y] == 0):
            # 如果该结点朝下
            if self.direction == 'down':
                timestep = 1
            # 如果该结点朝左或朝右
            elif self.direction == 'left' or self.direction == 'right':
                timestep = 1 + TIME_OF_TURN90
            # 如果该结点朝上
            else:
                timestep = 1 + TIME_OF_TURN180
            # 下方临近结点
            downNode = Node(x + 1, y, self.t + timestep, 'down', self.g + 10 * timestep,
                            (abs(x + 1 - endx) + abs(y - endy)) * 10, self)
            result.append(downNode)
        # 如果该结点不在地图左边界（y==0）并且左方无障碍物
        if (y != 0 and mapdata[x][y - 1] == 0):
            # 如果该结点朝左
            if self.direction == 'left':
                timestep = 1
            # 如果该结点朝上或朝下
            elif self.direction == 'up' or self.direction == 'down':
                timestep = 1 + TIME_OF_TURN90
            # 如果该结点朝右
            else:
                timestep = 1 + TIME_OF_TURN180
            # 左方临近结点
            leftNode = Node(x, y - 1, self.t + timestep, 'left', self.g + 10 * timestep,
                            (abs(x - endx) + abs(y - 1 - endy)) * 10, self)
            result.append(leftNode)
        # 如果该结点不在地图右边界（len(mapdata[0]) - 1）并且右方无障碍物
        if (y != len(mapdata[0]) - 1 and mapdata[x][y + 1] == 0):
            # 如果该结点朝右
            if self.direction == 'right':
                timestep = 1
            # 如果该结点朝上或朝下
            elif self.direction == 'up' or self.direction == 'down':
                timestep = 1 + TIME_OF_TURN90
            # 如果该结点朝左
            else:
                timestep = 1 + TIME_OF_TURN180
            # 右方临近结点
            rightNode = Node(x, y + 1, self.t + timestep, 'right', self.g + 10 * timestep,
                             (abs(x - endx) + abs(y + 1 - endy)) * 10, self)
            result.append(rightNode)
        # 原地等待
        stayNode = Node(x, y, self.t + 1, self.direction, self.g + 10, self.h, self)
        result.append(stayNode)
        return result

    def inConstraints(self, constraints, endx, endy, tmax):
        """
        功能：检测结点是否违背约束
        :param constraints: list，约束列表
            约束为dict，字段如下：
            'agent': int，AGV id
            'loc': list，约束位置，长度为1，i.e.[(x, y)]或2，i.e.[(x1, y1), (x2, y2)]
                   分别对应vertex collision（两车同时到达同一位置）和edge collision（两车交换位置）
            'timestep': int，碰撞发生的时间步，若是edge collision则是交换位置后的时间步（即(x2, y2)）的时间步
            'type': str，'positive', 'negative', 'additional' 中的一种
        :param endx: int，终点的x坐标
        :param endy: int，终点的y坐标
        :param tmax: int，约束列表中最大的timestep
        :return: 如果违背约束列表中的某一个约束，返回True，否则返回False
        """
        for constraint in constraints:
            # 如果时空位置和constraint一致或者父节点的空间位置和约束一致且约束时间步位于父节点时间步和当前结点之间（即发生转向时停在原地的状况）
            if (constraint['loc'] == [(self.x, self.y)] and constraint['timestep'] == self.t) or (
                    constraint['loc'] == [(self.father.x, self.father.y)] and constraint['timestep'] > self.father.t and
                    constraint['timestep'] < self.t):
                return True
            # 如果时空位置满足edge collision引发的constraint
            if constraint['loc'] == [(self.father.x, self.father.y), (self.x, self.y)] and constraint[
                'timestep'] == self.t:
                return True
            # 如果在约束的最大时间之前到达终点
            if (endx, endy) == (self.x, self.y) and self.t < tmax:
                return True
        return False
    def hasNode(self, openList):
        """
        功能：检测openList中是否含有该结点
        :param openList: list，A*算法中的open list，即所有被考虑用来寻找最短路的结点
        :return: 如果open list中存在该结点则返回True，否则返回False
        """
        for node in openList:
            if (node.x, node.y, node.t) == (self.x, self.y, self.t):
                return True
        return False
    def changeG(self, openList):
        """
        功能：更新open list中的g值
        :param openList: list，A*算法中的open list，即所有被考虑用来寻找最短路的结点
        """
        for node in openList:
            if (node.x, node.y, node.t) == (self.x, self.y, self.t):
                if node.g > self.g:
                    node.g = self.g

def getKeyforSort(element: Node):
    """
    功能：返回结点的f值(g+h)用作open list排序的key
    :param element:
    :return: g+h
    """
    return element.g + element.h

def astar(mapdata, startx, starty, endx, endy, startdirection, constraints):
    """
    功能：给出符合约束的最短路
    :param mapdata: 2d-np.array，栅格地图矩阵（0代表可通行，非0代表障碍物）
    :param startx: int，起点x坐标
    :param starty: int，起点y坐标
    :param endx: int，终点x坐标
    :param endy: int，终点y坐标
    :param startdirection: str，AGV其起始方向，'up', 'down', 'right, 'down' 中的一个
    :param constraints: list of dicts，约束列表
    :return: 如果存在，返回list of tuples，路径列表[(x1, y1, t1, direction1), (x2, y2, t2, direction2), ...]
             否则返回None
    """
    positive_constraints = []  # 正约束列表，即AGV在某个时间点必须到某个坐标或者从某个坐标走到某个坐标
    negative_constraints = {}  # 负约束字典，即AGV在某个时间点不允许到某个坐标或者从某个坐标走到某个坐标
    tlist = []  # 约束的时间步列表
    additional_constraint = []  # 额外约束，由于AGV在货架或工作台处必须停留一段时间，因而在路径最后加上一段停留在某个位置
    for constraint in constraints:
        if constraint['type'] == 'positive':
            positive_constraints.append(constraint)
            tlist.append(constraint['timestep'])
        if constraint['type'] == 'negative':
            if constraint['timestep'] in negative_constraints:
                negative_constraints[constraint['timestep']].append(constraint)
            else:
                negative_constraints[constraint['timestep']] = [constraint]
            tlist.append(constraint['timestep'])
        if constraint['type'] == 'additional':
            additional_constraint = constraint
    if tlist:
        tmax = max(tlist)  # 约束列表中最大的timestep
    else:
        tmax = 0
    # 起点
    startNode = Node(startx, starty, t=0, direction=startdirection, g=0,
                     h=(abs(startx - endx) + abs(starty - endy)) * 10, father=None)
    openList = []  # A*算法中的open list，即所有被考虑用来寻找最短路的结点
    closedlist = []  # A*算法中的closed list，即所有不再会被考虑用来寻找最短路的结点
    closedlist.append(startNode)
    currNode = startNode
    # 循环终止条件为找到终点且时间步大于约束的最大时间步
    while ((endx, endy) != (currNode.x, currNode.y)) or currNode.t <= tmax:
        neighborList = currNode.getNeighbor(mapdata, endx, endy)  # currNode的临近结点列表
        for neighbor in neighborList:
            constraints_to_be_considered = []
            for t in range(currNode.t, neighbor.t + 1):
                if t in negative_constraints:
                    constraints_to_be_considered += negative_constraints[t]
            if (neighbor not in closedlist) and (not neighbor.inConstraints(constraints_to_be_considered, endx, endy, tmax)):
                # debug用
                # print('neighbor: x=%d, y=%d, t=%d, direction=%s, g=%d, h=%d' %(neighbor.x, neighbor.y, neighbor.t, neighbor.direction, neighbor.g, neighbor.h))
                if neighbor.hasNode(openList):
                    neighbor.changeG(openList)
                else:
                    openList.append(neighbor)
        # print('----------------------------------------------------------')
        openList.sort(key=getKeyforSort)
        if openList:
            currNode = openList.pop(0)
        else:
            print('astar no solution')
            return None
        closedlist.append(currNode)

    result = []  # 输出的路径
    while (currNode.father != None):
        result.append((currNode.x, currNode.y, currNode.t, currNode.direction))
        currNode = currNode.father
    result.append((currNode.x, currNode.y, currNode.t, currNode.direction))
    result.reverse()
    # 填充转向时的间隔，如t=3时在(2, 2)（第二行第二列），方向朝右，t=6时在(3, 2)（第三行第二列），方向朝下
    # 需要填充t=4和t=5时刻的位置和方向，即(x=2, y=2, t=4, direction='right')和(x=2, y=2, t=5, direction='down')
    for i, coord in enumerate(result):
        if coord[2] == i + 1:
            result.insert(i, (result[i - 1][0], result[i - 1][1], i, result[i][3]))
        elif coord[2] != i:
            result.insert(i, (result[i - 1][0], result[i - 1][1], i, result[i - 1][3]))
    # 如果存在额外约束，则在路径最后添加原地等待的过程（即位置坐标和方向不变，时间步增加）
    if additional_constraint:
        last_ind = len(result) - 1
        for i in range(1, additional_constraint['timestep'] + 1):
            result.append((result[last_ind][0], result[last_ind][1], last_ind + i, result[last_ind][3]))
    return result
