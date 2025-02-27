import heapq
from a_star import astar
def detect_collision(path1, path2):
    """
    功能：检测两路径的碰撞
    :param path1: list of tuples, 路径1，形式如[(x1, y1, t1, direction1), (x2, y2, t2, direction2), ...]
    :param path2: list of tuples, 路径2，形式如[(x1, y1, t1, direction1), (x2, y2, t2, direction2), ...]
    :return: 如果存在碰撞则返回 collision 字典，字段如下，否则返回None
        'loc': list，碰撞位置
        'timestep': int，碰撞时间
    """
    n = min(len(path1), len(path2)) # 两路径最短长度
    for t in range(1, n):
        # vertex collision 两车同时处于同一位置
        if (path1[t][0], path1[t][1]) == (path2[t][0], path2[t][1]):
            collision = {'loc': [(path1[t][0], path1[t][1])], 'timestep': t}
            return collision
        # edge collision 两车交换位置
        if (path1[t][0], path1[t][1]) == (path2[t - 1][0], path2[t - 1][1]) and (path1[t - 1][0], path1[t - 1][1]) == (
        path2[t][0], path2[t][1]):
            collision = {'loc': [(path1[t - 1][0], path1[t - 1][1]), (path1[t][0], path1[t][1])], 'timestep': t}
            return collision
    return None
def detect_collisions(paths):
    """
    功能：返回list，包含每两条路径中发生的第一个碰撞
    :param paths: list of paths
    :return: list
    """
    collisions = []
    n = len(paths)
    for i in range(n - 1):
        for j in range(i + 1, n):
            collision = detect_collision(paths[i], paths[j])
            if collision:
                collisions.append({'a1': i, 'a2': j, 'loc': collision['loc'], 'timestep': collision['timestep']})
    return collisions
def standard_splitting(collision):
    """
    功能：将一个碰撞分解为两条约束并返回
    :param collision: dict，字段如下
        'a1': int，第一台AGV的id
        'a2': int，第二台AGV的id
        'loc': list of tuples，碰撞发生的位置，如果是vertex collision则是一个坐标元组的列表，如果是edge collision则是两个坐标元组的列表
        'timestep': int, 碰撞发生的时间步，如果是edge collision则是交换位置后的时刻
    :return: list, 约束列表
    """
    cons = [] # 约束列表
    loc = collision['loc']
    # Vertex Collision（两车同时到达同地）：第一个约束禁止第一辆车在冲突时刻到达冲突点，第二个约束禁止第二辆车在冲突时刻到达冲突点
    if len(loc) == 1:
        # First Constraint
        cons.append({'agent': collision['a1'], 'loc': loc, 'timestep': collision['timestep'], 'type': 'negative'})
        # Second Constraint
        cons.append({'agent': collision['a2'], 'loc': loc, 'timestep': collision['timestep'], 'type': 'negative'})
        return cons
    # Edge Collision（两车交换位置）：第一个约束禁止第一辆车在冲突时刻游历冲突edge，第二个约束禁止第二辆车在冲突时刻游历冲突edge（从反方向）
    elif len(loc) == 2:
        # First Constraint
        cons.append(
            {'agent': collision['a1'], 'loc': [loc[0], loc[1]], 'timestep': collision['timestep'], 'type': 'negative'})
        # Second Constraint
        cons.append(
            {'agent': collision['a2'], 'loc': [loc[1], loc[0]], 'timestep': collision['timestep'], 'type': 'negative'})
        return cons
    return cons

def get_sum_of_cost(paths):
    """
    功能：返回所有路径的总成本，即所有路径长度加和
    :param paths: list of lists of tuples，路径的列表
    :return: int，路径总长度
    """
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst

def cbs(maps, starts, ends, directions, constraints, maxIteration):
    """
    功能：返回无冲突路径列表
    :param maps: list of 2d-np.array，栅格地图矩阵（0代表可通行，非0代表障碍物）的列表（由于每台AGV的目标货架处坐标只对自己是可通行的，因此每台AGV的地图各有不同）
    :param starts: list of tuples，起点坐标列表
    :param ends: list of tuples，终点坐标列表
    :param directions: list of strings，初始方向列表，形如 ['up', 'down', 'left', 'right'...]
    :param constraints: list of dicts, 初始约束列表，一般都是额外约束（在货架处或工作台处的停留）
    :param maxIteration: int，算法迭代次数上限
    :return: 如果在迭代上限内求出解则返回无冲突路径列表，否则返回None
    """
    constraint_table = {}  # 约束表，方便搜索
    for constraint in constraints:
        agent = constraint['agent']
        if agent in constraint_table:
            constraint_table[agent].append(constraint)
        else:
            constraint_table[agent] = [constraint]

    openList = [] # cbs算法中的open list，即待考虑的结点的列表
    num_of_generated = 0 # 由于heapq堆排的key是取列表中元组的第一个元素，相同的话取第二个，以此类推，如果出现tie的话会报错，因此引入子节点生成数作为唯一的key
    # 根节点
    root = {'cost': 0,
            'constraints': constraint_table,
            'paths': [],
            'collisions': []}

    # 为根节点生成初始路径列表
    for i in range(len(starts)):
        if i in root['constraints']:
            iconstraint = root['constraints'][i]
        else:
            iconstraint = []
        path = astar(maps[i], int(starts[i][0]), int(starts[i][1]), int(ends[i][0]), int(ends[i][1]), directions[i],
                     iconstraint)
        if not path:
            raise BaseException('No solution for AGV%d' % i)
        root['paths'].append(path)

    root['cost'] = get_sum_of_cost(root['paths'])
    root['collisions'] = detect_collisions(root['paths'])
    # 为提高计算效率，采用堆排，理论上为取最优解应以cost为key，这里用碰撞数作为key是为了在短时间内优选算出可行解
    heapq.heappush(openList, (len(root['collisions']), root['cost'], num_of_generated, root))
    num_of_generated += 1

    # cbs算法主循环
    iter = 0
    while openList and iter < maxIteration:
        _, _, node_id, p = heapq.heappop(openList)
        print('cbs iteration %d' % iter)
        print('p collision: ')
        print(p['collisions'])
        if not p['collisions']:
            print('solution found in iteration %d' % iter)
            return p['paths']
        constraints = standard_splitting(p['collisions'][0])
        for constraint in constraints:
            q = {'cost': 0,
                 'constraints': [],
                 'paths': [],
                 'collisions': []}
            q['constraints'] = p['constraints'].copy()
            agent = constraint['agent']
            if agent not in q['constraints']:
                q['constraints'][agent] = [constraint]
            elif constraint not in q['constraints'][agent]:
                q['constraints'][agent].append(constraint)
            q['paths'] = p['paths'].copy()
            print(q['constraints'][agent])
            path = astar(maps[agent], int(starts[agent][0]), int(starts[agent][1]), int(ends[agent][0]),
                         int(ends[agent][1]), directions[agent], q['constraints'][agent])
            if path:
                q['paths'][agent] = path
                q['collisions'] = detect_collisions(q['paths'])
                q['cost'] = get_sum_of_cost(q['paths'])
                heapq.heappush(openList, (len(q['collisions']), q['cost'], num_of_generated, q))
                num_of_generated += 1
        print('----------------------')
        iter += 1
    print('cbs no solution')
    return None




def cbs_reserve(maps, arrived_at_start, root_paths, starts, ends, directions, constraints, maxIteration):
    """
    功能：返回无冲突路径列表
    :param maps: list of 2d-np.array，栅格地图矩阵（0代表可通行，非0代表障碍物）的列表（由于每台AGV的目标货架处坐标只对自己是可通行的，因此每台AGV的地图各有不同）
    :param maps: list of lists of tuples，保留的初始路径（AGV更新时没走完的路）
    :param starts: list of tuples，起点坐标列表
    :param ends: list of tuples，终点坐标列表
    :param directions: list of strings，初始方向列表，形如 ['up', 'down', 'left', 'right'...]
    :param constraints: list of dicts, 初始约束列表，一般都是额外约束（在货架处或工作台处的停留）
    :param maxIteration: int，算法迭代次数上限
    :return: 如果在迭代上限内求出解则返回无冲突路径列表，否则返回None
    """
    constraint_table = {}  # 约束表，方便搜索
    for constraint in constraints:
        agent = constraint['agent']
        if agent in constraint_table:
            constraint_table[agent].append(constraint)
        else:
            constraint_table[agent] = [constraint]

    openList = [] # cbs算法中的open list，即待考虑的结点的列表
    num_of_generated = 0 # 由于heapq堆排的key是取列表中元组的第一个元素，相同的话取第二个，以此类推，如果出现tie的话会报错，因此引入子节点生成数作为唯一的key
    # 根节点
    root = {'cost': 0,
            'constraints': constraint_table,
            'paths': [],
            'collisions': []}

    # 为根节点生成初始路径列表
    for i in range(len(starts)):
        # 如果AGV i当前已有路径
        if i in root_paths:
            # 如果当前时刻有AGV刚刚回到起点，则地图上该AGV的起点变为不可通行，需要检查所保留的路径是否和该起点冲突
            if arrived_at_start:
                feasible = True
                for node in root_paths[i]:
                    if maps[i][node[0]][node[1]]:
                        feasible = False
                        break
                # 如果该路径可行
                if feasible:
                    path = root_paths[i]
                else:
                    if i in root['constraints']:
                        iconstraint = root['constraints'][i]
                    else:
                        iconstraint = []
                    path = astar(maps[i], int(starts[i][0]), int(starts[i][1]), int(ends[i][0]), int(ends[i][1]),
                                 directions[i],
                                 iconstraint)
            else:
                path = root_paths[i]
        # 如果AGV i当前没有路径
        else:
            if i in root['constraints']:
                iconstraint = root['constraints'][i]
            else:
                iconstraint = []
            path = astar(maps[i], int(starts[i][0]), int(starts[i][1]), int(ends[i][0]), int(ends[i][1]), directions[i],
                     iconstraint)
        if not path:
            raise BaseException('No solution for AGV%d' % i)
        root['paths'].append(path)

    root['cost'] = get_sum_of_cost(root['paths'])
    root['collisions'] = detect_collisions(root['paths'])
    # 为提高计算效率，采用堆排，理论上为取最优解应以cost为key，这里用碰撞数作为key是为了在短时间内优选算出可行解
    heapq.heappush(openList, (len(root['collisions']), root['cost'], num_of_generated, root))
    num_of_generated += 1

    # cbs算法主循环
    iter = 0
    while openList and iter < maxIteration:
        _, _, node_id, p = heapq.heappop(openList)
        print('cbs iteration %d' % iter)
        print('p collision: ')
        print(p['collisions'])
        if not p['collisions']:
            print('solution found in iteration %d' % iter)
            return p['paths']
        constraints = standard_splitting(p['collisions'][0])
        for constraint in constraints:
            q = {'cost': 0,
                 'constraints': [],
                 'paths': [],
                 'collisions': []}
            q['constraints'] = p['constraints'].copy()
            agent = constraint['agent']
            if agent not in q['constraints']:
                q['constraints'][agent] = [constraint]
            elif constraint not in q['constraints'][agent]:
                q['constraints'][agent].append(constraint)
            q['paths'] = p['paths'].copy()
            print(q['constraints'][agent])
            path = astar(maps[agent], int(starts[agent][0]), int(starts[agent][1]), int(ends[agent][0]),
                         int(ends[agent][1]), directions[agent], q['constraints'][agent])
            if path:
                q['paths'][agent] = path
                q['collisions'] = detect_collisions(q['paths'])
                q['cost'] = get_sum_of_cost(q['paths'])
                heapq.heappush(openList, (len(q['collisions']), q['cost'], num_of_generated, q))
                num_of_generated += 1
        print('----------------------')
        iter += 1
    print('cbs no solution')
    return None



def cbs_prioritized(maps, starts, ends, directions, constraints, maxIteration):
    """
    功能：结合cbs和prioritized，将同一终点（只可能是同一工作台）的AGV优先级设置为最高，返回无冲突路径列表
    :param maps: list of 2d-np.array，栅格地图矩阵（0代表可通行，非0代表障碍物）的列表（由于每台AGV的目标货架处坐标只对自己是可通行的，因此每台AGV的地图各有不同）
    :param starts: list of tuples，起点坐标列表
    :param ends: list of tuples，终点坐标列表
    :param directions: list of strings，初始方向列表，形如 ['up', 'down', 'left', 'right'...]
    :param constraints: list of dicts, 初始约束列表，一般都是额外约束（在货架处或工作台处的停留）
    :param maxIteration: int，算法迭代次数上限
    :return: 如果在迭代上限内求出解则返回无冲突路径列表，否则返回None
    """
    constraint_table = {}  # 约束表，方便搜索
    for constraint in constraints:
        agent = constraint['agent']
        if agent in constraint_table:
            constraint_table[agent].append(constraint)
        else:
            constraint_table[agent] = [constraint]

    paths = {}  # 路径字典，键为agv的id，值为路径
    indices = {}
    for i, loc in enumerate(ends):
        if loc in indices:
            indices[loc].append(i)
        else:
            indices[loc] = [i]

    prioritized_id = []  # 优先级高的AGV的id，即同一终点的AGV
    cbs_id = []  # 优先级低的AGV的id

    for loc, id_list in indices.items():
        if len(id_list) > 1:
            min_distance_id = 0
            distance = abs(starts[id_list[0]][0]-ends[id_list[0]][0]) + abs(starts[id_list[0]][1]-ends[id_list[0]][1])
            for i in range(len(id_list)):
                if distance > (abs(starts[id_list[i]][0]-ends[id_list[i]][0]) + abs(starts[id_list[i]][1]-ends[id_list[i]][1])):
                    distance = (abs(starts[id_list[i]][0]-ends[id_list[i]][0]) + abs(starts[id_list[i]][1]-ends[id_list[i]][1]))
                    min_distance_id = i
            prioritized_id.append(id_list[min_distance_id])
            id_list.pop(min_distance_id)
            cbs_id += id_list
        else:
            cbs_id += id_list

    print('prioritized_id: ', prioritized_id)
    print('cbs_id: ', cbs_id)

    # 对同一终点的AGV实施优先级规划
    for ind, i in enumerate(prioritized_id):
        print('prioritized astar %d start' % ind)
        print(constraint_table[i])
        path = astar(maps[i], int(starts[i][0]), int(starts[i][1]), int(ends[i][0]), int(ends[i][1]), directions[i],
                     constraint_table[i])
        print('prioritized astar %d finished' % ind)
        if not path:
            raise BaseException('No solution for AGV%d' % i)
        paths[i] = path
        len_path = len(path)
        for j in range(len_path):
            # vertex constraint
            for k in prioritized_id[ind + 1:] + cbs_id:
                if k in constraint_table:
                    constraint_table[k].append({'agent': k, 'loc': [path[j][:2]], 'timestep': path[j][2], 'type': 'negative'})
                else:
                    constraint_table[k] = [{'agent': k, 'loc': [path[j][:2]], 'timestep': path[j][2], 'type': 'negative'}]
            # edge constraint
                if j != 0:
                    if k in constraint_table:
                        constraint_table[k].append(
                            {'agent': k, 'loc': [path[j][:2], path[j - 1][:2]], 'timestep': path[j][2], 'type': 'negative'})
                    else:
                        constraint_table[k] = [
                            {'agent': k, 'loc': [path[j][:2], path[j - 1][:2]], 'timestep': path[j][2], 'type': 'negative'}]

    print('paths of prioritized: ', paths)
    openList = [] # cbs算法中的open list，即待考虑的结点的列表
    num_of_generated = 0 # 由于heapq堆排的key是取列表中元组的第一个元素，相同的话取第二个，以此类推，如果出现tie的话会报错，因此引入子节点生成数作为唯一的key

    # 根节点
    root = {'cost': 0,
            'constraints': constraint_table,
            'paths': [],
            'collisions': []}

    # 为根节点生成初始路径列表
    for ind, i in enumerate(cbs_id):
        print('root astar %d start' %ind)
        print('start: (%d, %d), end:(%d, %d)'%(int(starts[i][0]), int(starts[i][1]), int(ends[i][0]), int(ends[i][1])))
        print('constraints: ', root['constraints'][i])
        path = astar(maps[i], int(starts[i][0]), int(starts[i][1]), int(ends[i][0]), int(ends[i][1]), directions[i],
                     root['constraints'][i])
        if not path:
            raise BaseException('No solution for AGV%d' % i)
        root['paths'].append(path)
        print('root astar %d finished' %ind)

    root['cost'] = get_sum_of_cost(root['paths'])
    root['collisions'] = detect_collisions(root['paths'])
    # 为提高计算效率，采用堆排，理论上为取最优解应以cost为key，这里用碰撞数作为key是为了在短时间内优选算出可行解
    heapq.heappush(openList, (len(root['collisions']), root['cost'], num_of_generated, root))
    num_of_generated += 1
    # debug用
    for collision in root['collisions']:
        print(standard_splitting(collision))
    # cbs算法主循环
    iter = 0
    while openList and iter < maxIteration:
        print('cbs iteration %d' % iter)
        _, _, node_id, p = heapq.heappop(openList)
        if not p['collisions']:
            print('solution found in iteration %d' % iter)
            for ind, path in enumerate(p['paths']):
                paths[cbs_id[ind]] = path

            print('paths of all:', paths)
            paths_list = [value[1] for value in sorted(paths.items(), key=lambda x: x[0])]
            return paths_list
        constraints = standard_splitting(p['collisions'][0])
        for constraint in constraints:
            local_id = constraint['agent']
            constraint['agent'] = cbs_id[local_id]  # 将局部的id转为全局id
            q = {'cost': 0,
                 'constraints': [],
                 'paths': [],
                 'collisions': []}
            agent = constraint['agent']
            q['constraints'] = p['constraints'].copy()
            if constraint not in q['constraints'][agent]:
                q['constraints'][agent].append(constraint)
            q['paths'] = p['paths'].copy()
            path = astar(maps[agent], int(starts[agent][0]), int(starts[agent][1]), int(ends[agent][0]),
                         int(ends[agent][1]), directions[agent], q['constraints'][agent])
            if path:
                q['paths'][local_id] = path
                q['collisions'] = detect_collisions(q['paths'])
                print('q collision: ')
                print(q['collisions'])

                q['cost'] = get_sum_of_cost(q['paths'])
                heapq.heappush(openList, (len(q['collisions']), q['cost'], num_of_generated, q))
                num_of_generated += 1

        print('----------------------')
        iter += 1
    print('cbs no solution')
    return None