'''
JPath_Finding_A_Star
    主方法: 
        用于建立节点树, 并返回节点树
        get_node_tree(start_point: float|int, goal_point: float|int, map_matrix: JMap_Grid_Matrix_From_Yaml) -> list[JPath_Node]
    其他方法: 
        用于将坐标系转换为矩阵索引
        from_coordinate_to_index(coordinate: tuple[float|int, float|int] | list[float|int, float|int], map_matrix: JMap_Grid_Matrix_From_Yaml) -> tuple[int]
        用于将矩阵索引转换为坐标系, 点为方格中点
        from_index_to_coordinate(index: list[int, int], map_matrix: JMap_Grid_Matrix_From_Yaml) -> tuple[float|int]

JPath_Tree
    读取: 
        tree_list() -> list[JPath_Node]

JPath_Node: 
    读取: 
        parent_node()       -> JPath_Node       父节点
        current_point()     -> list | tuple     当前点(矩阵索引)
        goal_point()        -> list | tuple     目标点(矩阵索引)
        cost()              -> float            损失值
        heuristic_cost()    -> float | int      启发函数值
        value()             -> float            总值
        scaned_flag()       -> bool             已扫描标志
    写入: 
        set_scaned_flag(flag: bool) -> None
    计算方法: 
        heuristic_function()    -> float | int  启发式函数
        cost_function()         -> float        损失函数
'''

from JAlgorithm_Map_Matrix_Manager import *
from copy import deepcopy

# 节点类


class JPath_Node():
    def __init__(self, current_point: list | tuple, goal_point: list | tuple, parent_node: 'JPath_Node' = None) -> None:
        self.__parent_node = parent_node
        self.__current_point = current_point
        self.__goal_point = goal_point
        self.__heuristic_estimate = 0
        self.__cost = 0
        self.__value = 0
        self.__scaned_flag = False
        self.heuristic_function()
        self.cost_function()

    @property
    def parent_node(self) -> 'JPath_Node':
        return self.__parent_node

    @property
    def current_point(self) -> list | tuple:
        return self.__current_point

    @property
    def goal_point(self) -> list | tuple:
        return self.__goal_point

    @property
    def cost(self) -> float:
        return self.__cost

    @property
    def heuristic_cost(self) -> float | int:
        return self.__heuristic_estimate

    @property
    def value(self) -> float:
        self.__value = self.__heuristic_estimate + self.__cost
        return self.__value

    @property
    def scaned_flag(self) -> bool:
        return self.__scaned_flag

    # 设置已扫描标志
    def set_scaned_flag(self, flag: bool) -> None:
        self.__scaned_flag = flag

    # 计算启发式函数
    def heuristic_function(self) -> float | int:
        c_x, c_y = self.__current_point
        g_x, g_y = self.__goal_point
        x_cost = g_x - c_x
        y_cost = g_y - c_y
        x_estimate_positiv = abs(x_cost)
        y_estimate_positiv = abs(y_cost)
        self.__heuristic_estimate = x_estimate_positiv + y_estimate_positiv
        return self.__heuristic_estimate

    # 计算损失函数
    def cost_function(self) -> float:
        if not self.__parent_node:
            return None
        c_x, c_y = self.__current_point
        p_x, p_y = self.__parent_node.current_point
        x_cost = c_x - p_x
        y_cost = c_y - p_y
        cost = (x_cost ** 2 + y_cost ** 2) ** 0.5 + self.__parent_node.cost
        self.__cost = cost
        return cost


class JPath_Tree():
    def __init__(self, start_point: list | tuple, goal_point: list | tuple, map_matrix: JMap_Grid_Matrix_From_Yaml) -> None:
        self.start_point = start_point
        self.goal_point = goal_point
        self.map_matrix = map_matrix.map_abstract_matrix
        self.matrix = deepcopy(self.map_matrix)
        self.matrix[self.start_point[0]][self.start_point[1]
                                         ] = JPath_Node(self.start_point, self.goal_point)
        self.__tree_list = []
        self.__node_list = []
        self.__end_node = None
        self.scan_matrix()

    @property
    def tree_list(self) -> list[JPath_Node]:
        return self.__tree_list

    # 先找节点, 排序子节点, 永远从排序中最小且未被扫描的节点开始寻找子节点, 当节点的坐标为终点时, 停止循环
    def scan_matrix(self) -> None:
        parent_node = JPath_Node(self.start_point, self.goal_point)
        self.__node_list.append(parent_node)
        # 循环直至找到终点
        while not self.__end_node:
            # 读取self.__node_list列表最小元素, 判断是否为终点, 若非终点, 则扩展子节点
            # for i in self.__node_list:
            #     i:JPath_Node
            #     if not i.scaned_flag:
            #         read_node = i
            #         i.set_scaned_flag(True)
            for i in self.__node_list:
                i: JPath_Node
                # print(i.current_point, i.value)
                # print('g\t',i.goal_point)
            read_node: JPath_Node = self.__node_list.pop(0)
            if read_node.current_point[0] == self.goal_point[0] and read_node.current_point[1] == self.goal_point[1]:
                self.__end_node = read_node
                break
            self.find_child_node(read_node)
        self.find_parent_node(self.__end_node)

    # 寻找子节点, 有两种方法, 一种是周围4格扩展, 一种是周围8格扩展
    def find_child_node(self, parent_node: JPath_Node) -> None:
        p_x, p_y = parent_node.current_point
        temp_child_node_list = []
        # 八格搜寻
        extension_list = [[p_x-1, p_y-1], [p_x, p_y-1], [p_x+1, p_y-1],
                          [p_x-1, p_y], [p_x+1, p_y],
                          [p_x-1, p_y+1], [p_x, p_y+1], [p_x+1, p_y+1]]
        # 四格搜寻
        # extension_list =[[p_x-1, p_y], [p_x+1, p_y], [p_x, p_y-1], [p_x, p_y+1]]
        for i in extension_list:
            new_x = i[0]
            new_y = i[1]
            if 0 <= new_x < len(self.matrix) and 0 <= new_y < len(self.matrix[0]):
                new_element = self.matrix[new_x][new_y]
            else:
                continue
            if new_element == 1 and not isinstance(new_element, JPath_Node):
                new_element = JPath_Node(
                    [new_x, new_y], self.goal_point, parent_node)
                self.matrix[new_x][new_y] = new_element
                temp_child_node_list.append(new_element)
        # print(temp_child_node_list)

        # 进行排序, 按照value从小到大, 先看value值大小, 若相等则看heuristic_cost大小
        if not temp_child_node_list:
            return None
        for element in temp_child_node_list:
            element: JPath_Node
            if not self.__node_list:
                self.__node_list.append(element)
                continue
            sub_node_list = self.__node_list[:]
            for index, item in enumerate(sub_node_list):
                item: JPath_Node
                inserted = False
                if element.value < item.value or (element.value == item.value and element.heuristic_cost < item.heuristic_cost):
                    self.__node_list.insert(index, element)
                    inserted = True
                    break
                if not inserted:
                    self.__node_list.append(element)
                # print(self.__node_list)

    # 寻找父节点, 进行递归
    def find_parent_node(self, child_node: JPath_Node) -> None:
        if not self.__tree_list:
            self.__tree_list.append(child_node)
        if child_node.parent_node:
            self.__tree_list.insert(0, child_node.parent_node)
            self.find_parent_node(child_node.parent_node)

# A*算法类


class JPath_Finding_A_Star():
    def __init__(self) -> None:
        pass

    # 将坐标系转换为索引
    def from_coordinate_to_index(self, coordinate: tuple[float | int, float | int] | list[float | int, float | int], map_matrix: JMap_Grid_Matrix_From_Yaml) -> tuple[int]:
        # coordinate: (x, y)
        temp_index = [None, None]
        for x_index, x_item in enumerate(map_matrix.map_obj_matrix):
            x_item: list[JWall]
            for y_index, y_item in enumerate(x_item):
                y_item: JWall
                if not y_item:  # 排除None值
                    continue
                # 判断条件: 方向为水平, 输入的 y 值小于当前板子的右边界, 同时 temp_index 为第一次赋值
                if y_item.orientation == 'H' and coordinate[1] < y_item.rightside and not temp_index[1]:
                    temp_index[1] = y_index
                    break
                elif y_item.orientation == 'H' and coordinate[1] == y_item.rightside or coordinate[1] == y_item.leftside:
                    print('请检查输入值, 输入值 y 与墙重叠')
                    raise ValueError('请检查输入值, 输入值 y 与墙重叠')
                # 判断条件: 方向为竖直, 输入的 x 值小于当前板子的下边界, 同时 temp_index 为第一次赋值
                if y_item.orientation == 'V' and coordinate[0] < y_item.bottomside and not temp_index[0]:
                    temp_index[0] = x_index
                    break
                elif y_item.orientation == 'V' and (coordinate[0] == y_item.bottomside or coordinate[0] == y_item.topside):
                    print('请检查输入值, 输入值 x 与墙重叠')
                    raise ValueError('请检查输入值, 输入值 x 与墙重叠')
        return temp_index[0], temp_index[1]

    # 将索引转换为坐标系, 返回中心点的坐标系
    def from_index_to_coordinate(self, index: list[int, int], map_matrix: JMap_Grid_Matrix_From_Yaml) -> tuple[float | int]:
        # index: (x, y)
        x_coordinate = (index[0] * 0.1265) + 0.0015
        y_coordinate = (index[1] * 0.1265) + 0.0015
        return x_coordinate, y_coordinate

    # 主要方法, 计算并获得路径节点列表
    def get_node_tree(self, start_point: float | int, goal_point: float | int, map_matrix: JMap_Grid_Matrix_From_Yaml) -> list[JPath_Node]:
        start_point = self.from_coordinate_to_index(start_point, map_matrix)
        goal_point = self.from_coordinate_to_index(goal_point, map_matrix)
        tree_model = JPath_Tree(start_point, goal_point, map_matrix)
        return tree_model.tree_list
