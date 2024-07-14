'''
JPath_Node:
    读取:
        parent_node()       -> JPath_Node       父节点
        current_index()     -> list | tuple     当前点(矩阵索引)
        goal_index()        -> list | tuple     目标点(矩阵索引)
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

from JAlgorithm_JMap_Matrix_Manager import *
from JAlgorithm_JPath_Action import *
from JetBot_Parameter_Const import *
from JLogger import *


log_info = logger_dict_algo['info']

# 节点类


class JPath_Node():
    def __init__(self, current_index: list, goal_index: list, parent_node: 'JPath_Node' = None) -> None:
        self.__parent_node = parent_node
        self.__current_index = current_index
        self.__goal_index = goal_index
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
    def current_index(self) -> list:
        return self.__current_index

    @property
    def goal_index(self) -> list:
        return self.__goal_index

    @property
    def cost(self) -> float:
        return self.__cost

    @property
    def heuristic_cost(self) -> float:
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
    def heuristic_function(self) -> float:
        c_x, c_y = self.__current_index
        g_x, g_y = self.__goal_index
        if not c_x or not c_y or not g_x or not g_y:
            log_info(f'c_x, c_y, g_x, g_y 值错误: \t {c_x}, {c_y}, {g_x}, {g_y}')
            return
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
        c_x, c_y = self.__current_index
        p_x, p_y = self.__parent_node.current_index
        x_cost = c_x - p_x
        y_cost = c_y - p_y
        cost = (x_cost ** 2 + y_cost ** 2) ** 0.5 + self.__parent_node.cost
        self.__cost = cost
        return cost


class JMap_Path_Node():
    def __init__(self, parent=None) -> None:
        self.parent_node: JMap_Path_Node = parent
        self.from_parent_action = None
        self.front = None
        self.back = None
        self.left_list = []
        self.right_list = []
        self.start_id = None
        self.start_distance = None
        self.end_distance = None
        self.children_list = []
        self.branch = []
        self.unpassed_branch = []
        self.flag_collected = False

    def get_whole_id_list(self):
        temp = []
        temp_sort = []
        if self.front:
            temp.append(self.front)
        if len(self.left_list) > 0:
            for i in self.left_list:
                if not i:
                    continue
                temp.append(i)
        if len(self.right_list) > 0:
            for i in self.right_list:
                if not i:
                    continue
                temp.append(i)
        for i in temp:
            if isinstance(i, list):
                for j in i:
                    temp_sort.append(j)
            else:
                temp_sort.append(i)
        # print(temp)
        # print(temp_sort)
        return temp_sort

    def append_branch(self, data):
        self.branch.append(data)
        self.unpassed_branch.append(data)

    def set_parent_action(self, action: str):
        if action == 'front':
            self.from_parent_action = 'front'
            self.right_list.append(self.parent_node.right_list[-1])
            self.left_list.append(self.parent_node.left_list[-1])
            self.start_id = self.parent_node.start_id
            self.back = self.parent_node.back
        elif action == 'left':
            self.from_parent_action = 'left'
            self.back = self.parent_node.right_list[-1]
            self.right_list.append(self.front)
            self.left_list.append(None)
        elif action == 'right':
            self.from_parent_action = 'right'
            self.back = self.parent_node.left_list[-1]
            self.left_list.append(self.front)
            self.right_list.append(None)
        else:
            log_info(f'[JMap_Path_Node][set_parent_action]: 参数 action 错误， 当前为 {action}')
