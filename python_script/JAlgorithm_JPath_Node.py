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
            raise ValueError(f'c_x, c_y, g_x, g_y 值错误: \t {c_x}, {c_y}, {g_x}, {g_y}')
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
