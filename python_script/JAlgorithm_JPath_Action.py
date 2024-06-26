import math
from JAlgorithm_JPath_Node import *


class JAction():
    def __init__(self) -> None:
        self.__orientation: float = None
        self.__start_node: JPath_Node = None
        self.__end_node: JPath_Node = None
        self.__vector: list = None
        self.__path_length: float = None
        self.__mode: str = None
        self.__last_node: JPath_Node = None
        self.__last_vector: list = None
        self.__turn_radius = None
        self.__turn_angle = None
        self.__index_x_increase: int = 0
        self.__index_y_increase: int = 0

    @property
    def orientation(self):
        return self.__orientation

    @property
    def start_node(self):
        return self.__start_node

    @property
    def end_node(self):
        return self.__end_node

    @property
    def vector(self):
        return self.__vector

    @property
    def path_length(self):
        return self.__path_length

    @property
    def mode(self):
        return self.__mode

    @property
    def last_vector(self):
        return self.__last_vector

    @property
    def index_x_increase(self):
        return self.__index_x_increase

    @property
    def index_y_increase(self):
        return self.__index_y_increase

    def set_start_node(self, node):
        self.__start_node = node
        if self.__mode == 'circle' and self.__end_node and self.__last_node:
            self.__set_turn_parameter()

    def set_end_node(self, node):
        self.__end_node = node
        self.__vector = [self.__end_node.current_index[0] - self.__start_node.current_index[0], self.__end_node.current_index[1] - self.__start_node.current_index[1]]
        self.__path_length = (self.__vector[0] ** 2 + self.__vector[1] ** 2) ** 0.5
        self.__orientation = self.get_angle(self.__vector)
        if self.__orientation == 0:
            self.__index_x_increase = 1
        elif self.__orientation == 90:
            self.__index_y_increase = 1
        elif self.__orientation == 180:
            self.__index_x_increase = -1
        elif self.__orientation == -90:
            self.__index_y_increase = -1
        if self.__mode == 'circle' and self.__start_node and self.__last_node:
            self.__set_turn_parameter()

    def get_angle(self, vector):
        return math.degrees(math.atan2(vector[1], vector[0]))

    def set_mode(self, mode):
        if mode not in ['line', 'circle']:
            raise ValueError(f'set_mode只有两个模式, line 和 circle, 当前输入为 {mode}')
        self.__mode = mode
        if self.__mode == 'circle' and self.__end_node and self.__last_node and self.__start_node:
            self.__set_turn_parameter()

    def set_last_node(self, last_node):
        self.__last_node = last_node
        if self.__mode == 'circle' and self.__end_node and self.__start_node:
            self.__set_turn_parameter()

    def __set_turn_parameter(self):
        self.__last_vector = [self.__start_node.current_index[0] - self.__last_node.current_index[0], self.__start_node.current_index[1] - self.__last_node.current_index[1]]
        angle_between = math.acos((self.__last_vector[0]*self.__vector[0] + self.__last_vector[1]*self.__vector[1]) /
                                  (((self.__last_vector[0]**2 + self.__last_vector[1]**2) ** 0.5)*(self.__vector[0]**2 + self.__vector[1]**2) ** 0.5))
        self.__turn_radius = ((self.__last_vector[0] ** 2 + self.__last_vector[1] ** 2) ** 0.5) * math.tan((math.pi - angle_between)/2)
        self.__turn_angle = angle_between / 2


# def get_angle(vector):
#     return math.degrees(math.atan2(vector[1], vector[0]))


# print(get_angle([0, 2]))
