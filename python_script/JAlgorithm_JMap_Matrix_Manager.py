'''
JMap_Grid_Matrix_From_Yaml  读取Yaml文件, 建立抽象和对象地图矩阵
    读取: 
        map_obj_matrix          对象地图矩阵, 矩阵中是Jwall对象, 即含有墙的信息
        map_abstract_matrix     抽象地图矩阵, 矩阵中只含有Jwall的墙体标记, 底层矩阵为全是 1 的矩阵
    主要方法: 
        JMap_generator (yaml_data: yaml) -> None    将Yaml文件转换为地图矩阵, 并存入属性中
'''
from JAlgorithm_JWall import *
from typing import Union

# 读取Yaml类


class JMap_Grid_Matrix_From_Yaml():
    def __init__(self, yaml_data: yaml) -> None:
        self.origin_JWall_list = []
        self.__map_obj_matrix: list = []
        self.__map_abstract_matrix: list = []
        self.JMap_generator(yaml_data)

    @property
    def map_obj_matrix(self):
        return self.__map_obj_matrix

    @property
    def map_abstract_matrix(self):
        return self.__map_abstract_matrix

    def JMap_generator(self, yaml_data: yaml) -> None:
        JWalls_obj = JWalls()
        JWalls_obj.build_JWall_list_from_yaml(yaml_data)
        self._from_JWall_list_generate_map(JWalls_obj)

    # 获取含有对象的矩阵
    def _from_JWall_list_generate_map(self, JWall_list: JWalls) -> None:
        temp_obj_matrix = [[None for _ in range(JWall_list.range_Maze_y * 2 + 1)] for _ in range(JWall_list.range_Maze_x * 2 + 1)]
        for _, wall in enumerate(JWall_list.JWall_list):
            wall: JWall
            x_index = int((wall.middle.x() - 0.0015) / 0.1265)  # 索引坐标(以中点计算) = 1.5 + (125 + 1.5) * 索引值 n
            y_index = int((wall.middle.y() - 0.0015) / 0.1265)
            temp_obj_matrix[x_index][y_index] = wall
        self.__map_obj_matrix = temp_obj_matrix
        self._from_map_obj_to_abstract_matrix()

    # 获取抽象矩阵
    def _from_map_obj_to_abstract_matrix(self) -> None:
        temp_abstract_matrix = [[1 for _ in range(len(self.__map_obj_matrix))] for _ in range(len(self.__map_obj_matrix))]
        for x_index, wall_list in enumerate(self.__map_obj_matrix):
            for y_index, wall in enumerate(wall_list):
                wall: JWall
                if wall:
                    temp_abstract_matrix[x_index][y_index] = wall.identifier
                    if wall.orientation == 'H':    # 横向延伸
                        temp_abstract_matrix[x_index][max(0, y_index - 1)] = wall.sub_identifier                  # 向左延伸一格墙壁，取0和索引-1两者最大值，避免出现负数
                        temp_abstract_matrix[x_index][min(y_index + 1, len(wall_list)-1)] = wall.sub_identifier   # 向右延伸一格墙壁，取索引+1和列表最大索引两者最大值，避免出现索引超出范围
                    else:                           # 纵向延伸
                        temp_abstract_matrix[max(0, x_index - 1)][y_index] = wall.sub_identifier                  # 向上延伸一格墙壁，取0和索引-1两者最大值，避免出现负数
                        temp_abstract_matrix[min(x_index + 1, len(wall_list)-1)][y_index] = wall.sub_identifier   # 向下延伸一格墙壁，取索引+1和列表最大索引两者最大值，避免出现索引超出范围
        self.__map_abstract_matrix = temp_abstract_matrix

    def from_id_to_index(self, id: int, distance: float = 0.0001):
        index = [None, None]
        distance = abs(distance)
        for x_index, list_item in enumerate(self.map_obj_matrix):
            for y_index, i in enumerate(list_item):
                i: JWall
                if i and id in i.id_list:
                    if id % 4 == 0 or id % 4 == 1:
                        if i.orientation == 'H':    # 向上
                            x_index = i.topside-i.width/2 - distance
                            y_index = i.middle.y()
                        elif i.orientation == 'V':  # 向左
                            x_index = i.middle.x()
                            y_index = i.leftside-i.width/2 - distance
                    elif id % 4 == 2 or id % 4 == 3:
                        if i.orientation == 'H':    # 向下
                            x_index = i.topside+i.width/2 + distance
                            y_index = i.middle.y()
                        elif i.orientation == 'V':  # 向右
                            x_index = i.middle.x()
                            y_index = i.leftside+i.width/2 + distance
                    if x_index < 0 or y_index < 0:
                        print('请正确输入id和距离, 当前id号和距离对应部分超出地图范围')
                        return None
                    index = self.from_coordinate_to_index([x_index, y_index])
                    return index

    # 将坐标系转换为索引
    def from_coordinate_to_index(self, coordinate: list) -> tuple:
        # coordinate: [[x, y]
        if not coordinate[0] or not coordinate[1]:
            raise ValueError(f'方法 from_coordinate_to_index 中, 输入含有 None 值: {coordinate[0]} {coordinate[1]}')
        temp_index = [None, None]
        for x_index, x_item in enumerate(self.map_obj_matrix):
            x_item: list
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
        return [temp_index[0], temp_index[1]]

    # 将索引转换为坐标系, 返回中心点的坐标系
    def from_index_to_coordinate(self, index: list) -> tuple:
        # index: (x, y)
        x_coordinate = (index[0] * 0.1265) + 0.0015
        y_coordinate = (index[1] * 0.1265) + 0.0015
        return x_coordinate, y_coordinate
