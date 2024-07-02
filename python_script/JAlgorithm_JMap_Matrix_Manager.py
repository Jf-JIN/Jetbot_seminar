'''
JMap_Grid_Matrix_From_Yaml  读取Yaml文件, 建立抽象和对象地图矩阵
    读取:
        map_obj_matrix          对象地图矩阵, 矩阵中是Jwall对象, 即含有墙的信息
        map_abstract_matrix     抽象地图矩阵, 矩阵中只含有Jwall的墙体标记, 底层矩阵为全是 1 的矩阵
    主要方法:
        JMap_generator (yaml_data: yaml) -> None    将Yaml文件转换为地图矩阵, 并存入属性中
'''
from JAlgorithm_JWall import *
from JetBot_Parameter import *
from JLocation import *
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
            x_index = int((wall.middle.x() - WALL_THICKNESS_HALF) / (WALL_THICKNESS_HALF+WALL_WIDTH_HALF))  # 索引坐标(以中点计算) = 1.5 + (125 + 1.5) * 索引值 n
            y_index = int((wall.middle.y() - WALL_THICKNESS_HALF) / (WALL_THICKNESS_HALF+WALL_WIDTH_HALF))
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
                        temp_abstract_matrix[x_index][max(0, y_index - 1)] = wall.sub_identifier                  # 向左延伸一格墙壁, 取0和索引-1两者最大值, 避免出现负数
                        temp_abstract_matrix[x_index][min(y_index + 1, len(wall_list)-1)] = wall.sub_identifier   # 向右延伸一格墙壁, 取索引+1和列表最大索引两者最大值, 避免出现索引超出范围
                    else:                          # 纵向延伸
                        temp_abstract_matrix[max(0, x_index - 1)][y_index] = wall.sub_identifier                  # 向上延伸一格墙壁, 取0和索引-1两者最大值, 避免出现负数
                        temp_abstract_matrix[min(x_index + 1, len(wall_list)-1)][y_index] = wall.sub_identifier   # 向下延伸一格墙壁, 取索引+1和列表最大索引两者最大值, 避免出现索引超出范围
        self.__map_abstract_matrix = temp_abstract_matrix

    def from_id_to_index(self, id: int, distance: float = 0.0001):
        index = [None, None]
        distance = abs(distance)
        for x_index, list_item in enumerate(self.map_obj_matrix):
            for y_index, i in enumerate(list_item):
                i: JWall
                if i and id in i.id_list:
                    if id in [i.main_0.id, i.main_1.id]:    # 正面 向上或向左
                        if i.orientation == 'H':    # 向上， 车位于码的上方
                            x_index = i.topside - distance
                            y_index = i.middle.y()
                        elif i.orientation == 'V':  # 向左， 车位于码的左侧
                            x_index = i.middle.x()
                            y_index = i.leftside - distance
                    elif id in [i.sub_0.id, i.sub_1.id]:    # 背面 向下或向右
                        if i.orientation == 'H':    # 向下， 车位于码的下方
                            x_index = i.topside + distance
                            y_index = i.middle.y()
                        elif i.orientation == 'V':  # 向右， 车位于码的右侧
                            x_index = i.middle.x()
                            y_index = i.leftside + distance
                    if x_index < 0 or y_index < 0:
                        print(f'请正确输入id和距离, 当前id号({id})和距离{distance}对应部分x: {x_index} y: {y_index}超出地图范围')
                        return None

                    print(f'当前id号({id})，距离：{distance}, 索引: [{x_index}, {y_index}]')
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
                    print(f'[板子y]: {y_item.topleft, y_item.bottomright}')
                    break
                elif y_item.orientation == 'H' and coordinate[1] == y_item.rightside or coordinate[1] == y_item.leftside:
                    print('请检查输入值, 输入值 y 与墙重叠')
                    raise ValueError('请检查输入值, 输入值 y 与墙重叠')
                # 判断条件: 方向为竖直, 输入的 x 值小于当前板子的下边界, 同时 temp_index 为第一次赋值
                if y_item.orientation == 'V' and coordinate[0] < y_item.bottomside and not temp_index[0]:
                    print(f'[板子x]: {y_item.topleft, y_item.bottomright}')
                    temp_index[0] = x_index
                    break
                elif y_item.orientation == 'V' and (coordinate[0] == y_item.bottomside or coordinate[0] == y_item.topside):
                    print('请检查输入值, 输入值 x 与墙重叠')
                    raise ValueError('请检查输入值, 输入值 x 与墙重叠')
        return [temp_index[0], temp_index[1]]

    # 将索引转换为坐标系, 返回中心点的坐标系
    def from_index_to_coordinate(self, index: list) -> tuple:
        # index: (x, y)
        x_coordinate = (index[0] * (WALL_THICKNESS_HALF+WALL_WIDTH_HALF)) + WALL_THICKNESS_HALF
        y_coordinate = (index[1] * (WALL_THICKNESS_HALF+WALL_WIDTH_HALF)) + WALL_THICKNESS_HALF
        return x_coordinate, y_coordinate


class JMap_Builder():
    def __init__(self) -> None:
        pass

    def scan_path(self, matrix, jlocation: JLocation):
        front_apr = jlocation.front.front
        front_wall = JWall()
        front_wall.set_wall_from_apriltag(front_apr)
        distance_index_raw = (jlocation.front.front.distance.z() + JB_DISTANCE_FROM_CAMERA_TO_CENTER) / front_wall.width
        temp_matrix = []
        if distance_index_raw > 1:
            temp_line = [None for _ in int(distance_index_raw)]
            temp_line[-1] = front_wall
            temp_matrix.append(temp_line)
        if jlocation.left_list:
            temp_line = [None for _ in len(jlocation.left_list)]
            left_0_apr = jlocation.left_list[0]
            if left_0_apr:
                left_0_wall = JWall()
                left_0_wall.set_wall_from_apriltag(left_0_apr)
                temp_line[0] = left_0_wall
            if len(jlocation.left_list) == 2:
                left_1_apr = jlocation.left_list[1]
                if left_1_apr:
                    left_1_wall = JWall()
                    left_1_wall.set_wall_from_apriltag(left_1_apr)
        if jlocation.right_list:
            right_0_apr = jlocation.right_list[0]
            if right_0_apr:
                right_0_wall = JWall()
                right_0_wall.set_wall_from_apriltag(right_0_apr)
            if len(jlocation.right_list) == 2:
                right_1_apr = jlocation.right_list[1]
                if right_1_apr:
                    right_1_wall = JWall()
                    right_1_wall.set_wall_from_apriltag(right_1_apr)

        pass

    def extend_map_matix(self, matix):

        pass

    def matrix_rotation(self, matrix, mode: int = 0):   # mode 为奇数, 表示顺时针, 为偶数, 表示逆时针
        # 顺时针
        if (-1)**mode == 1:
            transposed = list(zip(*matrix))
            return [list(row)[::-1] for row in transposed]
        # 逆时针
        else:
            transposed = list(zip(*matrix))
            return [list(row) for row in transposed][::-1]


'''
使用路径作为建立地图的工具
先是获取所有路径的节点, 在节点中记录相关的信息, 以起始点作为零点, 当扫描到两个方向的Apriltag后, 即可确定方向, 随后可生成yaml文件, 
根据路径的节点, 对yaml文件进行更新、扩展。
然后当完全扫描完后, 将零点移动到最小值处, 然后重新更新yaml文件
'''
