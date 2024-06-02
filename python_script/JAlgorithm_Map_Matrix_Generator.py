
from JAlgorithm_JWall import *

class JMap_Grid_Matrix_From_Yaml():
    def __init__(self) -> None:
        self.origin_JWall_list = []
        self.__map_obj_matrix:list[list[JWall]] = []
        self.__map_abstract_matrix:list[list[int|bool|str]] = []
    
    @property
    def map_obj_matrix(self):
        return self.__map_obj_matrix
    
    @property
    def map_abstract_matrix(self):
        return self.__map_abstract_matrix
    
    def JMap_generator(self, yaml_data: yaml):
        JWalls_obj = JWalls()
        JWalls_obj.build_JWall_list_from_yaml(yaml_data)
        self._from_JWall_list_generate_map(JWalls_obj)
        
    # 获取含有对象的矩阵
    def _from_JWall_list_generate_map(self, JWall_list:JWalls) -> list[list[JWall]]:
        temp_obj_matrix = [[None for _ in range(JWall_list.range_Maze_y * 2 + 1)] for _ in range(JWall_list.range_Maze_x * 2 + 1)]
        for _, wall in enumerate(JWall_list.JWall_list):
            wall:JWall
            x_index = int((wall.middle.x() - 0.0015) / 0.1265)
            y_index = int((wall.middle.y() - 0.0015) / 0.1265)
            temp_obj_matrix[x_index][y_index] = wall
        self.__map_obj_matrix = temp_obj_matrix
        self._from_map_obj_to_abstract_matrix()
    
    # 获取抽象矩阵
    def _from_map_obj_to_abstract_matrix(self):
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
                        temp_abstract_matrix[min(x_index + 1,len(wall_list)-1)][y_index] = wall.sub_identifier   # 向下延伸一格墙壁，取索引+1和列表最大索引两者最大值，避免出现索引超出范围
        self.__map_abstract_matrix = temp_abstract_matrix