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
'''

from JAlgorithm_JPath_Tree import *
from JAlgorithm_JPath_Action import *

# A*算法类


class JPath_Finding_A_Star():
    def __init__(self) -> None:
        pass

    # 主要方法, 计算并获得路径节点列表
    def get_node_tree(self, start_index: float, goal_index: float, map_matrix: JMap_Grid_Matrix_From_Yaml) -> list:
        tree_model = JPath_Tree(start_index, goal_index, map_matrix)
        return tree_model.tree_list

    def get_angle(self, vector):
        return math.degrees(math.atan2(vector[1], vector[0]))

    def get_path_tree(self, start_index: int, goal_index: int, map_matrix: JMap_Grid_Matrix_From_Yaml):
        node_tree = self.get_node_tree(start_index, goal_index, map_matrix)
        old_node = None
        old_vector = None
        path_list = []
        action = None
        for index, item in enumerate(node_tree):
            index: int
            item: JPath_Node
            new_node = item
            # 如果只读取一个节点，则无法分析角度，继续读取，并存档旧节点
            if index == 0:
                old_node = new_node
                action = JAction()
                action.set_start_node(old_node)
                continue
            new_vector = [new_node.current_index[0] - old_node.current_index[0], new_node.current_index[1] - old_node.current_index[1]]
            # 如果读取了两个节点，可以分析角度，但不能分析走势，继续读取，并存档旧向量
            if not old_vector:
                old_node = new_node
                old_vector = new_vector
                action.set_end_node(new_node)
                continue
            if self.get_angle(old_vector) == self.get_angle(new_vector):
                action.set_end_node(new_node)
                # action.set_mode('line')
                old_node = new_node
                old_vector = new_vector
            else:
                # action.set_end_node(node_tree[index-2])  # 这里看是否退后一个节点，使得转弯处连贯
                path_list.append(action)
                # turn_action = JAction()
                # turn_action.set_start_node(old_node)
                # turn_action.set_end_node(new_node)
                # turn_action.set_mode('circle')
                # turn_action.set_last_node(node_tree[index-2])
                # path_list.append(turn_action)
                action = JAction()
                # action.set_start_node(new_node)
                action.set_start_node(old_node)
                # print('拐点:', old_node.current_index)
                action.set_end_node(new_node)
                old_node = new_node
                old_vector = new_vector
            if index == len(node_tree) - 1:
                action.set_end_node(item)
                path_list.append(action)
        # print(len(path_list))
        # print(path_list)
        temp_path = []
        for i in path_list:
            # print('\ncurrent:\t', i.end_node.current_index)
            # print('increase:\t', i.index_x_increase, i.index_y_increase)
            i: JAction
            # 寻找目标墙
            goal_wall = None
            if i.index_x_increase:
                x_index = i.end_node.current_index[0]
                while not goal_wall:
                    x_index += i.index_x_increase
                    goal_wall_index = [x_index, i.end_node.current_index[1]]
                    goal_wall: JWall = map_matrix.map_obj_matrix[x_index][i.end_node.current_index[1]]
                distance = abs(x_index - i.end_node.current_index[0] - i.index_x_increase) * goal_wall.width + goal_wall.width / 2
            elif i.index_y_increase:
                y_index = i.end_node.current_index[1]
                while not goal_wall:
                    y_index += i.index_y_increase
                    goal_wall_index = [i.end_node.current_index[0], y_index]
                    goal_wall: JWall = map_matrix.map_obj_matrix[i.end_node.current_index[0]][y_index]
                distance = abs(y_index - i.end_node.current_index[1] - i.index_y_increase) * goal_wall.width + goal_wall.width / 2
            # print('goal_wall\t', goal_wall_index, goal_wall)
            # print('distance\t', distance)
            if i.index_x_increase == 1 or i.index_y_increase == 1:  # 向右向下，看到的id是向左向上
                id_list = [goal_wall.id_list[0], goal_wall.id_list[1]]
            if i.index_x_increase == -1 or i.index_y_increase == -1:    # 向左向上，看到的id是向右向下
                id_list = [goal_wall.id_list[2], goal_wall.id_list[3]]

            temp_action_dict = {
                'mode': 'line',
                'goal_id': id_list,
                'end_pos': distance
            }
            temp_path.append(temp_action_dict)
            # print(i,  i.vector, i.path_length, i.orientation, i.start_node.current_index, i.end_node.current_index)
        # for i in temp_path:
        #     print(i)
        # print('\n')
        # print(temp_path)
        return temp_path
