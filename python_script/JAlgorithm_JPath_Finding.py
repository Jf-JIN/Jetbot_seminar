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
log_info = logger_dict_algo['info']


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
            # 方向相同，延长终点
            if self.get_angle(old_vector) == self.get_angle(new_vector):
                action.set_end_node(new_node)
                # action.set_mode('line')
                old_node = new_node
                old_vector = new_vector
            # 方向不同，转弯节点，保存之前的节点
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
                action.set_end_node(new_node)
                old_node = new_node
                old_vector = new_vector
            # 最后节点
            if index == len(node_tree) - 1:
                action.set_end_node(item)
                path_list.append(action)
        # log_info(len(path_list))
        # log_info(path_list)
        temp_path = []
        for index, i in enumerate(path_list):
            i: JAction
            # 寻找目标墙
            goal_wall = None
            # 纵向延伸寻找
            if i.index_x_increase:
                x_index = i.end_node.current_index[0]
                while not goal_wall:
                    x_index += i.index_x_increase
                    goal_wall_index = [x_index, i.end_node.current_index[1]]
                    log_info(goal_wall_index)
                    goal_wall: JWall = map_matrix.map_obj_matrix[goal_wall_index[0]][goal_wall_index[1]]
                distance = abs(x_index - i.end_node.current_index[0] - i.index_x_increase) * goal_wall.width / 2 + goal_wall.width / 2
            # 横向延伸寻找
            elif i.index_y_increase:
                y_index = i.end_node.current_index[1]
                while not goal_wall:
                    y_index += i.index_y_increase
                    goal_wall_index = [i.end_node.current_index[0], y_index]
                    goal_wall: JWall = map_matrix.map_obj_matrix[goal_wall_index[0]][goal_wall_index[1]]
                distance = abs(y_index - i.end_node.current_index[1] - i.index_y_increase) * goal_wall.width / 2 + goal_wall.width / 2
            if i.index_x_increase == 1 or i.index_y_increase == 1:  # 向右向下，看到的id是向左向上
                id_list = [goal_wall.id_list[0], goal_wall.id_list[1]]
            if i.index_x_increase == -1 or i.index_y_increase == -1:    # 向左向上，看到的id是向右向下
                id_list = [goal_wall.id_list[2], goal_wall.id_list[3]]
            if index < len(path_list)-1:
                next_vector = path_list[index+1].vector
                theta_rad = math.acos((i.vector[0]*next_vector[0]+i.vector[1]*next_vector[1])/(((i.vector[0]**2 + i.vector[1]**2) ** 0.5) * ((next_vector[0]**2 + next_vector[1]**2) ** 0.5)))
                theta = math.degrees(theta_rad)
                if i.vector[0]*next_vector[1]-i.vector[1]*next_vector[0] < 0:
                    theta = -theta
                if theta > 0:
                    end_orientation = 'turn_left'
                elif theta < 0:
                    end_orientation = 'turn_right'
                else:
                    end_orientation = 'continue'
            else:
                end_orientation = 'end'
            temp_action_dict = {
                'mode': 'line',
                'goal_id': id_list,
                'end_pos': distance,
                'end_orientation': end_orientation
            }
            temp_path.append(temp_action_dict)
        log_info('[分解路径]')
        for i in temp_path:
            print(i)
        log_info('[路径字典]')
        log_info(temp_path)
        return temp_path
