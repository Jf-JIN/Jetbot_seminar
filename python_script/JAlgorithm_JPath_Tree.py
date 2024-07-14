'''
JPath_Tree
    读取: 
        tree_list() -> list[JPath_Node]
'''

from JAlgorithm_JPath_Node import *
from copy import deepcopy

# 节点树类

info = logger_dict_algo['info']


class JPath_Tree():
    def __init__(self, start_index: Union[list, tuple], goal_index: Union[list, tuple], map_matrix: JMap_Grid_Matrix_From_Yaml) -> None:
        self.start_index = start_index
        self.goal_index = goal_index
        self.map_matrix = map_matrix.map_abstract_matrix
        self.matrix = deepcopy(self.map_matrix)
        self.matrix[self.start_index[0]][self.start_index[1]
                                         ] = JPath_Node(self.start_index, self.goal_index)
        self.__tree_list = []
        self.__node_list = []
        self.__end_node = None
        self.scan_matrix()

    @property
    def tree_list(self) -> list[JPath_Node]:
        return self.__tree_list

    # 先找节点, 排序子节点, 永远从排序中最小且未被扫描的节点开始寻找子节点, 当节点的坐标为终点时, 停止循环
    def scan_matrix(self) -> None:
        parent_node = JPath_Node(self.start_index, self.goal_index)
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
            read_node: JPath_Node = self.__node_list.pop(0)
            if read_node.current_index[0] == self.goal_index[0] and read_node.current_index[1] == self.goal_index[1]:
                self.__end_node = read_node
                break
            self.find_child_node(read_node)
        self.find_parent_node(self.__end_node)

    # 寻找子节点, 有两种方法, 一种是周围4格扩展, 一种是周围8格扩展
    def find_child_node(self, parent_node: JPath_Node) -> None:
        p_x, p_y = parent_node.current_index
        temp_child_node_list = []
        # 八格搜寻
        # extension_list = [[p_x-1, p_y-1], [p_x, p_y-1], [p_x+1, p_y-1], [p_x-1, p_y], [p_x+1, p_y], [p_x-1, p_y+1], [p_x, p_y+1], [p_x+1, p_y+1]]
        # 四格搜寻
        extension_list = [[p_x-1, p_y], [p_x+1, p_y], [p_x, p_y-1], [p_x, p_y+1]]
        for i in extension_list:
            new_x = i[0]
            new_y = i[1]
            if 0 <= new_x < len(self.matrix) and 0 <= new_y < len(self.matrix[0]):
                new_element = self.matrix[new_x][new_y]
            else:
                continue
            if new_element == 1 and not isinstance(new_element, JPath_Node):
                new_element = JPath_Node(
                    [new_x, new_y], self.goal_index, parent_node)
                self.matrix[new_x][new_y] = new_element
                temp_child_node_list.append(new_element)

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

    # 寻找父节点, 进行递归
    def find_parent_node(self, child_node: JPath_Node) -> None:
        if not self.__tree_list:
            self.__tree_list.append(child_node)
        if child_node.parent_node:
            self.__tree_list.insert(0, child_node.parent_node)
            self.find_parent_node(child_node.parent_node)
