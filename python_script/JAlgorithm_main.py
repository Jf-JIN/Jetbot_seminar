'''
这个文件主要是用于写路径算法的，包括计算路径及规划，建立地图，向中央处理单元发送指令建议

任务1: Labyrinthlösung 已知地图寻路径
任务2: Erkundung und Mapping 构建地图
任务3: Search and Rescue 寻找红色方块, 并原路返回

'''

from JAlgorithm_JPath_Finding import *
import matplotlib.pyplot as plt


class JAlgorithm():
    def __init__(self) -> None:
        pass

    def aufgabe1(self, yaml_data_path: str, start_index: list = None, end_index: list = None):
        with open(yaml_data_path, 'r') as file:
            loaded_data = yaml.safe_load(file)
        map_manager = JMap_Grid_Matrix_From_Yaml(loaded_data)
        for i in map_manager.map_abstract_matrix:
            print(i)
        path_finding = JPath_Finding_A_Star()
        path_finding.get_path_tree(start_index, end_index, map_manager)

    def get_angle(self, vector):
        return math.degrees(math.atan2(vector[1], vector[0]))


yaml_data_path = r'E:\10_Programm\0002_Python\0902_Jetbot_seminar\pyscript\test2.yaml'
with open(yaml_data_path, 'r') as file:
    loaded_data = yaml.safe_load(file)
a = JAlgorithm()
pf = JPath_Finding_A_Star()
zwi = JMap_Grid_Matrix_From_Yaml(loaded_data)
a.aufgabe1(r'E:\10_Programm\0002_Python\0902_Jetbot_seminar\pyscript\test2.yaml', [1, 1], [3, 5])

list_t = pf.get_node_tree([1, 1], [3, 5], zwi)

zw = deepcopy(zwi.map_abstract_matrix)
# print(zw)
for x_index, x_item in enumerate(zw):
    for y_index, y_item in enumerate(x_item):
        if y_item == 1:
            zw[x_index][y_index] = (255, 255, 255)
        elif y_item == 0:
            zw[x_index][y_index] = (0, 0, 0)
for i in list_t:
    i: JPath_Node
    zw[i.current_index[0]][i.current_index[1]] = (0, 220, 0)
    # print(i)
zw[list_t[0].current_index[0]][list_t[0].current_index[1]] = (255, 0, 0)
zw[list_t[-1].current_index[0]][list_t[-1].current_index[1]] = (0, 0, 255)
plt.imshow(zw)
plt.show()
