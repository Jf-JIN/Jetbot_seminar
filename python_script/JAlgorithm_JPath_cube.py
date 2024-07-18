from JMotor_Drive_main import *
from JData_Collect_main import *
from JAlgorithm_JPath_Node import *
from PyQt5.QtCore import QThread

from JAlgorithm_JPath_Node import JMap_Path_Node

log_info = logger_dict_algo['info']
log_error = logger_dict_algo['error']


class JPath_Search_Cube(QThread):
    def __init__(self, data_collector, server_console_send_all) -> None:
        super().__init__()
        self.data_collector: JData_Collection = data_collector
        self.server_consolesend_all = server_console_send_all
        self.flag_thread_running = True
        self.flag_running = True
        self.motor = JMotor(self, self.data_collector, self.server_consolesend_all)
        self.motor.signal_finished.connect(self.change_thread_flag)
        self.update_location()
        self.visiting_list = []
        self.start_node = None
        self.id_list = []
        self.cube_location: JLocation = None
        log_info('开始')
        # yaml_path = '/opt/ros/noetic/share/apriltag_ros/config/tags.yaml'
        # with open(yaml_path, 'r', encoding='utf-8') as yaml_file:
        #     yaml_data = yaml.safe_load(yaml_file)
        # self.wall_list = yaml_data['tag_bundles'][0]['layout']
        # self.map_manager = JMap_Grid_Matrix_From_Yaml(yaml_data)

    def update_location(self):
        self.location = self.data_collector.update_jlocation_all()

    def change_thread_flag(self, flag):
        self.flag_thread_running = flag

    def search_rotation(self, time_diff, degrees):
        self.flag_thread_running = True
        self.motor.run_circle(time_diff, degrees)
        self.wait_for_thread()

    def turn_left(self):
        log_info('进行左转')
        self.flag_thread_running = True
        self.motor.run_circle(1.3, 85)
        self.wait_for_thread()
        log_info('左转结束')

    def turn_right(self):
        self.flag_thread_running = True
        self.motor.run_circle(1.3, -85)
        self.wait_for_thread()

    def straight_front_correction(self, start_id, start_distance):
        log_info('进行前行')
        self.flag_thread_running = True
        start_distance = WALL_WIDTH_HALF-JB_DISTANCE_FROM_CAMERA_TO_CENTER
        log_info(start_distance)
        self.motor.run_line_slow(start_id, start_distance)
        self.wait_for_thread()
        log_info('前行结束')

    def straight_back_correction(self, start_id, end_distance):
        log_info('进行后退')
        self.flag_thread_running = True
        end_distance = WALL_WIDTH_HALF-JB_DISTANCE_FROM_CAMERA_TO_CENTER
        log_info(end_distance)
        self.motor.run_return_slow(start_id, end_distance)
        self.wait_for_thread()
        log_info('后退结束')

    def straight_front(self, start_id, start_distance):
        log_info('进行前行')
        self.flag_thread_running = True
        start_distance = start_distance // WALL_WIDTH * WALL_WIDTH - WALL_WIDTH_HALF
        log_info(start_distance)
        self.motor.run_line_slow(start_id, start_distance)
        self.wait_for_thread()
        log_info('前行结束')

    def straight_back(self, start_id, end_distance):
        log_info('进行后退')
        self.flag_thread_running = True
        end_distance = end_distance // WALL_WIDTH * WALL_WIDTH + WALL_WIDTH_HALF
        log_info(end_distance)
        self.motor.run_return_slow(start_id, end_distance)
        self.wait_for_thread()
        log_info('后退结束')

    def back_fix_time(self, time_diff):
        log_info('后退固定时间')
        self.flag_thread_running = True
        self.motor.run_return_fix_time(time_diff)
        self.wait_for_thread()
        log_info('固定时间后退结束')

    def front_fix_time(self, time_diff):
        log_info('前进固定时间')
        self.flag_thread_running = True
        self.motor.run_line_slow_fix_time(time_diff)
        self.wait_for_thread()
        log_info('前进固定时间结束')

    def return_to_node(self, node: JMap_Path_Node):
        # next_node: JMap_Path_Node = self.visiting_list.pop(-1)
        log_info('已进入退回操作线程')
        parent_node: JMap_Path_Node = node.parent_node
        next_node: JMap_Path_Node = self.visiting_list[-1]
        while parent_node:  # and parent_node.start_id != self.location.front.front.id:
            log_info(f'退回 id:{node.start_id} 目标距离: {node.start_distance} 当前距离: {self.location.front.front.distance.z()} ')
            # while parent_node.start_id == self.location.front.front.id:
            self.update_location()
            if parent_node.start_id == self.location.front.front.id and node.start_distance//WALL_WIDTH == self.location.front.front.distance.z()//WALL_WIDTH:
                break
            else:
                self.straight_back(node.start_id, node.start_distance)
            self.update_location()
            self.correction()
            action = node.from_parent_action
            if action == 'left':
                self.turn_right()
            elif action == 'right':
                self.turn_left()
            else:
                pass
            # self.correction()
            # self.back_fix_time(0.5)
            self.correction()
            node = node.parent_node
            parent_node = node.parent_node
            if node.start_id == next_node.parent_node.start_id and node.start_distance//WALL_WIDTH == next_node.parent_node.start_distance//WALL_WIDTH:
                break

    def return_to_original_node(self, node: JMap_Path_Node):
        # next_node: JMap_Path_Node = self.visiting_list.pop(-1)
        log_info('已进入退回起点操作线程')
        parent_node: JMap_Path_Node = node.parent_node
        while parent_node:  # and parent_node.start_id != self.location.front.front.id:
            log_info(f'退回起点 退回 id:{node.start_id} 目标距离: {node.start_distance} 当前距离: {self.location.front.front.distance.z()} ')
            # while parent_node.start_id == self.location.front.front.id:
            self.update_location()
            if parent_node.start_id == self.location.front.front.id and node.start_distance//WALL_WIDTH == self.location.front.front.distance.z()//WALL_WIDTH:
                break
            else:
                self.straight_back(node.start_id, node.start_distance)
            self.update_location()
            self.correction()
            action = node.from_parent_action
            if action == 'left':
                self.turn_right()
            elif action == 'right':
                self.turn_left()
            else:
                pass
            # self.correction()
            # self.back_fix_time(0.5)
            self.correction()
            node = node.parent_node
            parent_node = node.parent_node
            if not parent_node:
                break

    def stop_motor(self):
        self.motor.stop_motor()

    def wait_for_thread(self):
        while self.flag_thread_running:
            time.sleep(0.5)
        time.sleep(0.5)

    def correction(self):
        # 已找到码，但非正对
        if abs(self.location.front.front.orientation.y()) > 2:
            while True:
                log_info(f'已找到码，正在角度正对 {self.location.front.front.id}')
                angle_diff = self.location.front.front.orientation.y()
                turn_time = 0.2 * abs(angle_diff) / 10
                if turn_time < 0.15:
                    turn_time = 0.15
                log_info(f'angle_diff turn_time {angle_diff} {turn_time}')
                self.search_rotation(time_diff=turn_time, degrees=-angle_diff)
                self.update_location()
                if abs(self.location.front.front.orientation.y()) < 2:
                    break
            distance = self.location.front.front.distance.z() - 0.07
            # if abs(distance) > 0.02 and self.location.front.front.distance.z() < 0.2:
            #     log_info(f'已找到码，正在距离正对 {self.location.front.front.id} {distance}')
            #     id = self.location.front.front.id
            #     if distance < 0:
            #         # time_diff = distance // 0.05
            #         self.straight_back(id, WALL_WIDTH_HALF-JB_DISTANCE_FROM_CAMERA_TO_CENTER)
            #     else:
            #         # time_diff = distance // 0.05
            #         self.straight_front(id, WALL_WIDTH_HALF-JB_DISTANCE_FROM_CAMERA_TO_CENTER)
            # log_info(f'正对 {self.location.front.front.id} 结束')
        self.update_location()

    def init_node(self):
        log_info('开始')
        self.update_location()
        # 未找到码
        if self.location.front.front.orientation.y() is None:
            log_info('未找到码，正在搜索')
            num = 1
            flag_change_orientation = True
            while not self.location.front.front.orientation.y():
                if flag_change_orientation:
                    self.search_rotation(0.2, 10)
                elif num < 0 and num <= -10:
                    self.search_rotation(0.2, -10)
                num += 1
                if num == 10:
                    flag_change_orientation = not flag_change_orientation
            self.correction()

        # 已找到码，且正对，则开始记录节点
        while True:
            if (self.location.front.front.orientation.y()) < 2:
                log_info('正在建立初节点')
                original_node = JMap_Path_Node()
                temp_list = []
                # 预扫描，判断哪个方位有板子，将该方位置为后方
                self.update_location()
                None_num = 0
                for _ in range(4):
                    # 判断是否在格子内
                    self.update_location()
                    if self.location.front.front.distance.z() < 0.2:
                        temp_list.append(self.location.front.front.id)
                        log_info(f'init_node temp_list {temp_list}')
                    else:
                        temp_list.append(None)
                        log_info(f'init_node temp_list {temp_list}')
                        None_num += 1
                    self.turn_left()
                    self.correction()
                log_info(f'init_node temp_list {temp_list}')
                # 检查哪里有板子，转向背对板子方向
                # 分三种情况，四个方向中有1-3块板子
                # 有一块板子或两块板子，则以第一块检测到的墙作为背板
                if None_num == 3 or None_num == 2:
                    back_num = 0
                    for index, item in enumerate(temp_list):
                        if item:
                            back_num = index
                            break
                    front_num = (back_num + 2) % len(temp_list)
                    log_info('2或1墙')
                elif None_num == 1:
                    for index, item in enumerate(temp_list):
                        if not item:
                            front_num = index
                            break
                    back_num = (front_num + 2) % len(temp_list)
                    log_info('3墙')
                else:
                    log_info('四周无墙')
                    self.update_location()
                    self.straight_front(self.location.front.front.id, self.location.front.front.distance.z())
                    self.correction()
                    return None
                if front_num > 1:
                    front_num -= len(temp_list)
                log_info(f'fn {front_num} bn {back_num}')
                original_node.front = temp_list[front_num]
                original_node.left_list.append(temp_list[(back_num + 3) % len(temp_list)])
                original_node.right_list.append(temp_list[(back_num + 1) % len(temp_list)])
                original_node.back = temp_list[back_num]
                log_info(f'init_node temp_list 000 {original_node.front} {original_node.left_list} {original_node.right_list} {original_node.back}')
                log_info(f'{back_num} {front_num}')
                for i in range(abs(front_num)):
                    log_info(i)
                    if front_num < 0:
                        self.turn_right()
                    elif front_num > 0:
                        self.turn_left()
                    self.correction()
                # self.update_location()
                original_node.start_distance = self.location.front.front.distance.z()
                original_node.start_id = self.location.front.front.id
                self.create_child_node(original_node)
                log_info(
                    f'初阶段建立结束 {original_node.start_id} {original_node.start_distance} {original_node.front} {original_node.left_list} {original_node.right_list} {original_node.back}')
                return original_node
            else:
                self.correction()

    def create_child_node(self, parent_node: JMap_Path_Node):
        # self.update_location()
        node_l = None
        node_r = None
        node_f = None
        log_info(f'{parent_node.front} {parent_node.left_list} {parent_node.right_list}')
        log_info(
            f'{parent_node.front} {parent_node.left_list} {parent_node.right_list}'
        )
        if parent_node.left_list[-1] is None:
            node_l = JMap_Path_Node(parent_node)
            node_l.set_parent_action('left')
            log_info(f'[action] {node_l.from_parent_action}')
        if parent_node.right_list[-1] is None:
            node_r = JMap_Path_Node(parent_node)
            node_r.set_parent_action('right')
            log_info(f'[action] {node_r.from_parent_action}')
        if parent_node.front is None:
            node_f = JMap_Path_Node(parent_node)
            node_f.set_parent_action('front')
            log_info(f'[action] {node_f.from_parent_action}')
        for item in [node_l, node_f, node_r]:
            if item:
                self.visiting_list.append(item)
                parent_node.children_list.append(item)

    def extent_node(self, node: JMap_Path_Node):
        self.update_location()
        # if self.location.front.front.id != node.parent_node.start_id and abs(self.location.front.front.distance.z() - node.parent_node.end_distance) > 0.01:

        # 检查起点是否为父节点最后的位置
        log_info(f'{self.location.front.front.id} != {node.parent_node.start_id} {node.parent_node.front} {node.parent_node.left_list} {node.parent_node.right_list}')
        while self.location.front.front.id != node.parent_node.start_id:
            self.motor.run_circle(1, 10)
            self.correction()
        # 转至当前节点朝向
        action = node.from_parent_action
        log_info(f'[action] {action}')
        if action == 'left':
            log_info('子节点 左')
            self.turn_left()
            self.correction()
            node.start_id = self.location.front.front.id
            node.start_distance = self.location.front.front.distance.z()
        elif action == 'right':
            log_info('子节点 右')
            self.turn_right()
            self.correction()
            node.start_id = self.location.front.front.id
            node.start_distance = self.location.front.front.distance.z()
        elif action == 'front':
            self.correction()
            log_info('子节点 前')
            node.start_id = self.location.front.front.id
            node.start_distance = self.location.front.front.distance.z()
        # 赋值当前节点定位
        branch_num = 0
        flag_to_end = False
        self.flag_found = False
        while self.flag_running:
            self.update_location()
            node.front = self.location.front.front.id
            cube = self.location.cube
            if cube:
                log_info('找到骰子')
                self.flag_found = True
                self.cube_location = copy.deepcopy(self.location)
                break
            log_info(f'node.front {node.front}')
            if isinstance(node.front, list) and (node.front[0] in self.id_list or node.front[1] in self.id_list):
                log_info('前方重复节点')
                if self.location.front.front.distance.z() > 0.25 and self.location.front.front.distance.z() < 0.5:
                    log_info('前方重复节点，忽略访问')
                    flag_to_end = True
                    break
            if len(self.location.left_list) > 0:
                node.left_list.append(self.location.left_list[0].id)
                if node.left_list[-1][0] in self.id_list or node.left_list[-1][1] in self.id_list:
                    log_info('左侧重复节点，忽略访问')
                    flag_to_end = True
                    break
            else:
                if self.location.front.front.distance.z() // 0.25 > 1:
                    node.front = None
                node.left_list.append(None)
                branch_num += 1
                log_info('left leer')
            if len(self.location.right_list) > 0:
                node.right_list.append(self.location.right_list[0].id)
                if node.right_list[-1][0] in self.id_list or node.right_list[-1][1] in self.id_list:
                    log_info('右侧重复节点，忽略访问')
                    flag_to_end = True
                    break
            else:
                if self.location.front.front.distance.z() // 0.25 > 1:
                    node.front = None
                node.right_list.append(None)
                branch_num += 1
                log_info('right leer')
            # 判断是否有支路，或者转弯, 终止循环
            if branch_num:
                log_info('有支路')
                break
            # 判断是否前方为最后一块板子, 且左右没有支路, 终止循环
            if self.location.front.front.distance.z() // 0.25 == 1 and branch_num == 0:
                flag_to_end = True
                log_info('前方为最后一块板子, 且左右没有支路, 终止循环')
                break
            log_info(f'id distance: {node.start_id} {node.start_distance}')
            self.straight_front(node.start_id, node.start_distance)
            self.correction()

        # 前方有路，但有支路
        if branch_num != 0:
            log_info(f'前方有路，但有支路 {node.start_id} {node.start_distance}')
            self.create_child_node(node)
            self.straight_front(node.start_id, self.location.front.front.distance.z())
        elif self.flag_found:
            log_info('找到骰子返回起点')
            self.return_to_original_node(node)
            log_info('完成返回')
        # 前方无路, 且无子节点
        elif flag_to_end:
            log_info('前方无路, 且无子节点')
            self.return_to_node(node)
            log_info('完成退回')
        # 前方有路，且无支路
        else:
            log_info('继续前行')
            # self.straight_front(node.start_id, node.start_distance)
            self.straight_front(node.start_id, node.start_distance)
            self.correction()

        temp_visiting = [[i.start_id, i.start_distance, i.from_parent_action] for i in self.visiting_list]
        log_info(f'[visiting_list] {temp_visiting} ')

    def collect_id(self, start_node: JMap_Path_Node):
        for i in start_node.children_list:
            i: JMap_Path_Node
            if len(i.children_list):
                self.id_list.append(i.get_whole_id_list())
                self.collect_id(i)
            else:
                self.id_list.append(i.get_whole_id_list())
        self.collect_id_in_single_list()

    def collect_id_in_single_list(self):
        temp_list = []
        for i in self.id_list:
            if isinstance(i, list):
                for j in i:
                    if j in temp_list:
                        continue
                    temp_list.append(j)
            else:
                temp_list.append(i)
        self.id_list = temp_list

    def start_tree(self):
        log_info(f'开始 {self.start_node}')
        # self.start_node: JMap_Path_Node = self.init_node()
        self.start_node = None
        while not self.start_node:
            log_info('开始初始节点')
            self.start_node: JMap_Path_Node = self.init_node()
        node = self.start_node
        self.update_location()
        try:
            while self.flag_running:
                log_info('start_tree while 开始')
                temp_visiting = [[i.start_id, i.start_distance, i.from_parent_action] for i in self.visiting_list]
                log_info(f'[visiting_list] {temp_visiting} ')
                node: JMap_Path_Node = self.visiting_list.pop(-1)
                log_info(f'node: {node} {node.start_id}')
                self.extent_node(node)
                if self.flag_found:
                    break
                self.get_id_list()
        except Exception as e:
            e = traceback.format_exc()
            log_error(e)
            self.collect_id(self.start_node)
        finally:
            self.get_id_list()
            return self.id_list

    def check_id_list(self):
        for index, item in enumerate(self.id_list):
            if not item:
                del self.id_list[index]
        log_info(f'id_list {self.id_list}')

    def get_id_list(self):
        self.collect_id(self.start_node)
        self.check_id_list()
        text = {'a2_id_list': self.id_list}
        self.server_consolesend_all(text)

    # def build_matrix(self):
    #     take_wall_list = []
    #     for index, wall_item in enumerate(self.wall_list):
    #         for id_item in self.id_list:
    #             if wall_item['id'] == id_item:
    #                 take_wall_list.append(wall_item)
    #     yaml_dict = {'tag_bundles': [{'layout': None}]}
    #     yaml_dict['tag_bundles'][0]['layout'] = take_wall_list
    #     self.map_manager.JMap_generator(yaml_dict)
    #     self.abstract_matrix = self.map_manager.map_abstract_matrix
    #     self.obj_matrix = self.map_manager.map_obj_matrix

    def get_map(self):
        try:
            log_info('开始')
            self.start_tree()
            text = {'a2_id_list': self.id_list}
            self.server_consolesend_all(text)
            yaml_path = '/opt/ros/noetic/share/apriltag_ros/config/tags.yaml'
            with open(yaml_path, 'r', encoding='utf-8') as yaml_file:
                yaml_data = yaml.safe_load(yaml_file)
            wall_list = yaml_data['tag_bundles'][0]['layout']
            wall_orientation = None
            for i in wall_list:
                # i['qx'] == 0.5: 横向  qy = -0.5 朝上 + 0.5 朝下
                # i['qx'] != 0.5: 横向  qy = 0 朝左 qx = 0 朝右
                wall_dict_x_distance = i['x']
                wall_dict_y_distance = i['y']
                if i['qx'] == 0.5:
                    if i['qy'] == -0.5:  # 朝上
                        wall_orientation = 'H_O'
                    else:  # 朝下
                        wall_orientation = 'H_U'
                else:
                    if i['qy'] == 0:  # 朝左
                        wall_orientation = 'V_L'
                    else:  # 朝右
                        wall_orientation = 'V_R'
                if i['id'] in self.cube_location.front.front.id:
                    wall_x_distance = self.cube_location.front.front.distance.x()
                    wall_z_distance = self.cube_location.front.front.distance.z()
                    cube_x_distance = self.cube_location.cube.distance.x()
                    cube_z_distance = self.cube_location.cube.distance.z()
                    cube_y_orientation = self.cube_location.cube.orientation.y()
                    log_info(
                        f'wall_x_distance, wall_z_distance {i["id"]}, {wall_x_distance}, {wall_z_distance}\ncube_x_distance, cube_z_distance {cube_x_distance}, {cube_z_distance}\ncube_y_orientation {cube_y_orientation}\n wall_dict_x_distance, wall_dict_y_distance {wall_dict_x_distance} {wall_dict_y_distance}')
                    if wall_orientation == 'H_U':  # 车在下方
                        tag_pos_y = wall_dict_y_distance + wall_x_distance + cube_x_distance
                        tag_pos_x = wall_dict_x_distance + (wall_z_distance - cube_z_distance)
                        cube_center_y = tag_pos_y - CUBE_TO_CENTER * math.sin(math.radians(cube_y_orientation))
                        cube_center_x = tag_pos_x - CUBE_TO_CENTER * math.cos(math.radians(cube_y_orientation))

                    elif wall_orientation == 'H_O':  # 车在上方
                        tag_pos_y = wall_dict_y_distance + wall_x_distance + cube_x_distance
                        tag_pos_x = wall_dict_x_distance - (wall_z_distance - cube_z_distance)
                        cube_center_y = tag_pos_y + CUBE_TO_CENTER * math.sin(math.radians(cube_y_orientation))
                        cube_center_x = tag_pos_x + CUBE_TO_CENTER * math.cos(math.radians(cube_y_orientation))

                    elif wall_orientation == 'V_R':  # 车在右侧
                        tag_pos_x = wall_dict_x_distance + wall_x_distance + cube_x_distance
                        tag_pos_y = wall_dict_y_distance + (wall_z_distance - cube_z_distance)
                        cube_center_x = tag_pos_x + CUBE_TO_CENTER * math.sin(math.radians(cube_y_orientation))
                        cube_center_y = tag_pos_y - CUBE_TO_CENTER * math.cos(math.radians(cube_y_orientation))

                    elif wall_orientation == 'V_L':  # 车在左侧
                        tag_pos_x = wall_dict_x_distance + wall_x_distance + cube_x_distance
                        tag_pos_y = wall_dict_y_distance - (wall_z_distance - cube_z_distance)
                        cube_center_x = tag_pos_x - CUBE_TO_CENTER * math.sin(math.radians(cube_y_orientation))
                        cube_center_y = tag_pos_y + CUBE_TO_CENTER * math.cos(math.radians(cube_y_orientation))
                    log_info(f'wall_orientation {wall_orientation}')
                    log_info(f'骰子位置: {cube_center_x}, {cube_center_y}')
                    log_info(f'骰子Apriltag位置: {tag_pos_x}, {tag_pos_y}')
                    text = {'cube_pos': [cube_center_x, cube_center_y]}
                    text2 = {'cube_tag_pos': [tag_pos_x, tag_pos_y]}
                    self.server_consolesend_all(text)
                    self.server_consolesend_all(text2)
                    break

        except Exception as e:
            e = traceback.format_exc()
            log_error(e)

    def run(self):
        log_info('开始')
        self.get_map()
