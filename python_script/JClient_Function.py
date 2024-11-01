
from PyQt5.QtWidgets import QMessageBox, QLabel, QLineEdit, QGroupBox, QGridLayout


from JClientUI import *
from JClient_Server_Console import *
from JClient_Server_Video import *
from JLocation import *
from JAlgorithm_JMap_Matrix_Manager import *


import functools

# PYTHON_FILE_PATH = os.path.dirname(__file__)
PYTHON_FILE_PATH = os.getcwd()


log_info = logger_dict_client['info']
log_error = logger_dict_client['error']


logger_dict_a2_map = JLog('a2_map', 'a2_map')
log_map = logger_dict_a2_map['info']


class JClient_Function(JClient_UI):
    signal_data_video_send = pyqtSignal(list)
    signal_data_console_send = pyqtSignal(dict)
    signal_data_close_client_send = pyqtSignal(dict)
    signal_data_close_server_send = pyqtSignal(dict)

    def __init__(self) -> None:
        super().__init__()
        self.parameter_init()
        self.signal_connections()

    def parameter_init(self) -> None:
        super().parameter_init()
        self.current_pos_index = None
        self.list_connection_map_pb = []
        self.last_passed_pb = None
        self.flag_save_last_data = False
        self.a2_map_data = []
        self.plot_list = {
            'front': [[0, 0, 0, 0, 0, 0] for _ in range(PLOT_LIST_LENGTH)],
            'left': [[0, 0, 0, 0, 0, 0] for _ in range(PLOT_LIST_LENGTH)],
            'right': [[0, 0, 0, 0, 0, 0] for _ in range(PLOT_LIST_LENGTH)],
            'imu_ori': [[0, 0, 0] for _ in range(PLOT_LIST_LENGTH)],
            'imu_acc': [[0, 0, 0] for _ in range(PLOT_LIST_LENGTH)],
            'imu_ang_spd': [[0, 0, 0] for _ in range(PLOT_LIST_LENGTH)],
            'imu_mag': [[0, 0, 0] for _ in range(PLOT_LIST_LENGTH)],
        }

    def signal_connections(self) -> None:
        super().signal_connections()
        self.pb_connect.clicked.connect(self.connection_to_server)
        self.pb_reconnect_console.clicked.connect(self.console_connection_to_server)
        self.pb_reconnect_video.clicked.connect(self.video_connection_to_server)
        self.pb_close_server.clicked.connect(self.close_server)
        # self.pb_map_clear.clicked.connect(self.clear_a2_map)
        self.pb_camera_listener.clicked.connect(lambda: self.signal_data_console_send.emit({'camera_listener': 'on'}))
        self.pb_force_accept.clicked.connect(lambda: self.signal_data_console_send.emit({'force_accept': True}))
        self.pb_stop.clicked.connect(lambda: self.signal_data_console_send.emit({'motor_action': {'mode': 'stop'}}))
        self.console_win.pb_jlocation_listen_init.clicked.connect(lambda: self.signal_data_console_send.emit({'jlocation_listener': 'on'}))
        self.console_win.pb_ros_node_init.clicked.connect(lambda: self.signal_data_console_send.emit({'ros_node_init': 'on'}))
        self.console_win.cb_save_last_data.stateChanged.connect(self.change_save_last_location_data)
        self.dsb_distance_kd.valueChanged.connect(self.send_motor_parameter)
        self.dsb_distance_ki.valueChanged.connect(self.send_motor_parameter)
        self.dsb_distance_kp.valueChanged.connect(self.send_motor_parameter)
        self.dsb_motor_left_calib.valueChanged.connect(self.send_motor_parameter)
        self.dsb_motor_right_calib.valueChanged.connect(self.send_motor_parameter)

        self.tabWidget.currentChanged.connect(self.clear_launch_function)
        self.pb_launch.clicked.connect(self.launch_function)
    # ==================================== JConsole 信号连接 ====================================
        self.pb_le_init('roscore')
        self.pb_le_init('ros_camera')
        self.pb_le_init('ros_rectify')
        self.pb_le_init('ros_apriltag_detection')
        self.pb_le_init('ros_imu')
        self.pb_le_init('ros_imu_calib')
        self.pb_le_init('ros_motor')
        self.pb_le_init('ros_algorithm')
        # self.pb_le_init('')
        # self.pb_le_init('')

    def pb_le_init(self, sign: str):
        pb_start_name = f'pb_{sign}_start'
        pb_stop_name = f'pb_{sign}_stop'
        le_input_name = f'le_{sign}'
        pb_start = getattr(self.console_win, pb_start_name)
        pb_stop = getattr(self.console_win, pb_stop_name)
        le_input = getattr(self.console_win, le_input_name)
        pb_start.clicked.connect(lambda: self.send_console_to_server(sign))
        pb_stop.clicked.connect(lambda: self.emergency_stop(sign))
        le_input.returnPressed.connect(lambda: self.send_console_to_server(sign))

    def emergency_stop(self, key):
        self.signal_data_console_send.emit({key: 'Emergency_Stop'})

    # 用于执行任务
    def launch_function(self):
        try:
            # self.signal_data_console_send.emit({'motor_action': {'mode': 'circle', 'direction': 'left', 'time_diff': 3, 'radius': 0, 'degrees': 80, 'id': None, 'distance': None}})
            if self.tabWidget.currentIndex() == 0:
                if not hasattr(self, 'list_pb_map') or not self.list_pb_map:
                    return
                self.map_pb_function_connection()
                # self.display_current_pos(0)
            elif self.tabWidget.currentIndex() == 1:
                text = {'a2_map_build': 'on'}
                self.signal_data_console_send.emit(text)
            elif self.tabWidget.currentIndex() == 2:
                text = {'a3_find': 'on'}
                self.signal_data_console_send.emit(text)
        except Exception as e:
            e = traceback.format_exc()
            text = f'[客户端][launch_function][!错误!]: \n{e}'
            log_error(text)
            self.append_TB_text(text, self.tb_console)

    # 用于清除执行任务后的关联
    def clear_launch_function(self):
        try:
            if self.list_connection_map_pb:
                for index, item in enumerate(self.list_connection_map_pb):
                    self.list_pb_map[index].clicked.disconnect(item)
                self.list_connection_map_pb = []
        except Exception as e:
            e = traceback.format_exc()
            text = f'[客户端][clear_launch_function][!错误!]: \n{e}'
            log_error(text)
            self.append_TB_text(text, self.tb_console)

    # 对按钮和功能进行连接，主要是地图按钮和功能
    def map_pb_function_connection(self):
        try:
            if self.current_pos_index:
                for i in self.list_pb_map:
                    i: QPushButton
                    connection = i.clicked.connect(functools.partial(self.map_pb_function, i))
                    self.list_connection_map_pb.append(connection)
        except Exception as e:
            e = traceback.format_exc()
            text = f'[客户端][map_pb_function_connection][!错误!]: \n{e}'
            log_error(text)
            self.append_TB_text(text, self.tb_console)

    # 地图按钮的功能
    def map_pb_function(self, sender):
        try:
            log_info(self.current_pos_index)
            if not self.current_pos_index and self.current_pos_index != [0, 0]:
                return
            pb_index = [int(sender.objectName().split('_')[-2]), int(sender.objectName().split('_')[-1])]
            self.lb_goal_index.setText(f'目标: {pb_index}')
            self.path_finding = JPath_Finding_A_Star()
            node_tree = self.path_finding.get_node_tree(self.current_pos_index, pb_index, self.map_manager)
            for i in self.list_pb_map:
                style = f'background-color: {PATH_COLOR};'
                i.setStyleSheet(style)
            for node_index, node_item in enumerate(node_tree):
                node_item: JPath_Node
                path_index = node_item.current_index
                style = f'background-color: {PLAN_PATH_COLOR};'
                if node_index == len(node_tree) - 1:
                    style = f'background-color: {GOAL_POINT_COLOR};'
                for i in self.list_pb_map:
                    i: QPushButton
                    if i.objectName() == f'pb_map_{path_index[0]}_{path_index[1]}':
                        i.setStyleSheet(style)
            text = {'a1_path_dict': [self.path_finding.get_path_tree(self.current_pos_index, pb_index, self.map_manager), [self.dsb_distance_kp.value(),
                                                                                                                           self.dsb_distance_ki.value(), self.dsb_distance_kd.value(), '', '', ''], [self.dsb_motor_left_calib.value(), self.dsb_motor_right_calib.value()]]}
            log_info('here')
            self.signal_data_console_send.emit(text)
        except Exception as e:
            e = traceback.format_exc()
            text = f'[客户端][map_pb_function][!错误!]: \n{e}'
            log_error(text)
            self.append_TB_text(text, self.tb_console)

    def send_motor_parameter(self):
        motor_parameter_text = {'motor_parameter':
                                {'pid_distance': [self.dsb_distance_kp.value(), self.dsb_distance_ki.value(), self.dsb_distance_kd.value()],
                                 'motor_calib': [self.dsb_motor_left_calib.value(), self.dsb_motor_right_calib.value()]
                                 }}
        self.signal_data_console_send.emit(motor_parameter_text)
        # 获取ip地址

    def get_ip(self) -> str:
        return self.le_jetbot_ip.text().strip()

    # 连接服务端

    def connection_to_server(self) -> None:
        self.video_connection_to_server()
        self.console_connection_to_server()

    # ==================================== Client_Video_QThread ====================================

    def video_connection_to_server(self) -> None:
        try:
            if self.flag_connection_to_server:
                return
            ip = self.get_ip()
            if not ip:
                QMessageBox.warning(None, '提示', '请填写Jetbot的ip地址')
                return
            self.client_video = Client_Video_QThread(ip)
            self.client_video.signal_connected_port.connect(self.load_jetbot_video_port)       # 端口信号
            self.client_video.signal_connected_flag.connect(functools.partial(self.clear_port_display, self.lb_video_port))     # 断开连接信号
            # self.client_video.signal_error_output.connect(functools.partial(self.append_TB_text, self.tb_console))
            self.client_video.signal_data_video_recv.connect(self.video_update)     # 接收视频信号
            self.client_video.signal_server_close.connect(self.send_close_signal)   # 向服务器回复断开连接的命令

            self.signal_data_close_client_send.connect(self.client_video.send)      # 发送客户端关闭信号
            self.signal_data_close_server_send.connect(self.client_video.send)      # 发送服务器关闭信号
            self.client_video.start()
        except Exception as e:
            e = traceback.format_exc()
            text = f'[客户端][video_connection_to_server][!错误!]: \n{e}'
            log_error(text)
            self.append_TB_text(text, self.tb_console)

    # ==================================== Client_Console_QThread ====================================
    def console_connection_to_server(self) -> None:
        try:
            if self.flag_connection_to_server:
                return
            ip = self.get_ip()
            if not ip:
                QMessageBox.warning(None, '提示', '请填写Jetbot的ip地址')
                return
            self.client_console = Client_Console_QThread(ip)
            self.client_console.signal_connected_port.connect(self.load_jetbot_console_port)     # 端口信号
            self.client_console.signal_connected_flag.connect(functools.partial(self.clear_port_display, self.lb_console_port))     # 断开连接信号
            self.client_console.signal_data_console_recv.connect(self.signal_sort)        # 接收命令提示符返回内容
            self.client_console.signal_server_close.connect(self.send_close_signal)   # 向服务器回复断开连接的命令
            self.client_console.signal_ping.connect(self.display_ping)

            self.signal_data_console_send.connect(self.client_console.send)        # 发送信号
            self.signal_data_close_client_send.connect(self.client_console.send)   # 发送客户端关闭信号
            self.signal_data_close_server_send.connect(self.client_console.send)   # 发送服务器关闭信号
            self.client_console.start()
        except Exception as e:
            e = traceback.format_exc()
            text = f'[客户端][console_connection_to_server][!错误!]: \n{e}'
            log_error(text)
            self.append_TB_text(text, self.tb_console)

    # 加载端口

    def load_jetbot_console_port(self, port: str):
        log_info(f'load_jetbot_console_port \t{port}')
        self.lb_console_port.setText(port)
        self.change_client_server_connection_display()
        self.reconnect_display()
    # 加载端口

    def load_jetbot_video_port(self, port: str):
        log_info(f'load_jetbot_video_port \t{port}')
        self.lb_video_port.setText(port)
        self.change_client_server_connection_display()
        self.reconnect_display()

    def display_ping(self, ping_ms):
        # current_time = time.time()
        # ping_time_ms = (current_time - send_time)  # * 1000
        self.lb_ping.setText(str(int(ping_ms))+' ms  ')

    # 断开连接时，清空端口显示
    def clear_port_display(self, tag: QLabel, flag: bool) -> None:
        if not flag:
            tag.clear()
            self.change_client_server_connection_display()
            self.reconnect_display()

    # 显示更新视频
    def video_update(self, pixmap: QPixmap):
        try:
            origal_width = pixmap.width()
            origal_height_width_rate = pixmap.height() / pixmap.width()
            width_rate = self.hs_video_size.value()
            width = int(origal_width * width_rate * 1.5 / 100)
            height = int(width * origal_height_width_rate)
            scaled_pixmap = pixmap.scaled(width, height, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.lb_image.setPixmap(scaled_pixmap)
        except Exception as e:
            e = traceback.format_exc()
            text = f'[客户端][video_update][!错误!]: \n{e}'
            log_error(text)
            self.append_TB_text(text, self.tb_console)

    def ping_display(self, text):
        self.lb_ping.setText(text)
        # 从客户端关闭服务器

    def close_server(self):
        if self.flag_connection_to_server:
            text = {'server_close': 'Server will be closed'}
            self.signal_data_close_server_send.emit(text)

    # 客户端向服务端发送关闭信号
    def send_close_signal(self):
        try:
            if hasattr(self, 'client_video') and self.client_video and self.client_video.isRunning():
                close_signal = {'close': 'Client_disconnected'}
                self.signal_data_close_client_send.emit(close_signal)
                self.signal_data_close_client_send.disconnect(self.client_video.send)      # 断开信号连接，避免重复被激活
                self.client_video.stop()
                self.clear_port_display(self.lb_video_port, False)
            if hasattr(self, 'client_console') and self.client_console and self.client_console.isRunning():
                close_signal = {'close': 'Client_disconnected'}
                self.signal_data_close_client_send.emit(close_signal)
                self.client_console.stop()
                self.clear_port_display(self.lb_console_port, False)
        except Exception as e:
            e = traceback.format_exc()
            text = f'[客户端][send_close_signal][!错误!]: \n{e}'
            log_error(e)
            self.append_TB_text(text, self.tb_console)

        # **************************************** 子窗口功能 ****************************************

    def send_console_to_server(self, key: str):
        try:
            sender = self.sender()
            data = self.console_win.build_console_dict()  # 获取所有 LineEdit 控件
            log_info(f'[触发控件]: {sender}')
            line_edit_widget: QLineEdit = data[key]
            line_edit_text = line_edit_widget.text()
            if line_edit_text == '':
                line_edit_text = line_edit_widget.placeholderText()
            command_data = {key: line_edit_text}
            log_info(f'[发送命令信号]: {command_data}')
            self.signal_data_console_send.emit(command_data)
            line_edit_widget.clear()
        except Exception as e:
            e = traceback.format_exc()
            text = f'[客户端][send_console_to_server][!错误!]: \n{e}'
            log_error(e)
            self.append_TB_text(text, self.tb_console)

    def change_save_last_location_data(self):
        if self.console_win.cb_save_last_data.isChecked():
            self.flag_save_last_data = True
        else:
            self.flag_save_last_data = False

    # 数据解包

    def signal_sort(self, data):
        data: dict
        # log_info(f'\n[接收命令行包]: {data}')
        data_sort_dict = {
            'roscore': lambda: self.append_TB_text(data['roscore'], self.console_win.tb_roscore),
            'ros_camera': lambda: self.append_TB_text(data['ros_camera'], self.console_win.tb_ros_camera),
            'ros_rectify': lambda: self.append_TB_text(data['ros_rectify'], self.console_win.tb_ros_rectify),
            'ros_apriltag_detection': lambda: self.append_TB_text(data['ros_apriltag_detection'], self.console_win.tb_ros_apriltag_detection),
            'ros_imu': lambda: self.append_TB_text(data['ros_imu'], self.console_win.tb_ros_imu),
            'ros_imu_calib': lambda: self.append_TB_text(data['ros_imu_calib'], self.console_win.tb_ros_imu_calib),
            'ros_motor': lambda: self.append_TB_text(data['ros_motor'], self.console_win.tb_ros_motor),
            'ros_algorithm': lambda: self.append_TB_text(data['ros_algorithm'], self.console_win.tb_ros_algorithm),
            'current_pos_float': lambda: self.display_current_pos(data['current_pos_float']),
            'jlocation_package': lambda: self.display_jlocation(data['jlocation_package']),
            'map_generation': lambda: self.map_display_update(data['map_generation']),
            'motor_data': lambda: self.motor_data_display(data['motor_data']),
            'a2_id_list': lambda: self.a2_map_build(data['a2_id_list']),
            'send_time': lambda: self.display_ping(data['send_time']),
            'cube_pos': lambda: self.cube_pos_display(data['cube_pos']),
            'cube_tag_pos': lambda: self.cube_tag_pos_display(data['cube_tag_pos'])
        }
        for key, value in data_sort_dict.items():
            if key in data:
                value()

    def cube_tag_pos_display(self, data: list):
        cube_x = '{:.6f}'.format(data[0])
        cube_y = '{:.6f}'.format(data[1])
        self.lb_cube_tag_pos.setText(f'{cube_x}, {cube_y}')

    def cube_pos_display(self, data: list):
        cube_x = '{:.6f}'.format(data[0])
        cube_y = '{:.6f}'.format(data[1])
        self.lb_cube_pos.setText(f'{cube_x}, {cube_y}')

    def clear_a2_map(self):
        map_layout = self.frame_a2_map.layout()
        if map_layout:
            self.clearLayout(map_layout)

    def complete_wall_list(self, data):
        try:
            yaml_path = os.path.join(PYTHON_FILE_PATH, 'wall.yaml')
            take_wall_list = []
            with open(yaml_path, 'r', encoding='utf-8') as yaml_file:
                yaml_data = yaml.safe_load(yaml_file)
            wall_list = yaml_data['tag_bundles'][0]['layout']
            for index, wall_item in enumerate(wall_list):
                for id_item in data:
                    if wall_item['id'] == id_item:
                        take_wall_list.append(wall_item)
            new_yaml_path = os.path.join(PYTHON_FILE_PATH, 'Aufgabe2.yaml')
            temp_yaml = {'tag_bundles': [{'layout': None}]}
            temp_yaml['tag_bundles'][0]['layout'] = take_wall_list
            self.a2_map_data = copy.deepcopy(temp_yaml)
            self.map_manager_a2 = JMap_Grid_Matrix_From_Yaml(self.a2_map_data)
            temp_yaml['standalone_tags'] = temp_yaml['tag_bundles'][0]['layout']
            # self.a2_map_data = copy.deepcopy(temp_yaml)
            with open(new_yaml_path, 'w', encoding='utf-8') as output_file:
                yaml.dump(temp_yaml, output_file, default_flow_style=False, sort_keys=False)
        except Exception as e:
            e = traceback.format_exc()
            text = f'[客户端][complete_wall_list][!错误!]: \n{e}'
            log_error(e)
            self.append_TB_text(text, self.tb_console)

    def a2_map_build(self, data):
        try:
            log_map(f'data {data}')
            self.append_TB_text(data, self.tb_console)
            self.complete_wall_list(data)
            log_map(f'self.a2_map_data {self.a2_map_data}')
            self.map_manager_a2.JMap_generator(self.a2_map_data)
            abstract_map = self.map_manager_a2.map_abstract_matrix
            obj_map = self.map_manager_a2.map_obj_matrix
            matrix_text = ''
            for i in abstract_map:
                matrix_text += str(i) + '\n'
            log_map(f'matrix_text {matrix_text}')
            map_layout = self.frame_a2_map.layout()
            if map_layout:
                self.clearLayout(map_layout)
            else:
                map_layout = QGridLayout()
            map_layout.setVerticalSpacing(0)
            map_layout.setHorizontalSpacing(0)
            for i in range(len(abstract_map)):
                map_layout.setRowStretch(i, 1)
            for j in range(len(abstract_map[0])):
                map_layout.setColumnStretch(j, 1)
            for x_index, line_list in enumerate(abstract_map):
                line_list: list[JWall]
                for y_index, item in enumerate(line_list):
                    jwall_item: JWall = obj_map[x_index][y_index]
                    if item == 0:  # 墙
                        wall_item = QLabel('   ')
                        wall_item_name = f'lb_map_{x_index}_{y_index}'
                        wall_item.setObjectName(wall_item_name)
                        wall_item.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
                        wall_item.setCursor(Qt.CrossCursor)
                        style = f'background-color: {WALL_COLOR};'
                        wall_item.setStyleSheet(style)
                        map_layout.addWidget(wall_item, x_index, y_index)
                        if jwall_item:
                            if jwall_item.orientation == 'H':
                                wall_item.setToolTip(
                                    f'id:\t[{jwall_item.main_0.id}, {jwall_item.main_1.id}]\n\t[{jwall_item.sub_0.id}, {jwall_item.sub_1.id}]\n索引:\t[{x_index}, {y_index}]\n位置:\t[{jwall_item.middle.xStr_mm()}, {jwall_item.middle.yStr_mm()}]\n方向:\t{jwall_item.orientation}')
                            else:
                                wall_item.setToolTip(
                                    f'id:\t[{jwall_item.main_0.id}, {jwall_item.main_1.id}] [{jwall_item.sub_0.id}, {jwall_item.sub_1.id}]\n索引:\t[{x_index}, {y_index}]\n位置:\t[{jwall_item.middle.xStr_mm()}, {jwall_item.middle.yStr_mm()}]\n方向:\t{jwall_item.orientation}')
                        else:
                            style = f'background-color: {WALL_VIRTUAL_COLOR};'
                            wall_item.setStyleSheet(style)
                    elif item == 1:
                        path_item = QPushButton()
                        path_item_name = f'pb_map_{x_index}_{y_index}'
                        path_middle_x, path_middle_y = self.map_manager_a2.from_index_to_coordinate([x_index, y_index])
                        path_middle_x = '{:.{}f}'.format(path_middle_x * 1000, DIGITS_S)
                        path_middle_y = '{:.{}f}'.format(path_middle_y * 1000, DIGITS_S)
                        path_item.setObjectName(path_item_name)
                        path_item.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
                        path_item.setToolTip(f'索引:\t[{x_index}, {y_index}]\n位置:\t[{path_middle_x}, {path_middle_y}]')
                        path_item.setCursor(Qt.PointingHandCursor)
                        map_layout.addWidget(path_item, x_index, y_index)
                self.frame_a2_map.setLayout(map_layout)
                frame_map_style_sheet =\
                    'QFrame#frame_a2_map QPushButton{' + \
                    f'''background-color: {PATH_COLOR};
                        border: None;
                        border-radius: 0px;
                        min-width: 10px;
                        min-height:10px;''' + \
                    '}' + \
                    '''QFrame#frame_a2_map QPushButton:hover''' + '{' + \
                    f'background-color: {PATH_HOVER_COLOR};' + \
                    '}' + \
                    'QFrame#frame_a2_map QLabel' + '{' +\
                    '''min-width: 10px;
                        min-height:10px;''' + \
                    '}'
                self.frame_a2_map.setStyleSheet(frame_map_style_sheet)
        except Exception as e:
            e = traceback.format_exc()
            text = f'[客户端][a2_map_build][!错误!]: \n{e}'
            log_error(e)
            log_map(e)
            self.append_TB_text(text, self.tb_console)

    def check_pwm_data(self, object: QLabel, data):
        try:
            if (data or data == 0) and isinstance(data, (float, int)):
                object.setText('{:.{}f}'.format(data, DIGITS_S))
            else:
                object.setText('')
        except Exception as e:
            e = traceback.format_exc()
            text = f'[客户端][check_pwm_data][!错误!]: \n{e}'
            log_error(e)
            self.append_TB_text(text, self.tb_console)

    def motor_data_display(self, data):
        try:
            dict_lb_motor = {
                'volt_l': self.lb_motor_volt_l,
                'volt_r': self.lb_motor_volt_r,
                'pwm_left': self.lb_motor_pwm_l,
                'pwm_right': self.lb_motor_pwm_r,
                'pid_left': self.lb_motor_left_pid,
                'pid_right': self.lb_motor_right_pid,
                'distance_diff': self.lb_pid_distance_diff,
                'psi': self.lb_pid_angle_diff,
                'delta_time': self.lb_pid_time_diff,
                'integral': self.lb_pid_integral,
                'turn_rate': self.lb_turn_rate
            }
            for key, value in dict_lb_motor.items():
                if key in data:
                    self.check_pwm_data(value, data[key])
                else:
                    log_info(f'motor_data_display {key}')
            # list_lb_motor = [
            #     self.lb_motor_volt_l,
            #     self.lb_motor_volt_r,
            #     self.lb_motor_pwm_l,
            #     self.lb_motor_pwm_r,
            #     self.lb_motor_left_pid,
            #     self.lb_motor_right_pid,
            #     self.lb_pid_distance_diff,
            #     self.lb_pid_angle_diff,
            #     self.lb_pid_time_diff,
            #     self.lb_pid_integral,
            #     self.lb_turn_rate]
            # for index, item in enumerate(list_lb_motor):
            #     self.check_pwm_data(item, data[index])
            # self.check_pwm_data(self.lb_motor_volt_l, data[0])
            # self.check_pwm_data(self.lb_motor_volt_r, data[1])
            # self.check_pwm_data(self.lb_motor_pwm_l, data[2])
            # self.check_pwm_data(self.lb_motor_pwm_r, data[3])
            # self.check_pwm_data(self.lb_motor_left_pid, data[4])
            # self.check_pwm_data(self.lb_motor_right_pid, data[5])
            # self.check_pwm_data(self.lb_pid_distance_diff, data[6])
            # self.check_pwm_data(self.lb_pid_angle_diff, data[6])
            # self.check_pwm_data(self.lb_pid_time_diff, data[7])
        except Exception as e:
            e = traceback.format_exc()
            text = f'[客户端][motor_data_display][!错误!]: \n{e}'
            log_error(e)
            self.append_TB_text(text, self.tb_console)

    def map_display_update(self, data):
        # data 是一个json文件
        # yaml_data = yaml.dump(data, default_flow_style=False)
        # map_manager = JMap_Matrix_From_Yaml(yaml_data)
        # map_abstract = map_manager.map_abstract_matrix
        pass

    def display_current_pos(self, data: list):
        try:
            if hasattr(self, 'frame_map_style_sheet'):
                self.lb_current_pos.setText('')
                if not data:
                    self.frame_map.setStyleSheet(self.frame_map_style_sheet)
                    if hasattr(self, 'last_passed_pb') and self.last_passed_pb:
                        style = f'background-color: {PATH_COLOR};'
                        self.last_passed_pb.setStyleSheet(style)
                    return
                # id_list = location.front.front.id
                # current_pos_float = location.current_position
                # log_info(f'[current_pos_float]\t {current_pos_float}')
                # if current_pos_float:
                #     self.lb_current_pos.setText(str(current_pos_float))

                id_list = data[0]
                # id_list = [8, 9]
                # id_list = [14, 15]
                # id_list = [116, 117]
                id = None
                for i in id_list:
                    if i:
                        id = i
                        break
                self.current_pos_float = data[1]
                # log_info(f'[type(current_pos_float)], {current_pos_float}')
                # current_pos_float = 0.5
                if not id or not self.current_pos_float:
                    log_info(f'无id或无距离, id:{id} 距离: {self.current_pos_float}')
                    self.current_pos_index = None
                    return

                if hasattr(self, 'map_manager'):
                    # self.current_pos_index = self.map_manager.from_coordinate_to_index([current_pos_float[0], current_pos_float[1]])
                    self.current_pos_index = self.map_manager.from_id_to_index(id, float(self.current_pos_float))

                    # 目标点样式
                    if not self.current_pos_index:
                        self.frame_map.setStyleSheet(self.frame_map_style_sheet)
                        if self.last_passed_pb:
                            style = f'background-color: {GOAL_POINT_COLOR};'
                            self.last_passed_pb.setStyleSheet(style)
                        return
                    # 当前点样式
                    x_index, y_index = self.current_pos_index
                    if x_index and y_index:
                        self.lb_current_pos.setText(f'[位置]{id}前{"{:.2f}".format(float(self.current_pos_float)*1000)}\n[索引][{x_index}, {y_index}]')
                        self.frame_map.setStyleSheet(self.frame_map_style_sheet)
                        if self.last_passed_pb and self.last_passed_pb.objectName() != f'pb_map_{x_index}_{y_index}':
                            style = f'background-color: {PATH_COLOR};'
                            self.last_passed_pb.setStyleSheet(style)
                        for i in self.list_pb_map:
                            i: QPushButton
                            if i.objectName() == f'pb_map_{x_index}_{y_index}':
                                style = f'background-color: {CURRENT_POINT_COLOR};'
                                i.setStyleSheet(style)
                                self.last_passed_pb: QPushButton = i
        except Exception as e:
            e = traceback.format_exc()
            text = f'[客户端][display_current_pos][!错误!]: \n{e}'
            log_error(e)
            self.append_TB_text(text, self.tb_console)

    def display_jlocation(self, data):
        location = self.jlocation_pack_from_dict(data)
        # self.display_current_pos(location)
        location_dict = {
            'lb_april_front_front_distance_x': location.front.front.distance.xStr(),
            'lb_april_front_front_distance_y': location.front.front.distance.yStr(),
            'lb_april_front_front_distance_z': location.front.front.distance.zStr(),
            'lb_april_front_front_orientation_x': location.front.front.orientation.xStr(),
            'lb_april_front_front_orientation_y': location.front.front.orientation.yStr(),
            'lb_april_front_front_orientation_z': location.front.front.orientation.zStr(),
            'lb_april_front_front_id': location.front.front.id,
            'lb_april_front_left_distance_x': location.front.left.distance.xStr(),
            'lb_april_front_left_distance_y': location.front.left.distance.yStr(),
            'lb_april_front_left_distance_z': location.front.left.distance.zStr(),
            'lb_april_front_left_orientation_x': location.front.left.orientation.xStr(),
            'lb_april_front_left_orientation_y': location.front.left.orientation.yStr(),
            'lb_april_front_left_orientation_z': location.front.left.orientation.zStr(),
            'lb_april_front_left_id': location.front.left.id,
            'lb_april_front_right_distance_x': location.front.right.distance.xStr(),
            'lb_april_front_right_distance_y': location.front.right.distance.yStr(),
            'lb_april_front_right_distance_z': location.front.right.distance.zStr(),
            'lb_april_front_right_orientation_x': location.front.right.orientation.xStr(),
            'lb_april_front_right_orientation_y': location.front.right.orientation.yStr(),
            'lb_april_front_right_orientation_z': location.front.right.orientation.zStr(),
            'lb_april_front_right_id': location.front.right.id,
            'lb_imu_o_x': location.imu.orientation.xStr(),
            'lb_imu_o_y': location.imu.orientation.yStr(),
            'lb_imu_o_z': location.imu.orientation.zStr(),
            'lb_imu_v_x': location.imu.velocity.xStr(),
            'lb_imu_v_y': location.imu.velocity.yStr(),
            'lb_imu_v_z': location.imu.velocity.zStr(),
            'lb_imu_a_x': location.imu.acceleration.xStr(),
            'lb_imu_a_y': location.imu.acceleration.yStr(),
            'lb_imu_a_z': location.imu.acceleration.zStr(),
            'lb_imu_av_x': location.imu.angular_velocity.xStr(),
            'lb_imu_av_y': location.imu.angular_velocity.yStr(),
            'lb_imu_av_z': location.imu.angular_velocity.zStr(),
            'lb_imu_m_x': location.imu.magnetic_field.xStr(),
            'lb_imu_m_y': location.imu.magnetic_field.yStr(),
            'lb_imu_m_z': location.imu.magnetic_field.zStr()
        }
        trans_list = ['distance']
        try:
            for key, value in location_dict.items():
                for i in trans_list:
                    if i in key and value:
                        value = '{:.{}f}'.format(float(value)*1000, DIGITS_S)
                if not value and 'april' in key:
                    getattr(self.console_win, key).setText('')
                else:
                    getattr(self.console_win, key).setText(str(value))
            self.clear_group_box(self.console_win.gb_april_l0)
            self.clear_group_box(self.console_win.gb_april_l1)
            self.clear_group_box(self.console_win.gb_april_r0)
            self.clear_group_box(self.console_win.gb_april_r1)
            lb_l0_x_x_d = QLabel('x_mm')
            lb_l0_y_y_d = QLabel('y_mm')
            lb_l0_z_z_d = QLabel('z_mm')
            lb_l0_x_x_o = QLabel('x_degree')
            lb_l0_y_y_o = QLabel('y_degree')
            lb_l0_z_z_o = QLabel('z_degree')
            lb_l0_id_id = QLabel('id')
            lb_l1_x_x_d = QLabel('x_mm')
            lb_l1_y_y_d = QLabel('y_mm')
            lb_l1_z_z_d = QLabel('z_mm')
            lb_l1_x_x_o = QLabel('x_degree')
            lb_l1_y_y_o = QLabel('y_degree')
            lb_l1_z_z_o = QLabel('z_degree')
            lb_l1_id_id = QLabel('id')
            lb_r0_x_x_d = QLabel('x_mm')
            lb_r0_y_y_d = QLabel('y_mm')
            lb_r0_z_z_d = QLabel('z_mm')
            lb_r0_x_x_o = QLabel('x_degree')
            lb_r0_y_y_o = QLabel('y_degree')
            lb_r0_z_z_o = QLabel('z_degree')
            lb_r0_id_id = QLabel('id')
            lb_r1_x_x_d = QLabel('x_mm')
            lb_r1_y_y_d = QLabel('y_mm')
            lb_r1_z_z_d = QLabel('z_mm')
            lb_r1_x_x_o = QLabel('x_degree')
            lb_r1_y_y_o = QLabel('y_degree')
            lb_r1_z_z_o = QLabel('z_degree')
            lb_r1_id_id = QLabel('id')
            # 判断是否只有一个或者没有码
            if len(location.left_list) > 0:
                if not hasattr(self, 'grid_l0'):
                    self.grid_l0 = QGridLayout()
                else:
                    self.clear_gridlayout(self.grid_l0)
                if location.left_list[0]:
                    lb_l0_x_d = QLabel('{:.{}f}'.format(float(location.left_list[0].distance.xStr())*1000, DIGITS_S))
                    lb_l0_y_d = QLabel('{:.{}f}'.format(float(location.left_list[0].distance.yStr())*1000, DIGITS_S))
                    lb_l0_z_d = QLabel('{:.{}f}'.format(float(location.left_list[0].distance.zStr())*1000, DIGITS_S))
                    lb_l0_x_o = QLabel(location.left_list[0].orientation.xStr())
                    lb_l0_y_o = QLabel(location.left_list[0].orientation.yStr())
                    lb_l0_z_o = QLabel(location.left_list[0].orientation.zStr())
                    lb_l0_id = QLabel(str(location.left_list[0].id))
                elif not self.flag_save_last_data:
                    lb_l0_x_d = QLabel('None')
                    lb_l0_y_d = QLabel('None')
                    lb_l0_z_d = QLabel('None')
                    lb_l0_x_o = QLabel('None')
                    lb_l0_y_o = QLabel('None')
                    lb_l0_z_o = QLabel('None')
                    lb_l0_id = QLabel('None')
                self.grid_l0.addWidget(lb_l0_x_x_d, 0, 0)
                self.grid_l0.addWidget(lb_l0_x_d, 0, 1)
                self.grid_l0.addWidget(lb_l0_x_x_o, 0, 2)
                self.grid_l0.addWidget(lb_l0_x_o, 0, 3)
                self.grid_l0.addWidget(lb_l0_y_y_d, 1, 0)
                self.grid_l0.addWidget(lb_l0_y_d, 1, 1)
                self.grid_l0.addWidget(lb_l0_y_y_o, 1, 2)
                self.grid_l0.addWidget(lb_l0_y_o, 1, 3)
                self.grid_l0.addWidget(lb_l0_z_z_d, 2, 0)
                self.grid_l0.addWidget(lb_l0_z_d, 2, 1)
                self.grid_l0.addWidget(lb_l0_z_z_o, 2, 2)
                self.grid_l0.addWidget(lb_l0_z_o, 2, 3)
                self.grid_l0.addWidget(lb_l0_id_id, 3, 0)
                self.grid_l0.addWidget(lb_l0_id, 3, 1)
                self.console_win.gb_april_l0.setLayout(self.grid_l0)

            if len(location.left_list) > 1:
                if not hasattr(self, 'grid_l1'):
                    self.grid_l1 = QGridLayout()
                else:
                    self.clear_gridlayout(self.grid_l1)
                if location.left_list[1]:
                    lb_l1_x_d = QLabel('{:.{}f}'.format(float(location.left_list[1].distance.xStr())*1000, DIGITS_S))
                    lb_l1_y_d = QLabel('{:.{}f}'.format(float(location.left_list[1].distance.yStr())*1000, DIGITS_S))
                    lb_l1_z_d = QLabel('{:.{}f}'.format(float(location.left_list[1].distance.zStr())*1000, DIGITS_S))
                    lb_l1_x_o = QLabel(location.left_list[1].orientation.xStr())
                    lb_l1_y_o = QLabel(location.left_list[1].orientation.yStr())
                    lb_l1_z_o = QLabel(location.left_list[1].orientation.zStr())
                    lb_l1_id = QLabel(str(location.left_list[1].id))
                elif not self.flag_save_last_data:
                    lb_l1_x_d = QLabel('None')
                    lb_l1_y_d = QLabel('None')
                    lb_l1_z_d = QLabel('None')
                    lb_l1_x_o = QLabel('None')
                    lb_l1_y_o = QLabel('None')
                    lb_l1_z_o = QLabel('None')
                    lb_l1_id = QLabel('None')
                self.grid_l1.addWidget(lb_l1_x_x_d, 0, 0)
                self.grid_l1.addWidget(lb_l1_x_d, 0, 1)
                self.grid_l1.addWidget(lb_l1_x_x_o, 0, 2)
                self.grid_l1.addWidget(lb_l1_x_o, 0, 3)
                self.grid_l1.addWidget(lb_l1_y_y_d, 1, 0)
                self.grid_l1.addWidget(lb_l1_y_d, 1, 1)
                self.grid_l1.addWidget(lb_l1_y_y_o, 1, 2)
                self.grid_l1.addWidget(lb_l1_y_o, 1, 3)
                self.grid_l1.addWidget(lb_l1_z_z_d, 2, 0)
                self.grid_l1.addWidget(lb_l1_z_d, 2, 1)
                self.grid_l1.addWidget(lb_l1_z_z_o, 2, 2)
                self.grid_l1.addWidget(lb_l1_z_o, 2, 3)
                self.grid_l1.addWidget(lb_l1_id_id, 3, 0)
                self.grid_l1.addWidget(lb_l1_id, 3, 1)
                self.console_win.gb_april_l1.setLayout(self.grid_l1)

            if len(location.right_list) > 0:
                if not hasattr(self, 'grid_r0'):
                    self.grid_r0 = QGridLayout()
                else:
                    self.clear_gridlayout(self.grid_r0)
                if location.right_list[0]:
                    lb_r0_x_d = QLabel('{:.{}f}'.format(float(location.right_list[0].distance.xStr())*1000, DIGITS_S))
                    lb_r0_y_d = QLabel('{:.{}f}'.format(float(location.right_list[0].distance.yStr())*1000, DIGITS_S))
                    lb_r0_z_d = QLabel('{:.{}f}'.format(float(location.right_list[0].distance.zStr())*1000, DIGITS_S))
                    lb_r0_x_o = QLabel(location.right_list[0].orientation.xStr())
                    lb_r0_y_o = QLabel(location.right_list[0].orientation.yStr())
                    lb_r0_z_o = QLabel(location.right_list[0].orientation.zStr())
                    lb_r0_id = QLabel(str(location.right_list[0].id))
                elif not self.flag_save_last_data:
                    lb_r0_x_d = QLabel('None')
                    lb_r0_y_d = QLabel('None')
                    lb_r0_z_d = QLabel('None')
                    lb_r0_x_o = QLabel('None')
                    lb_r0_y_o = QLabel('None')
                    lb_r0_z_o = QLabel('None')
                    lb_r0_id = QLabel('None')
                self.grid_r0.addWidget(lb_r0_x_x_d, 0, 0)
                self.grid_r0.addWidget(lb_r0_x_d, 0, 1)
                self.grid_r0.addWidget(lb_r0_x_x_o, 0, 2)
                self.grid_r0.addWidget(lb_r0_x_o, 0, 3)
                self.grid_r0.addWidget(lb_r0_y_y_d, 1, 0)
                self.grid_r0.addWidget(lb_r0_y_d, 1, 1)
                self.grid_r0.addWidget(lb_r0_y_y_o, 1, 2)
                self.grid_r0.addWidget(lb_r0_y_o, 1, 3)
                self.grid_r0.addWidget(lb_r0_z_z_d, 2, 0)
                self.grid_r0.addWidget(lb_r0_z_d, 2, 1)
                self.grid_r0.addWidget(lb_r0_z_z_o, 2, 2)
                self.grid_r0.addWidget(lb_r0_z_o, 2, 3)
                self.grid_r0.addWidget(lb_r0_id_id, 3, 0)
                self.grid_r0.addWidget(lb_r0_id, 3, 1)
                self.console_win.gb_april_r0.setLayout(self.grid_r0)
            if len(location.right_list) > 1:
                if not hasattr(self, 'grid_r1'):
                    self.grid_r1 = QGridLayout()
                else:
                    self.clear_gridlayout(self.grid_r1)
                if location.right_list[1]:
                    lb_r1_x_d = QLabel('{:.{}f}'.format(float(location.right_list[1].distance.xStr())*1000, DIGITS_S))
                    lb_r1_y_d = QLabel('{:.{}f}'.format(float(location.right_list[1].distance.yStr())*1000, DIGITS_S))
                    lb_r1_z_d = QLabel('{:.{}f}'.format(float(location.right_list[1].distance.zStr())*1000, DIGITS_S))
                    lb_r1_x_o = QLabel(location.right_list[1].orientation.xStr())
                    lb_r1_y_o = QLabel(location.right_list[1].orientation.yStr())
                    lb_r1_z_o = QLabel(location.right_list[1].orientation.zStr())
                    lb_r1_id = QLabel(str(location.right_list[1].id))
                elif not self.flag_save_last_data:
                    lb_r1_x_d = QLabel('None')
                    lb_r1_y_d = QLabel('None')
                    lb_r1_z_d = QLabel('None')
                    lb_r1_x_o = QLabel('None')
                    lb_r1_y_o = QLabel('None')
                    lb_r1_z_o = QLabel('None')
                    lb_r1_id = QLabel('None')
                self.grid_r1.addWidget(lb_r1_x_x_d, 0, 0)
                self.grid_r1.addWidget(lb_r1_x_d, 0, 1)
                self.grid_r1.addWidget(lb_r1_x_x_o, 0, 2)
                self.grid_r1.addWidget(lb_r1_x_o, 0, 3)
                self.grid_r1.addWidget(lb_r1_y_y_d, 1, 0)
                self.grid_r1.addWidget(lb_r1_y_d, 1, 1)
                self.grid_r1.addWidget(lb_r1_y_y_o, 1, 2)
                self.grid_r1.addWidget(lb_r1_y_o, 1, 3)
                self.grid_r1.addWidget(lb_r1_z_z_d, 2, 0)
                self.grid_r1.addWidget(lb_r1_z_d, 2, 1)
                self.grid_r1.addWidget(lb_r1_z_z_o, 2, 2)
                self.grid_r1.addWidget(lb_r1_z_o, 2, 3)
                self.grid_r1.addWidget(lb_r1_id_id, 3, 0)
                self.grid_r1.addWidget(lb_r1_id, 3, 1)
                self.console_win.gb_april_r1.setLayout(self.grid_r1)
            self.lb_o_x.setText(location.imu.orientation.xStr())
            self.lb_o_y.setText(location.imu.orientation.yStr())
            self.lb_o_z.setText(location.imu.orientation.zStr())
            self.lb_a_x.setText(location.imu.acceleration.xStr())
            self.lb_a_y.setText(location.imu.acceleration.yStr())
            self.lb_a_z.setText(location.imu.acceleration.zStr())
            self.lb_av_x.setText(location.imu.angular_velocity.xStr())
            self.lb_av_y.setText(location.imu.angular_velocity.yStr())
            self.lb_av_z.setText(location.imu.angular_velocity.zStr())
            self.lb_m_x.setText(location.imu.magnetic_field.xStr())
            self.lb_m_y.setText(location.imu.magnetic_field.yStr())
            self.lb_m_z.setText(location.imu.magnetic_field.zStr())
            self.update_plot_list(location)
        except Exception as e:
            e = traceback.format_exc()
            text = f'[客户端][display_jlocation][!错误!]: \n{e}'
            log_error(e)
            self.append_TB_text(text, self.tb_console)

    def jlocation_pack_from_dict(self, data):
        try:
            location = JLocation()
            location.front.front.set_distance(data['front']['front']['distance'])
            location.front.front.set_orientation(data['front']['front']['orientation'])
            location.front.front.set_id(data['front']['front']['id'])
            location.front.left.set_distance(data['front']['left']['distance'])
            location.front.left.set_orientation(data['front']['left']['orientation'])
            location.front.left.set_id(data['front']['left']['id'])
            location.front.right.set_distance(data['front']['right']['distance'])
            location.front.right.set_orientation(data['front']['right']['orientation'])
            location.front.right.set_id(data['front']['right']['id'])
            # location.set_current_position(data['current_position'])
            left_list = []
            right_list = []
            if len(data['left_list']) > 0:
                left_list = [None]
                if data['left_list'][0]:
                    left_list[0] = JApril_Tag_Info()
                    left_list[0].set_distance(data['left_list'][0]['distance'])
                    left_list[0].set_orientation(data['left_list'][0]['orientation'])
                    left_list[0].set_id(data['left_list'][0]['id'])
                if len(data['left_list']) == 2:
                    left_list.append(None)
                    if data['left_list'][1]:
                        left_list[1] = JApril_Tag_Info()
                        left_list[1].set_distance(data['left_list'][1]['distance'])
                        left_list[1].set_orientation(data['left_list'][1]['orientation'])
                        left_list[1].set_id(data['left_list'][1]['id'])
                location.set_left_list(left_list)
            if len(data['right_list']) > 0:
                right_list = [None]
                if data['right_list'][0]:
                    right_list[0] = JApril_Tag_Info()
                    right_list[0].set_distance(data['right_list'][0]['distance'])
                    right_list[0].set_orientation(data['right_list'][0]['orientation'])
                    right_list[0].set_id(data['right_list'][0]['id'])
                if len(data['right_list']) == 2:
                    right_list.append(None)
                    if data['right_list'][1]:
                        right_list[1] = JApril_Tag_Info()
                        right_list[1].set_distance(data['right_list'][1]['distance'])
                        right_list[1].set_orientation(data['right_list'][1]['orientation'])
                        right_list[1].set_id(data['right_list'][1]['id'])
                location.set_right_list(right_list)
            location.imu.orientation.set_x(data['imu']['orientation'][0])
            location.imu.orientation.set_y(data['imu']['orientation'][1])
            location.imu.orientation.set_z(data['imu']['orientation'][2])
            location.imu.velocity.set_x(data['imu']['velocity'][0])
            location.imu.velocity.set_y(data['imu']['velocity'][1])
            location.imu.velocity.set_z(data['imu']['velocity'][2])
            location.imu.angular_velocity.set_x(data['imu']['angular_velocity'][0])
            location.imu.angular_velocity.set_y(data['imu']['angular_velocity'][1])
            location.imu.angular_velocity.set_z(data['imu']['angular_velocity'][2])
            location.imu.acceleration.set_x(data['imu']['acceleration'][0])
            location.imu.acceleration.set_y(data['imu']['acceleration'][1])
            location.imu.acceleration.set_z(data['imu']['acceleration'][2])
            location.imu.angular_acceleration.set_x(data['imu']['angular_acceleration'][0])
            location.imu.angular_acceleration.set_y(data['imu']['angular_acceleration'][1])
            location.imu.angular_acceleration.set_z(data['imu']['angular_acceleration'][2])
            location.imu.magnetic_field.set_x(data['imu']['magnetic_field'][0])
            location.imu.magnetic_field.set_y(data['imu']['magnetic_field'][1])
            location.imu.magnetic_field.set_z(data['imu']['magnetic_field'][2])

            return location
        except Exception as e:
            e = traceback.format_exc()
            text = f'[客户端][jlocation_pack_from_dict][!错误!]: \n{e}'
            log_error(e)
            self.append_TB_text(text, self.tb_console)

    def clear_group_box(self, group_box: QGroupBox):
        try:
            layout = group_box.layout()
            if layout:
                while layout.count():
                    item = layout.takeAt(0)
                    widget = item.widget()
                    if widget is not None:
                        widget.deleteLater()
        except Exception as e:
            e = traceback.format_exc()
            text = f'[客户端][clear_group_box][!错误!]: \n{e}'
            log_error(e)
            self.append_TB_text(text, self.tb_console)

    def clear_gridlayout(self, grid):
        try:
            layout = grid.layout()
            if layout:
                # 移除布局中的所有小部件
                for i in reversed(range(layout.count())):
                    widget = layout.itemAt(i).widget()
                    if widget:
                        widget.setParent(None)  # 将 widget 分离，防止内存泄漏

                # 删除旧布局
                QWidget().setLayout(layout)
        except Exception as e:
            e = traceback.format_exc()
            text = f'[客户端][clear_gridlayout][!错误!]: \n{e}'
            log_error(e)
            self.append_TB_text(text, self.tb_console)

    def check_append_data(self, list_obj: list, content):
        if len(list_obj) >= PLOT_LIST_LENGTH:
            del list_obj[0]
        if hasattr(content, 'distance'):
            list_obj.append([
                content.distance.x(), content.distance.y(), content.distance.z(), content.orientation.x(), content.orientation.y(), content.orientation.z()
            ])
        else:
            list_obj.append([
                content.x(), content.y(), content.z()
            ])

    def update_plot_list(self, location: JLocation):
        location = copy.deepcopy(location)
        front_list = self.plot_list['front']
        left_list = self.plot_list['left']
        right_list = self.plot_list['right']
        imu_ori_list = self.plot_list['imu_ori']
        imu_acc_list = self.plot_list['imu_acc']
        imu_ang_spd_list = self.plot_list['imu_ang_spd']
        imu_mag_list = self.plot_list['imu_mag']

        # self.check_append_data(front_list, location.front.front)
        # if len(location.left_list) > 0:
        #     self.check_append_data(left_list, location.left_list[0])
        # if len(location.right_list) > 0:
        #     self.check_append_data(right_list, location.right_list[0])
        # self.check_append_data(imu_ori_list, location.imu.orientation)
        # self.check_append_data(imu_acc_list, location.imu.acceleration)
        # self.check_append_data(imu_ang_spd_list, location.imu.angular_acceleration)
        # self.check_append_data(imu_mag_list, location.imu.magnetic_field)

        # self.updata_plot_display(self.console_win.figure_front, self.console_win.canvas_front, front_list)
        # self.updata_plot_display(self.console_win.figure_left, self.console_win.canvas_left, left_list)
        # self.updata_plot_display(self.console_win.figure_right, self.console_win.canvas_right, right_list)
        # self.updata_plot_display(self.console_win.figure_imu_ori, self.console_win.canvas_imu_ori, imu_ori_list)
        # self.updata_plot_display(self.console_win.figure_imu_acc, self.console_win.canvas_imu_acc, imu_acc_list)
        # self.updata_plot_display(self.console_win.figure_imu_ang_spd, self.console_win.canvas_imu_ang_spd, imu_ang_spd_list)
        # self.updata_plot_display(self.console_win.figure_imu_mag, self.console_win.canvas_imu_mag, imu_mag_list)

    def updata_plot_display(self, figure: Figure, canvas: FigureCanvas, data):
        # 时间轴和数据生成
        sample_num = list(range(PLOT_LIST_LENGTH))
        data0 = [[0] * PLOT_LIST_LENGTH for _ in range(3)]
        data1 = [[0] * PLOT_LIST_LENGTH for _ in range(3)]
        flag_two_plot = False

        # 更新数据
        for index, item in enumerate(data):

            if item[0] is None and item[1] is None and item[2] is None:
                return

            if len(item) > 3:
                if item[3] is not None or item[4] is not None or item[5] is not None:
                    flag_two_plot = True
                    data1[0][index] = item[3]
                    data1[1][index] = item[4]
                    data1[2][index] = item[5]
            data0[0][index] = item[0]
            data0[1][index] = item[1]
            data0[2][index] = item[2]

        # 清除以前的图形
        ax_main = None
        ax_secondary = None
        figure.clear()

        # 创建主轴
        ax_main = figure.add_subplot(111)

        # 绘制第一组数据 (左侧y轴) - 使用实线
        colors1 = ['b', 'g', 'r']
        line_styles1 = ['-', '-', '-']
        lines1 = []

        for i, (color, line_style) in enumerate(zip(colors1, line_styles1)):
            label = 'XYZ'[i]
            line, = ax_main.plot(sample_num, data0[i], label=label, color=color, linestyle=line_style)
            lines1.append(line)

        # 设置左侧y轴标签
        ax_main.set_ylabel('XYZ')
        ax_main.tick_params(axis='y')

        lines = lines1
        if flag_two_plot:
            # 创建共享x轴的新y轴 (右侧y轴)
            ax_secondary = ax_main.twinx()
            colors2 = ['b', 'g', 'r']
            line_styles2 = ['--', '--', '--']
            lines2 = []

            for i, (color, line_style) in enumerate(zip(colors2, line_styles2)):
                label = 'XYZ-Rotation'[i]
                line, = ax_secondary.plot(sample_num, data1[i], label=label, color=color, linestyle=line_style)
                lines2.append(line)

            # 设置右侧y轴标签
            ax_secondary.set_ylabel('XYZ-Rotation')
            ax_secondary.tick_params(axis='y')
            lines = lines1 + lines2

        # # 设置x轴标签
        # ax_main.set_xlabel('Samples')

        # 合并所有线条标签并生成图例
        labs = [line.get_label() for line in lines]
        ax_main.legend(lines, labs, loc='upper left', bbox_to_anchor=(0, 1), ncol=1)

        # 调整布局
        figure.tight_layout()
        canvas.draw()
