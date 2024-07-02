
from PyQt5.QtWidgets import QMessageBox, QLabel, QLineEdit, QGroupBox, QGridLayout


from JClientUI import *
from JClient_Server_Console import *
from JClient_Server_Video import *
from JLocation import *
from JAlgorithm_JMap_Matrix_Manager import *


import functools
import threading


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
        self.current_pos_index = True
        self.list_connection_map_pb = []
        self.last_passed_pb = None

    def signal_connections(self) -> None:
        super().signal_connections()
        self.pb_connect.clicked.connect(self.connection_to_server)
        self.pb_reconnect_console.clicked.connect(self.console_connection_to_server)
        self.pb_reconnect_video.clicked.connect(self.video_connection_to_server)
        self.pb_close_server.clicked.connect(self.close_server)
        self.pb_camera_listener.clicked.connect(lambda: self.signal_data_console_send.emit({'camera_listener': 'on'}))
        self.console_win.pb_jlocation_listen_init.clicked.connect(lambda: self.signal_data_console_send.emit({'jlocation_listener': 'on'}))
        self.console_win.pb_ros_node_init.clicked.connect(lambda: self.signal_data_console_send.emit({'ros_node_init': 'on'}))
        self.console_win.pb_ros_apriltag_detection_start.clicked.connect(lambda: self.signal_data_console_send.emit({'camera_listener': 'on'}))
        self.tabWidget.currentChanged.connect(self.clear_launch_function)
        self.pb_launch.clicked.connect(self.launch_function)
    # ==================================== JConsole 信号连接 ====================================
        self.pb_le_init('roscore')
        self.pb_le_init('ros_camera')
        self.pb_le_init('ros_rectify')
        self.pb_le_init('ros_apriltag_detection')
        self.pb_le_init('ros_imu')
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

    # 用于执行任务
    def launch_function(self):
        if self.tabWidget.currentIndex() == 0:
            self.map_pb_function_connection()
            tt = threading.Thread(target=self.signal_sort_test)
            tt.start()

    # 用于清除执行任务后的关联
    def clear_launch_function(self):
        if self.list_connection_map_pb:
            for index, item in enumerate(self.list_connection_map_pb):
                self.list_pb_map[index].clicked.disconnect(item)
            self.list_connection_map_pb = []

    # 对按钮和功能进行连接，主要是地图按钮和功能
    def map_pb_function_connection(self):
        if self.current_pos_index:
            for i in self.list_pb_map:
                i: QPushButton
                connection = i.clicked.connect(functools.partial(self.map_pb_function, i))
                self.list_connection_map_pb.append(connection)

    # 地图按钮的功能
    def map_pb_function(self, sender):
        index = [int(sender.objectName().split('_')[-2]), int(sender.objectName().split('_')[-1])]
        self.lb_goal_index.setText(f'目标: {index}')
        self.path_finding = JPath_Finding_A_Star()
        text = {'a1_path_dict': self.path_finding.get_path_tree([1, 1], index, self.map_manager)}
        self.signal_data_console_send.emit(text)

    # 获取ip地址

    def get_ip(self) -> str:
        return self.le_jetbot_ip.text().strip()

    # 连接服务端

    def connection_to_server(self) -> None:
        self.video_connection_to_server()
        self.console_connection_to_server()

    # ==================================== Client_Video_QThread ====================================

    def video_connection_to_server(self) -> None:
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

    # ==================================== Client_Console_QThread ====================================
    def console_connection_to_server(self) -> None:
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

        self.signal_data_console_send.connect(self.client_console.send)        # 发送信号
        self.signal_data_close_client_send.connect(self.client_console.send)   # 发送客户端关闭信号
        self.signal_data_close_server_send.connect(self.client_console.send)   # 发送服务器关闭信号
        self.client_console.start()

    # 加载端口
    def load_jetbot_console_port(self, port: str):
        print('load_jetbot_console_port \t', port)
        self.lb_console_port.setText(port)
        self.change_client_server_connection_display()
        self.reconnect_display()
    # 加载端口

    def load_jetbot_video_port(self, port: str):
        print('load_jetbot_video_port \t', port)
        self.lb_video_port.setText(port)
        self.change_client_server_connection_display()
        self.reconnect_display()

    # 断开连接时，清空端口显示
    def clear_port_display(self, tag: QLabel, flag: bool) -> None:
        if not flag:
            tag.clear()
            self.change_client_server_connection_display()
            self.reconnect_display()

    # 显示更新视频
    def video_update(self, pixmap: QPixmap):
        origal_width = pixmap.width()
        origal_height_width_rate = pixmap.height() / pixmap.width()
        width_rate = self.hs_video_size.value()
        width = int(origal_width * width_rate * 1.5 / 100)
        height = int(width * origal_height_width_rate)
        scaled_pixmap = pixmap.scaled(width, height, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.lb_image.setPixmap(scaled_pixmap)

    # 从客户端关闭服务器
    def close_server(self):
        if self.flag_connection_to_server:
            text = {'server_close': 'Server will be closed'}
            self.signal_data_close_server_send.emit(text)

    # 客户端向服务端发送关闭信号
    def send_close_signal(self):
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

        # **************************************** 子窗口功能 ****************************************

    def send_console_to_server(self, key: str):
        sender = self.sender()
        data = self.console_win.build_console_dict()  # 获取所有 LineEdit 控件
        print(f'[触发控件]: {sender}')
        line_edit_widget: QLineEdit = data[key]
        line_edit_text = line_edit_widget.text()
        if line_edit_text == '':
            line_edit_text = line_edit_widget.placeholderText()
        command_data = {key: line_edit_text}
        print(f'[发送命令信号]: {command_data}')
        self.signal_data_console_send.emit(command_data)
        line_edit_widget.clear()

    def signal_sort_test(self):
        distance = 0.8
        while distance > 0:
            distance = distance - 0.01
            text = {'current_pos_float': [[116, 117], distance]}
            self.signal_sort(text)
            print(text)
            time.sleep(0.01)

    # 数据解包
    def signal_sort(self, data):
        data: dict
        print(f'[接收命令行包]: {data}')
        data_sort_dict = {
            'roscore': lambda: self.append_TB_text(data['roscore'], self.console_win.tb_roscore),
            'ros_camera': lambda: self.append_TB_text(data['ros_camera'], self.console_win.tb_ros_camera),
            'ros_rectify': lambda: self.append_TB_text(data['ros_rectify'], self.console_win.tb_ros_rectify),
            'ros_apriltag_detection': lambda: self.append_TB_text(data['ros_apriltag_detection'], self.console_win.tb_ros_apriltag_detection),
            'ros_imu': lambda: self.append_TB_text(data['ros_imu'], self.console_win.tb_ros_imu),
            'ros_motor': lambda: self.append_TB_text(data['ros_motor'], self.console_win.tb_ros_motor),
            'ros_algorithm': lambda: self.append_TB_text(data['ros_algorithm'], self.console_win.tb_ros_algorithm),
            'current_pos_float': lambda: self.display_current_pos(data['current_pos_float']),
            'jlocation_package': lambda: self.display_jlocation(data['jlocation_package']),
            'map_generation': lambda: self.map_display_update(data['map_generation'])
        }
        for key, value in data_sort_dict.items():
            if key in data:
                value()

    def map_display_update(self, data):
        # data 是一个json文件
        yaml_data = yaml.dump(data, default_flow_style=False)
        # map_manager = JMap_Matrix_From_Yaml(yaml_data)
        # map_abstract = map_manager.map_abstract_matrix
        pass

    def display_current_pos(self, data):
        id_list = data[0]
        for i in id_list:
            if i:
                id = i
                break
        current_pos_float = data[1]
        if hasattr(self, 'map_manager'):
            self.current_pos_index = self.map_manager.from_id_to_index(id, float(current_pos_float))
            if not self.current_pos_index:
                if self.last_passed_pb:
                    default_bgc_ss = self.last_passed_pb.styleSheet().replace('background-color: #EEEE00;', 'background-color: #006600;')
                    self.last_passed_pb.setStyleSheet(default_bgc_ss)
                return
            x_index, y_index = self.current_pos_index
            if self.last_passed_pb and self.last_passed_pb.objectName() != f'pb_map_{x_index}_{y_index}':
                default_bgc_ss = self.last_passed_pb.styleSheet().replace('background-color: #EEEE00;', 'background-color: #006600;')
                self.last_passed_pb.setStyleSheet(default_bgc_ss)
            for i in self.list_pb_map:
                i: QPushButton
                if i.objectName() == f'pb_map_{x_index}_{y_index}':
                    current_bgc_ss = i.styleSheet().replace('background-color: #006600;', 'background-color: #EEEE00;')
                    i.setStyleSheet(current_bgc_ss)
                    self.last_passed_pb: QPushButton = i
            print('按钮位置', f'pb_map_{x_index}_{y_index}')

    def display_jlocation(self, data):
        location = self.jlocation_pack_from_dict(data)
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
                else:
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
                else:
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
                else:
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
                else:
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
        except Exception as e:
            e = traceback.format_exc()
            print(e)

    def jlocation_pack_from_dict(self, data):
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

    def clear_group_box(self, group_box: QGroupBox):
        layout = group_box.layout()
        if layout:
            while layout.count():
                item = layout.takeAt(0)
                widget = item.widget()
                if widget is not None:
                    widget.deleteLater()

    def clear_gridlayout(self, grid):
        layout = grid.layout()
        if layout:
            # 移除布局中的所有小部件
            for i in reversed(range(layout.count())):
                widget = layout.itemAt(i).widget()
                if widget:
                    widget.setParent(None)  # 将 widget 分离，防止内存泄漏

            # 删除旧布局
            QWidget().setLayout(layout)

    def emergency_stop(self, key):
        self.signal_data_console_send.emit({key: 'Emergency_Stop'})
