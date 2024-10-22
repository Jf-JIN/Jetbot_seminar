
from PyQt5.QtWidgets import QMenuBar, QActionGroup, QAction, QTextBrowser, QTextEdit, QComboBox, QTabWidget, QSlider, QFileDialog, QPushButton, QLabel, QGridLayout, QSizePolicy
from PyQt5.QtGui import QPixmap, QIcon, QTextCursor, QFont
from PyQt5.QtCore import QByteArray, QSize, Qt
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

import socket
import os
import time
import traceback
import json
# import yaml
# from Icon_setting import *
from JClient_ui import *
from JConsoleUI import *
from JAlgorithm_JPath_Finding import *

log_info = logger_dict_client['info']
log_error = logger_dict_client['error']


class JClient_UI(Ui_MainWindow):
    signal_data_console_send = pyqtSignal(dict)

    def __init__(self) -> None:
        super().__init__()
        self.setupUi(self)
        self.parameter_init()
        self.UI_setup()

    def parameter_init(self):
        self.green_status = 'rgb(100, 190, 0)'
        self.red_status = 'rgb(255, 66, 0)'
        self.icon_size = QSize(32, 32)
        self.traceback_display_flag = True
        self.flag_connection_to_server = False
        self.console_win = JConcole()
        self.formatted_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))

    def signal_connections(self):
        self.pb_launch_2.clicked.connect(self.show_console_win)
        self.console_win.pb_main_win.clicked.connect(self.show_main_win)
        self.tabWidget.currentChanged.connect(self.cbb_and_tab_connection)
        self.cbb_task.currentIndexChanged.connect(self.cbb_and_tab_connection)
        self.hs_video_size.valueChanged.connect(self.hs_video_size_display)
        self.pb_a1_load.clicked.connect(self.load_yaml_file)
        self.pb_pid_distance_reset.clicked.connect(self.reset_pid_value)

    def UI_setup(self):
        self.pb_close_server.setEnabled(False)
        self.lb_connection_status.setText('disconnected')
        self.lb_connection_status.setStyleSheet(f'background-color: {self.red_status}')
        self.pb_reconnect_console.setStyleSheet(f'background-color: {self.red_status}')
        self.pb_reconnect_video.setStyleSheet(f'background-color: {self.red_status}')
        self.cbb_task.addItem('Aufgabe 1: Labyrinthlösung')
        self.cbb_task.addItem('Aufgabe 2: Erkundung und Mapping')
        self.cbb_task.addItem('Aufgabe 3: Search and Rescue')
        self.tb_console.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scrollArea.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scrollArea.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scrollArea_2.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scrollArea_2.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scrollArea_3.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scrollArea_3.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.lb_a1_discription.setWordWrap(True)
        self.lb_a2_discription.setWordWrap(True)
        self.lb_a3_discription.setWordWrap(True)
        self.menubar = QMenuBar()
        self.setMenuBar(self.menubar)
        self.menu_language = self.menubar.addMenu('语言')
        self.menu_action_group_language_change = QActionGroup(self)
        self.menu_action_Chinese = QAction('简体中文', self)
        self.menu_action_Chinese.setCheckable(True)
        self.menu_language.addAction(self.menu_action_Chinese)
        self.menu_action_group_language_change.addAction(self.menu_action_Chinese)
        self.menu_action_Deutsch = QAction('Deutsch', self)
        self.menu_action_Deutsch.setCheckable(True)
        self.menu_language.addAction(self.menu_action_Deutsch)
        self.menu_action_group_language_change.addAction(self.menu_action_Deutsch)
        self.hs_video_size.setRange(0, 100)  # 设置范围
        self.hs_video_size.setValue(40)       # 设置初始值
        self.lb_hs_display.setText('40')
        # self.hs_video_size.setTickPosition(QSlider.TicksBelow)  # 设置刻度位置
        self.hs_video_size.setTickInterval(10)  # 设置刻度间隔
        self.dsb_distance_kp.setValue(PID_DISTANCE_DEFAULT_KP)
        self.dsb_distance_kp.setSingleStep(DSB_STEP_S)
        self.dsb_distance_kp.setMaximum(100.0)
        self.dsb_distance_kp.setMinimum(-100.0)
        self.dsb_distance_ki.setValue(PID_DISTANCE_DEFAULT_KI)
        self.dsb_distance_ki.setSingleStep(DSB_STEP_S)
        self.dsb_distance_ki.setMaximum(100.0)
        self.dsb_distance_ki.setMinimum(-100.0)
        self.dsb_distance_kd.setValue(PID_DISTANCE_DEFAULT_KD)
        self.dsb_distance_kd.setSingleStep(DSB_STEP_S)
        self.dsb_distance_kd.setMaximum(100.0)
        self.dsb_distance_kd.setMinimum(-100.0)
        # self.dsb_angle_kp.setValue(PID_DISTANCE_DEFAULT_KP)
        # self.dsb_angle_kp.setSingleStep(DSB_STEP)
        # self.dsb_angle_kp.setMaximum(100.0)
        # self.dsb_angle_kp.setMinimum(-100.0)
        # self.dsb_angle_ki.setValue(PID_DISTANCE_DEFAULT_KI)
        # self.dsb_angle_ki.setSingleStep(DSB_STEP)
        # self.dsb_angle_ki.setMaximum(100.0)
        # self.dsb_angle_ki.setMinimum(-100.0)
        # self.dsb_angle_kd.setValue(PID_DISTANCE_DEFAULT_KD)
        # self.dsb_angle_kd.setSingleStep(DSB_STEP)
        # self.dsb_angle_kd.setMaximum(100.0)
        # self.dsb_angle_kd.setMinimum(-100.0)
        self.dsb_motor_left_calib.setValue(MOTOR_LEFT_CALIB)
        self.dsb_motor_left_calib.setSingleStep(DSB_STEP_S)
        self.dsb_motor_left_calib.setMaximum(10.0)
        self.dsb_motor_left_calib.setMinimum(-10.0)
        self.dsb_motor_right_calib.setValue(MOTOR_RIGHT_CALIB)
        self.dsb_motor_right_calib.setSingleStep(DSB_STEP_S)
        self.dsb_motor_right_calib.setMaximum(10.0)
        self.dsb_motor_right_calib.setMinimum(-10.0)

    def show_console_win(self):
        self.console_win.show()
        self.console_win.activateWindow()

    def show_main_win(self):
        self.show()
        self.activateWindow()

    def reset_pid_value(self):
        name = self.sender().objectName()
        if name == 'pb_pid_distance_reset':
            self.dsb_distance_kp.setValue(PID_DISTANCE_DEFAULT_KP)
            self.dsb_distance_ki.setValue(PID_DISTANCE_DEFAULT_KI)
            self.dsb_distance_kd.setValue(PID_DISTANCE_DEFAULT_KD)

    def change_client_server_connection_display(self) -> None:
        if self.lb_console_port.text() != '' and self.lb_video_port.text() != '':
            self.flag_connection_to_server = True
            self.pb_close_server.setEnabled(True)
            self.lb_connection_status.setText('connected')
            self.lb_connection_status.setStyleSheet(f'background-color: {self.green_status}')
        else:
            self.flag_connection_to_server = False
            self.pb_close_server.setEnabled(False)
            self.lb_connection_status.setText('disconnected')
            self.lb_connection_status.setStyleSheet(f'background-color: {self.red_status}')

    def reconnect_display(self):
        if self.lb_console_port.text() != '':
            self.pb_reconnect_console.setStyleSheet(f'background-color: {self.green_status}')
        else:
            self.pb_reconnect_console.setStyleSheet(f'background-color: {self.red_status}')
        if self.lb_video_port.text() != '':
            self.pb_reconnect_video.setStyleSheet(f'background-color: {self.green_status}')
        else:
            self.pb_reconnect_video.setStyleSheet(f'background-color: {self.red_status}')

    def icon_setup(self, icon_code):
        pixmap = QPixmap()
        pixmap.loadFromData(QByteArray(icon_code.encode()))
        return QIcon(pixmap)

    # 暂时无用
    def find_free_port(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(('localhost', 0))
            port = s.getsockname()[1]
            return str(port)

    def cbb_and_tab_connection(self):
        sender = self.sender()
        if isinstance(sender, QComboBox):
            goal_index = self.cbb_task.currentIndex()
            self.tabWidget.setCurrentIndex(goal_index)
        elif isinstance(sender, QTabWidget):
            goal_index = self.tabWidget.currentIndex()
            self.cbb_task.setCurrentIndex(goal_index)

    def hs_video_size_display(self, value):
        self.lb_hs_display.setText(str(value))

        # **************************************** 向 TextBrowser | TextEdit 添加内容 ****************************************

    def append_TB_text(self, text_content: str, textBrowser_object: QTextBrowser = None) -> None:
        if not textBrowser_object:
            textBrowser_object = self.tb_console
        try:
            textBrowser_object.moveCursor(QTextCursor.End)
            textBrowser_object.insertPlainText(str(text_content) + "\n")
            textBrowser_object.moveCursor(QTextCursor.End)
        except Exception as e:
            if self.traceback_display_flag:
                e = traceback.format_exc()
            textBrowser_object.moveCursor(QTextCursor.End)
            textBrowser_object.insertPlainText(str(e) + "\n")
            textBrowser_object.moveCursor(QTextCursor.End)

    def clearLayout(self, layout):
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
            else:
                self.clearLayout(item.layout())

    def load_yaml_file(self):
        options = QFileDialog.Options()
        try:
            yaml_file_path, _ = QFileDialog.getOpenFileName(None, '请选择需要加载的Yaml文件', '', 'yaml文件 (*.yaml)', 'yaml文件 (*.yaml)', options)
            if yaml_file_path:
                self.le_a1_path.setText(yaml_file_path)
                self.load_map_from_yaml(yaml_file_path)
                return yaml_file_path
            return None
        except Exception as e:
            e = traceback.format_exc()
            e_text = f'\n[加载yaml文件] - {self.formatted_time} \n[!错误!] {e}'
            log_error(e)
            self.append_TB_text(e_text, self.tb_console)

    def load_map_from_yaml(self, path):
        with open(path, 'r') as file:
            loaded_data = yaml.safe_load(file)
        # signal_text = json.dumps({'a1_map_yaml_dict': loaded_data})

        signal_text = {'a1_map_yaml_dict': loaded_data}
        self.signal_data_console_send.emit(signal_text)
        current_time_item = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        if 'standalone_tags' not in loaded_data:
            loaded_data['standalone_tags'] = loaded_data['tag_bundles'][0]['layout']
        loaded_data = {'time': current_time_item, **loaded_data}
        # signal_text = {'a1_map_yaml_dict': loaded_data}
        json_path_new = os.path.join(os.path.dirname(path), 'tags.json')
        yaml_path_new = os.path.join(os.path.dirname(path), 'tags.yaml')
        yaml_path_new_wall = os.path.join(os.path.dirname(path), 'wall.yaml')
        yaml_path_new_wall2 = os.path.join(os.path.dirname(path), 'pyscript', 'wall.yaml')
        with open(yaml_path_new, 'w') as yaml_file:
            yaml.dump(loaded_data, yaml_file, default_flow_style=False, sort_keys=False)

        with open(yaml_path_new_wall, 'w') as yaml_file_wall:
            yaml.dump(loaded_data, yaml_file_wall, default_flow_style=False, sort_keys=False)

        # with open(yaml_path_new_wall2, 'w') as yaml_file_wall2:
        #     yaml.dump(loaded_data, yaml_file_wall2, default_flow_style=False, sort_keys=False)

        with open(json_path_new, 'w') as json_file:
            json.dump(signal_text, json_file, indent=4)

        self.lb_goal_index.clear()
        self.map_manager = JMap_Grid_Matrix_From_Yaml(loaded_data)
        abstract_map = self.map_manager.map_abstract_matrix
        obj_map = self.map_manager.map_obj_matrix
        for i in abstract_map:
            self.append_TB_text(i, self.tb_console)
        # size_map = [len(abstract_map), len(abstract_map[0])]
        map_layout = self.frame_map.layout()
        if map_layout:
            self.clearLayout(map_layout)
            # self.frame_map.setLayout(None)
        else:
            map_layout = QGridLayout()
        map_layout.setVerticalSpacing(0)
        map_layout.setHorizontalSpacing(0)
        for i in range(len(abstract_map)):
            map_layout.setRowStretch(i, 1)
        for j in range(len(abstract_map[0])):
            map_layout.setColumnStretch(j, 1)
        self.list_pb_map = []
        for x_index, line_list in enumerate(abstract_map):
            # print(line_list)
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
                    path_middle_x, path_middle_y = self.map_manager.from_index_to_coordinate([x_index, y_index])
                    path_middle_x = '{:.{}f}'.format(path_middle_x * 1000, DIGITS_S)
                    path_middle_y = '{:.{}f}'.format(path_middle_y * 1000, DIGITS_S)
                    path_item.setObjectName(path_item_name)
                    path_item.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
                    path_item.setToolTip(f'索引:\t[{x_index}, {y_index}]\n位置:\t[{path_middle_x}, {path_middle_y}]')
                    path_item.setCursor(Qt.PointingHandCursor)
                    map_layout.addWidget(path_item, x_index, y_index)
                    self.list_pb_map.append(path_item)
            self.frame_map.setLayout(map_layout)
            self.frame_map_style_sheet =\
                'QFrame#frame_map QPushButton{' + \
                f'''background-color: {PATH_COLOR};
                    border: None;
                    border-radius: 0px;
                    min-width: 10px;
                    min-height:10px;''' + \
                '}' + \
                '''QFrame#frame_map QPushButton:hover''' + '{' + \
                f'background-color: {PATH_HOVER_COLOR};' + \
                '}' + \
                'QFrame#frame_map QLabel' + '{' +\
                '''min-width: 10px;
                    min-height:10px;''' + \
                '}'
            # frame_style_sheet = '''
            #                     QFrame#frame_map QPushButton{
            #                         background-color: #CFCC88;
            #                         border: None;
            #                         border-radius: 0px;
            #                         min-width: 10px;
            #                         min-height:10px;
            #                     }
            #                     QFrame#frame_map QPushButton:hover{
            #                         background-color: #CF5050;
            #                     }
            #                     QFrame#frame_map QLabel{
            #                         min-width: 10px;
            #                         min-height:10px;
            #                     }
            #                     '''

            self.frame_map.setStyleSheet(self.frame_map_style_sheet)

        # **************************************** 子窗口功能 ****************************************
