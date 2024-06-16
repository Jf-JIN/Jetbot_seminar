
from JClientUI import *
from JClient_Server_Console import *
from JClient_Server_Video import *
from PyQt5.QtWidgets import QMessageBox, QLabel, QLineEdit
import functools


class JClient_Function(JClient_UI):
    signal_data_video_send = pyqtSignal(list)
    signal_data_console_send = pyqtSignal(dict)
    signal_data_close_client_send = pyqtSignal(dict)
    signal_data_close_server_send = pyqtSignal(dict)

    def __init__(self) -> None:
        super().__init__()
        self.parameter_init()
        self.signal_connections()
        time.sleep(1)

    def parameter_init(self) -> None:
        super().parameter_init()

    def signal_connections(self) -> None:
        super().signal_connections()
        self.pb_connect.clicked.connect(self.connection_to_server)
        self.pb_reconnect_console.clicked.connect(self.console_connection_to_server)
        self.pb_reconnect_video.clicked.connect(self.video_connection_to_server)
        self.pb_close_server.clicked.connect(self.close_server)
        self.pb_camera_listener.clicked.connect(lambda: self.signal_data_console_send.emit({'camera_listener': 'on'}))
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
        self.client_console.signal_data_console_recv.connect(self.console_in_tb_display)        # 接收命令提示符返回内容
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
        # print(pixmap)
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

    # 数据解包
    def console_in_tb_display(self, data):
        print(f'[接收命令行包]: {data}')
        if 'roscore' in data:
            self.append_TB_text(data['roscore'], self.console_win.tb_roscore)
        elif 'ros_camera' in data:
            self.append_TB_text(data['ros_camera'], self.console_win.tb_ros_camera)
        elif 'ros_rectify' in data:
            self.append_TB_text(data['ros_rectify'], self.console_win.tb_ros_rectify)
        elif 'ros_apriltag_detection' in data:
            self.append_TB_text(data['ros_apriltag_detection'], self.console_win.tb_ros_apriltag_detection)
        elif 'ros_imu' in data:
            self.append_TB_text(data['ros_imu'], self.console_win.tb_ros_imu)
        elif 'ros_motor' in data:
            self.append_TB_text(data['ros_motor'], self.console_win.tb_ros_motor)
        elif 'ros_algorithm' in data:
            self.append_TB_text(data['ros_algorithm'], self.console_win.tb_ros_algorithm)

    def emergency_stop(self, key):
        self.signal_data_console_send.emit({key: 'Emergency_Stop'})
