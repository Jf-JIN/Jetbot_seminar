
from JClientUI import *
from JClient_Server_Console import *
from JClient_Server_Video import *
from PyQt5.QtWidgets import QMessageBox, QLabel
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
        time.sleep(1)

    def parameter_init(self) -> None:
        super().parameter_init()

    def signal_connections(self) -> None:
        super().signal_connections()
        self.pb_connect.clicked.connect(self.connection_to_server)
        self.pb_close_server.clicked.connect(self.close_server)
        self.console_win.pb_roscore_start.clicked.connect(self.send_console_to_server)
        self.console_win.pb_roscore_stop.clicked.connect(lambda: self.emergency_stop('roscore'))

    # 获取ip地址
    def get_ip(self) -> str:
        return self.le_jetbot_ip.text().strip()

    # 连接服务端
    def connection_to_server(self) -> None:
        if self.flag_connection_to_server:
            return
        ip = self.get_ip()
        if not ip:
            QMessageBox.warning(None, '提示', '请填写Jetbot的ip地址')
            return
        # ==================================== Client_Video_QThread ====================================
        self.client_video = Client_Video_QThread(ip)
        self.client_video.signal_connected_port.connect(functools.partial(self.load_jetbot_port, self.lb_video_port))       # 端口信号
        self.client_video.signal_connected_flag.connect(functools.partial(self.clear_port_display, self.lb_video_port))     # 断开连接信号
        self.client_video.signal_error_output.connect(functools.partial(self.append_TB_text, self.tb_console))
        self.client_video.signal_data_video_recv.connect(self.video_update)     # 接收视频信号

        self.signal_data_close_client_send.connect(self.client_video.send)      # 发送客户端关闭信号
        self.signal_data_close_server_send.connect(self.client_video.send)      # 发送服务器关闭信号
        self.client_video.start()
        # ==================================== Client_Console_QThread ====================================
        self.client_console = Client_Console_QThread(ip)
        self.client_console.signal_connected_port.connect(functools.partial(self.load_jetbot_port, self.lb_console_port))     # 端口信号
        self.client_console.signal_connected_flag.connect(functools.partial(self.clear_port_display, self.lb_console_port))     # 断开连接信号
        self.client_console.signal_data_console_recv.connect(self.console_in_tb_display)        # 接收命令提示符返回内容
        self.signal_data_console_send.connect(self.client_console.send)        # 发送信号
        self.signal_data_close_client_send.connect(self.client_console.send)   # 发送客户端关闭信号
        self.signal_data_close_server_send.connect(self.client_console.send)   # 发送服务器关闭信号
        self.client_console.start()

    # 加载端口
    def load_jetbot_port(self, tag: QLabel, port: str):
        print('load_jetbot_port \t', port)
        tag.setText(port)
        self.change_client_server_connection_display()

    # 断开连接时，清空端口显示
    def clear_port_display(self, tag: QLabel, flag: bool) -> None:
        if not flag:
            tag.clear()
            self.change_client_server_connection_display()

    # 显示更新视频
    def video_update(self, pixmap: QPixmap):
        self.lb_image.setPixmap(pixmap)

    # 从客户端关闭服务器
    def close_server(self):
        if self.flag_connection_to_server:
            text = {'server_close': 'Server will be closed'}
            self.signal_data_close_server_send.emit(text)

# **************************************** 子窗口功能 ****************************************
    def send_console_to_server(self):
        sender = self.sender()
        data = self.console_win.build_console_dict()  # 获取所有 LineEdit 的文字
        print(f'[命令字典]: {data}')
        print(f'[触发控件]: {sender}')
        key = None
        if sender == self.console_win.pb_roscore_start:
            key = 'roscore'
        if key == None:
            return
        command_data = {key: data[key]}
        print(f'[发送命令信号]: {command_data}')
        self.signal_data_console_send.emit(command_data)

    # 数据解包
    def console_in_tb_display(self, data):
        print(f'[接收命令行包]: {data}')
        if 'roscore' in data:
            self.append_TE_text(data['roscore'], self.console_win.tb_roscore)

    def emergency_stop(self, key):
        self.signal_data_console_send.emit({key: 'Emergency_Stop'})
