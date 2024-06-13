from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget
from PyQt5.QtGui import QCloseEvent
from PyQt5.QtCore import QObject

import sys
import socket

from JClient_Server_Video import *
from JClient_Server_Console import *
from JServer_Console_QThread import *


class JServer_Function(QMainWindow):
    signal_data_video_send = pyqtSignal(object)
    signal_data_console_send = pyqtSignal(dict)
    signal_close_server_send = pyqtSignal(dict)

    def __init__(self) -> None:
        super().__init__()
        self.parameter_init()
        self.server_init()
        time.sleep(1)

    def parameter_init(self) -> None:
        self.video_clients_list = []
        self.console_clients_list = []
        self.active_threads = {}

    # 获取服务器的ip地址
    def get_local_ip(self) -> str:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
        except Exception as e:
            local_ip = None
            print(f"Error: {e}")
        finally:
            s.close()
        return local_ip

    # 服务器初始化
    def server_init(self):
        ip = self.get_local_ip()
        self.server_video = Server_Vedio_QThread(ip)
        self.server_video.signal_connected_list.connect(self.get_console_connected_list)    # 获取 Video 服务器中的客户端列表
        self.signal_data_video_send.connect(self.server_video.send_img_all)                 # 向所有客户端发送视频数据
        self.server_video.signal_close_server.connect(self.close_server_by_client)          # 接收客户端传来的关闭信号
        self.signal_close_server_send.connect(self.server_video.send_all)                   # 向所有客户端发送关闭客户端的信号
        self.server_video.start()

        self.server_console = Server_Console_QThread(ip)
        self.server_console.signal_data_console_recv.connect(self.console_unpacking)            # 获取客户端发送来的命令行信号
        self.server_console.signal_connected_list.connect(self.get_console_connected_list)      # 获取 Console 服务器中的客户端列表
        self.server_console.signal_close_server.connect(self.close_server_by_client)            # 接收客户端传来的关闭信号
        self.signal_close_server_send.connect(self.server_console.send_all)                     # 向所有客户端发送关闭服务端的信号
        self.server_console.start()

    # 服务器发送视频信号
    # def send_video(self, video_data):
    #     self.signal_data_video_send.emit(video_data)
    def send_video(self):
        while 1:
            self.signal_data_video_send.emit(self.video_data)
            print('发送图片')
            time.sleep(2)

# **************************************** 命令行功能 ****************************************
    # 解包命令行信号
    @pyqtSlot(dict)
    def console_unpacking(self, data: dict):

        print('roscore' in data)
        # print('roscore' in self.active_threads)
        # print(data['roscore'] == 'Emergency_Stop')
        # print('roscore' in self.active_threads and data['roscore'] == 'Emergency_Stop')
        # print(f'[console_unpacking]: {data}')

        if 'roscore' in data:

            if 'roscore' not in self.active_threads and data['roscore'] != 'Emergency_Stop':
                self.concole_thread('roscore', data)
            elif 'roscore' in self.active_threads and data['roscore'] == 'Emergency_Stop':
                print('启动结束程序')
                self.active_threads['roscore'].stop_command()

    # 构建命令行线程
    def concole_thread(self, key, data):
        thread_under_concole_thread = JConsole_QThread(key, data[key])
        thread_under_concole_thread.signal_console_line.connect(self.server_console.send_all)
        thread_under_concole_thread.signal_finished.connect(lambda: self.thread_finish(key))
        # thread_under_concole_thread.
        thread_under_concole_thread.start()
        self.active_threads[key] = thread_under_concole_thread  # 保持对活动线程的引用
        print('[命令行线程][启动之后active_thread]: ', self.active_threads)

    # 结束命令行线程
    def thread_finish(self, key):
        print('结束命令行线程')
        sender_thrd = self.sender()
        print(f'[命令行线程][sender类型]: {type(sender_thrd)}')
        sender_thrd.wait()
        sender_thrd.quit()  # 停止线程的事件循环
        print('[命令行线程][删除之前active_thread]: ', self.active_threads)
        if key in self.active_threads:
            self.active_threads.pop(key)  # 删除对象的引用
        else:
            print(f"[命令行线程]: 线程键名 {key} 不存在于 active_threads 中")
        print('[命令行线程][删除之后active_thread]: ', self.active_threads)

# **************************************** 关闭服务器 ****************************************

    # 获取视频客户端列表

    def get_video_connected_list(self, clients_list: list) -> None:
        self.video_clients_list = clients_list

    # 获取命令行客户端列表
    def get_console_connected_list(self, clients_list: list) -> None:
        self.console_clients_list = clients_list

    # 从客户端关闭服务器
    def close_server_by_client(self):
        self.close_client_socket()
        # 此处关闭其他的线程
        sys.exit()

    # 关闭所有的客户端Socket
    def close_client_socket(self):
        for vclient in self.video_clients_list:
            vsocket: socket.socket = vclient[0]
            vsocket.close()
        for cclient in self.console_clients_list:
            csocket: socket.socket = cclient[0]
            csocket.close()
        print('关闭客户socket')

    # 服务器关闭事件
    def closeEvent(self, event):
        self.close_client_socket()
        super().closeEvent(event)
