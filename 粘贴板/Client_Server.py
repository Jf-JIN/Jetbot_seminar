
import socket
import json
import traceback
import time
import functools

from PyQt5.QtCore import pyqtSignal, QThread, pyqtSlot

# 常量
DEFAULT_PORT = 10010    # 默认端口
BUFFER_SIZE = 1024      # 默认缓存大小


class Client_QThread(QThread):
    data_recv_signal = pyqtSignal(dict)
    data_send_signal = pyqtSignal(object)
    connected_flag_signal = pyqtSignal(bool)
    error_output_signal = pyqtSignal(str)

    def __init__(self, ip: str) -> None:
        super().__init__()
        self.ip = ip
        self.running_flag = True
        try:
            self.client_socket = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM)
            self.server_address = (self.ip, DEFAULT_PORT)
            self.client_socket.connect(self.server_address)
        except Exception as e:
            e = traceback.format_exc()
            self.error_output_signal.emit(str(e))
            print([time.time()], e)

    # 发送：    输入发送方的socket对象
    @pyqtSlot(object)
    def send(self, message: dict | str) -> None:
        try:
            if isinstance(message, (str, int, float)):
                message = {'content': message}
            self.client_socket.sendall(json.dumps(message).encode())
        except Exception as e:
            e = traceback.format_exc()
            self.error_output_signal.emit(str(e))
            print([time.time()], e)

    # 接收：    输入接受方的socket对象
    @pyqtSlot()
    def receive(self) -> json:
        try:
            data = self.client_socket.recv(BUFFER_SIZE)
            if not data:
                self.connected_flag_signal.emit(False)  # 发送断开连接的信号
                return None

            return json.loads(data.decode())
        except Exception as e:
            self.connected_flag_signal.emit(False)  # 发送断开连接的信号
            e = traceback.format_exc()
            self.error_output_signal.emit(str(e))
            print(e)
            return None

    def run(self):
        try:
            while self.running_flag:
                data = self.receive()
                if data:
                    self.connected_flag_signal.emit(True)
                    self.data_recv_signal.emit(data)    # 接收数据信号
                    print('客户端接收：', data)
        except ConnectionResetError as e:
            print(f"连接被重置: {e}")
            self.error_output_signal.emit(e)
        finally:
            self.client_socket.close()

    def stop(self):
        self.running_flag = False
        self.wait()

    def isRunning(self):
        return super().isRunning() and self.running_flag


class Server_QThread(QThread):
    client_port_signal = pyqtSignal(str)
    data_recv_signal = pyqtSignal(dict)
    data_send_signal = pyqtSignal(object)
    connected_flag_signal = pyqtSignal(bool)
    error_output_signal = pyqtSignal(str)

    def __init__(self, ip: str) -> None:
        super().__init__()
        self.ip = ip
        self.running_flag = True
        self.connections_allow_number = 10  # 可接受连接数
        self.clients_list = []

        try:
            self.server_socket = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(
                socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 允许端口重用
            self.server_address = (self.ip, DEFAULT_PORT)
            self.server_socket.bind(self.server_address)
            self.server_socket.listen(self.connections_allow_number)
            print(f"服务器在 {self.ip}:{DEFAULT_PORT} 等待连接...")
        except Exception as e:
            e = traceback.format_exc()
            print([time.time()], e)
            self.error_output_signal.emit(e)

    # 发送：    输入发送方的socket对象
    def send(self, client_socket: socket.socket, message: dict) -> None:
        try:
            if isinstance(message, (str, int, float)):
                message = {'content': message}
            client_socket.sendall(json.dumps(message).encode())
        except Exception as e:
            self.error_output_signal.emit(str(e))

    # 向所有设备发送信息
    def send_all(self, text: dict):
        print('send_all')
        for client_socket, _ in self.clients_list:
            client_socket: socket.socket
            self.send(client_socket, text)

    # 删除clients_list中的元素
    def clients_list_remove(self, client: list[str, tuple]):
        self.clients_list.remove(client)

    # 不知道为啥，connect(信号.emit)，信号发送不出去
    # 用于向上中转信号
    def signal_extend(self, obj, signal):
        obj.emit(signal)

    def run(self):
        while self.running_flag:
            try:
                client_socket, client_address = self.server_socket.accept()
                if client_socket:
                    # 避免重复添加
                    if any(client_address[1] in item for item in self.clients_list):
                        continue
                    self.clients_list.append([client_socket, client_address])
                    # 接收客户端
                    recv_thread = Server_Handle_QThread(
                        self, client_socket, client_address)
                    recv_thread.data_recv_signal.connect(
                        functools.partial(self.signal_extend, self.data_recv_signal))
                    recv_thread.data_send_signal.connect(self.send_all)
                    recv_thread.remove_clients_signal.connect(
                        self.clients_list_remove)
                    recv_thread.connected_flag_signal.connect(
                        functools.partial(self.signal_extend, self.connected_flag_signal))
                    recv_thread.error_output_signal.connect(
                        functools.partial(self.signal_extend, self.error_output_signal))
                    recv_thread.start()
            except Exception as e:
                print("Error in handling client:", e)

    def stop(self):
        self.running_flag = False
        self.wait()

    def isRunning(self):
        # 覆盖 isRunning 方法
        return super().isRunning() and self.running_flag


class Server_Handle_QThread(QThread):
    data_send_signal = pyqtSignal(object)
    data_recv_signal = pyqtSignal(dict)
    connected_flag_signal = pyqtSignal(bool)
    remove_clients_signal = pyqtSignal(list)
    error_output_signal = pyqtSignal(str)

    def __init__(self, parent, client_socket: socket.socket, client_address: tuple) -> None:
        super().__init__(parent)
        self.parent_obj = parent
        self.client_socket = client_socket
        self.client_address = client_address

    # 接收：    输入接受方的socket对象
    def receive(self) -> json:
        try:
            data = self.client_socket.recv(BUFFER_SIZE)
            if not data:
                self.remove_clients_signal.emit(
                    [self.client_socket, self.client_address])  # 连接关闭时减少连接数量
                self.connected_flag_signal.emit(False)  # 发送断开连接的信号
                return None
            return json.loads(data.decode())
        except Exception as e:
            self.connected_flag_signal.emit(False)  # 发送断开连接的信号
            e = traceback.format_exc()
            self.error_output_signal.emit(str(e))
            print(e)
            return None

    def run(self):
        text = {"Jetbot_Port": self.client_address[1]}       # 初始发送内容
        self.data_send_signal.emit(text)
        self.connected_flag_signal.emit(True)
        print(f"[+] 接受来自 {self.client_address} 的连接")
        print('当前已连接客户端数量:', len(self.parent_obj.clients_list))
        self.connected_flag_signal.emit(True)
        try:
            while True:
                data = self.receive()
                if not data:
                    break
                else:
                    print(f"[*] 来自 {self.client_address} 的消息：\n{data}\n")
                    self.data_recv_signal.emit(data)    # 数据信号
                    self.connected_flag_signal.emit(True)
                    if 'close' in data:
                        print('关闭')
                        break
        except ConnectionResetError as e:
            print(f"连接被重置: {e}")
            self.error_output_signal.emit(e)
        finally:
            self.remove_clients_signal.emit(
                [self.client_socket, self.client_address])    # 移除客户端从列表中
            self.connected_flag_signal.emit(False)
            print('当前已连接客户端数量:', len(self.parent_obj.clients_list))
            print(f"[-] 与 {self.client_address} 的连接已关闭")
            self.client_socket.close()       # 断开与客户端socket连接
