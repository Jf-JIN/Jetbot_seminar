import socket
import json
import traceback
import time
import functools
from typing import Union

from PyQt5.QtCore import pyqtSignal, QThread, pyqtSlot

# 常量
DEFAULT_PORT = 10098  # 默认端口
BUFFER_SIZE = 1024  # 默认缓存大小


class Client_Console_QThread(QThread):
    signal_data_console_recv = pyqtSignal(dict)     # 接收内容信号
    # signal_data_console_send = pyqtSignal(dict)   # 发送内容信号
    signal_connected_flag = pyqtSignal(bool)        # 连接标签信号
    signal_connected_port = pyqtSignal(str)      # 端口号信号
    signal_error_output = pyqtSignal(str)            # 报错信号

    def __init__(self, ip: str) -> None:
        super().__init__()
        self.formatted_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
        print(f"[Client_Console]: [创建Socket] - {self.formatted_time}")
        self.ip = ip
        self.flag_running = True
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_address = (self.ip, DEFAULT_PORT)
            self.client_socket.connect(self.server_address)

        # [WinError 10061] 由于目标计算机积极拒绝，无法连接。
        except ConnectionRefusedError as e:
            e = traceback.format_exc()
            time.sleep(1)
            try:
                self.client_socket.connect(self.server_address)
            except:
                pass
            return None

        # [WinError 10057] 由于套接字没有连接并且(当使用一个 sendto 调用发送数据报套接字时)没有提供地址，发送或接收数据的请求没有被接受。
        except OSError as e:
            e = traceback.format_exc()
            self.client_socket.close()
            # self.__init__(self.ip)
            return None

        except Exception as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f"[Client_Console]: [初始化] - {self.formatted_time} \n{e}")
            print(f"[Client_Console]: [初始化] - {self.formatted_time} \n{e}")

    # 发送 客户端发送命令行指令
    @pyqtSlot(dict)
    def send(self, message: dict) -> None:
        try:
            self.client_socket.sendall(json.dumps(message).encode())
        except Exception as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f"[Client_Console]: [发送] - {self.formatted_time} \n{e}")
            print(f"[Client_Console]: [发送] - {self.formatted_time} \n{e}")

    # 接收 客户端接收服务器的终端显示
    @pyqtSlot()
    def receive(self) -> json:
        try:
            data = self.client_socket.recv(BUFFER_SIZE)
            if not data:
                self.signal_connected_flag.emit(False)  # 发送断开连接的信号
                return None
            data = json.loads(data.decode())
            if "JPort" in data:
                print(f"[Client_Console]: 已经与 {self.server_address} 建立连接")
                self.signal_connected_port.emit(str(data["JPort"]))
                print("[Client_Console]: 客户端端口: ", data["JPort"])
                return None
            if 'server_close' in data:
                print(f"[Client_Console]: 服务器将关闭 - {self.formatted_time}")
                self.signal_connected_flag.emit(False)
                self.flag_running = False
                return None
            return data
        except ConnectionRefusedError as e:     # [WinError 10061] 由于目标计算机积极拒绝，无法连接。
            e = traceback.format_exc()
            time.sleep(1)
            self.client_socket.connect(self.server_address)
            return None

        except (OSError, ConnectionResetError) as e:    # [WinError 10057] 由于套接字没有连接并且(当使用一个 sendto 调用发送数据报套接字时)没有提供地址，发送或接收数据的请求没有被接受。
            e = traceback.format_exc()
            self.client_socket.close()
            # self.__init__(self.ip)
            return None

        except Exception as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f"[Client_Console]: [接收] - {self.formatted_time} \n{e}")
            print(f"[Client_Console]: [接收] - {self.formatted_time} \n{e}")
            self.signal_connected_flag.emit(False)  # 发送断开连接的信号
            return None

    # 运行线程
    def run(self) -> None:
        try:
            while self.flag_running:
                data = self.receive()
                if data:
                    self.signal_connected_flag.emit(True)
                    self.signal_data_console_recv.emit(data)  # 接收数据信号
                    print("[Client_Console]: 客户端接收: ", data)
        except:
            e = traceback.format_exc()
            self.signal_error_output.emit(f"[Client_Console]: [线程run] - {self.formatted_time} \n{e}")
            print(f"[Client_Console]: [线程run] - {self.formatted_time} \n{e}")
        finally:
            print('[Client_Console]: 关闭客户端')
            self.client_socket.close()

    # 终止线程
    def stop(self):
        self.flag_running = False
        self.wait()

    # 检查线程是否正在运行
    def isRunning(self):
        return super().isRunning() and self.flag_running


class Server_Console_QThread(QThread):
    signal_data_console_recv = pyqtSignal(dict)     # 接收命令行信号
    # signal_data_console_send = pyqtSignal(object) # 发送命令行信号
    signal_connected_list = pyqtSignal(list)        # 连接设备列表信号
    # signal_connected_port = pyqtSignal(str)       # 端口信号
    signal_error_output = pyqtSignal(str)           # 报错信号
    signal_close_server = pyqtSignal(str)           # 关闭服务器信号

    def __init__(self, ip: str) -> None:
        super().__init__()
        self.formatted_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
        self.ip = ip
        self.flag_running = True
        self.connections_allow_number = 10  # 可接受连接数
        self.clients_list = []

        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 允许端口重用
            self.server_address = (self.ip, DEFAULT_PORT)
            self.server_socket.bind(self.server_address)
            self.server_socket.listen(self.connections_allow_number)
            print(f"[Server_Console]: 服务器在 {self.ip}:{DEFAULT_PORT} 等待连接...")
        except Exception as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f"[Server_Console]: [初始化] - {self.formatted_time} \n{e}")
            print(f"[Server_Console]: [初始化] - {self.formatted_time} \n{e}")

    # 发送: 服务器以json形式发送命令行显示
    def send(self, client_socket: socket.socket, message: dict) -> None:
        try:
            client_socket.sendall(json.dumps(message).encode())
        except Exception as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f"[Server_Console]: [发送] - {self.formatted_time} \n{e}")
            print(f"[Server_Console]: [发送] - {self.formatted_time} \n{e}")

    # 向所有设备发送信息
    def send_all(self, text: dict) -> None:
        print('send_all', self.clients_list)
        for client_socket, client_address in self.clients_list:
            print(client_socket, client_address)
            # client_socket: socket.socket
            self.send(client_socket, text)

    # 删除clients_list中的元素
    def clients_list_remove(self, client: list) -> None:
        print("[Server_Console]: 之前\t", self.clients_list)
        if client in self.clients_list:
            self.clients_list.remove(client)
        print("[Server_Console]: 之后\t", self.clients_list)

    # 不知道为啥，connect(信号.emit)，信号发送不出去
    # 用于向上中转信号
    def signal_extend(self, obj, signal) -> None:
        obj.emit(signal)

    # 运行线程
    def run(self):
        while self.flag_running:
            try:
                client_socket, client_address = self.server_socket.accept()
                if client_socket:
                    # 避免重复添加
                    if any(client_address[0] in item[1] for item in self.clients_list):
                        print('[Server_Console]: 重复设备')
                        continue
                    text = {"JPort": client_address[1]}         # 初始发送内容
                    self.send(client_socket, text)                                      # 发送连接端口号作为连接标志
                    self.clients_list.append([client_socket, client_address])           # 最后将客户端添加进发送列表中/客户端列表中
                    print(f"[+] [Server_Console]: 接受来自 {client_address} 的连接")
                    print('[Server_Console]: 当前已连接客户端数量:', len(self.clients_list))
                    # 接收客户端信息
                    recv_thread = Server_Handle_Console_QThread(self, client_socket, client_address)
                    recv_thread.signal_remove_clients.connect(self.clients_list_remove)         # 从客户端列表中移除已断开连接的客户端
                    recv_thread.signal_data_console_send.connect(self.send_all)                 # 向客户端发送关闭服务器的信号
                    recv_thread.signal_data_console_recv.connect(functools.partial(self.signal_extend, self.signal_data_console_recv))  # 转接客户端发来的命令行信号
                    recv_thread.signal_connected_list.connect(functools.partial(self.signal_extend, self.signal_connected_list))        # 转接已连接的客户端列表信号
                    recv_thread.signal_error_output.connect(functools.partial(self.signal_extend, self.signal_error_output))            # 转接报错信号
                    recv_thread.signal_close_server.connect(functools.partial(self.signal_extend, self.signal_close_server))            # 转接关闭服务器的信号
                    recv_thread.start()
            except Exception as e:
                e = traceback.format_exc()
                self.signal_error_output.emit(f"[Server_Console]: [线程run] - {self.formatted_time} \n{e}")
                print(f"[Server_Console]: [线程run] - {self.formatted_time} \n{e}")

    # 终止线程
    def stop(self):
        self.flag_running = False
        self.wait()

    # 检查线程是否正在运行
    def isRunning(self):
        # 覆盖 isRunning 方法
        return super().isRunning() and self.flag_running


class Server_Handle_Console_QThread(QThread):
    signal_data_console_send = pyqtSignal(dict)     # 发送服务器关闭的信号
    signal_data_console_recv = pyqtSignal(dict)     # 接收命令行信号
    # signal_connected_port = pyqtSignal(str)
    signal_connected_list = pyqtSignal(list)        # 连接设备列表信号
    signal_remove_clients = pyqtSignal(list)        # 移除断开连接的客户端信号
    signal_error_output = pyqtSignal(str)           # 报错信号
    signal_close_server = pyqtSignal(str)           # 关闭服务器信号

    def __init__(
        self, parent, client_socket: socket.socket, client_address: tuple
    ) -> None:
        super().__init__(parent)
        self.formatted_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
        self.parent_obj: QThread = parent
        self.client_socket = client_socket
        self.client_address = client_address
        self.flag_running = True

    # 接收 用于服务器接收客户端发送的close信号
    def receive(self) -> json:
        try:
            data = self.client_socket.recv(BUFFER_SIZE)
            if not data:
                return None
            return json.loads(data.decode())
        except Exception as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f"[Server_Console]: [线程子线程接收] - {self.formatted_time} \n{e}")
            print(f"[Server_Console]: [线程子线程接收] - {self.formatted_time} \n{e}")
            self.flag_running = False
            return None

    # 运行线程
    def run(self):
        try:
            while self.flag_running:
                data = self.receive()
                if not data:
                    continue
                else:
                    print(f"[*] [Server_Console]: 来自 {self.client_address} 的消息：\n{data}\n")
                    self.signal_data_console_recv.emit(data)  # 数据信号
                    self.signal_connected_list.emit(self.parent_obj.clients_list)
                    if "close" in data:
                        print("[Server_Console]: 关闭")
                        break
                    elif 'server_close' in data:
                        print("[Server_Console]: 服务器将关闭")
                        self.signal_close_server.emit('server_close')
                        close_signal = {'server_close': 'Server will be closed'}
                        self.signal_data_console_send.emit(close_signal)
                        break
        except ConnectionResetError as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f"[Server_Console]: [线程子线程run] - {self.formatted_time} \n{e}")
            print(f"[Server_Console]: [线程子线程run] - {self.formatted_time} \n{e}")
        finally:
            print('[Server_Console] 断开连接')
            self.signal_remove_clients.emit([self.client_socket, self.client_address])  # 连接关闭时,从客户端列表中移除断开连接的客户端
            self.signal_connected_list.emit(self.parent_obj.clients_list)  # 发送该客户端断开连接后的客户端列表，用于实例化服务器主线程的进程获取列表数据
            print("[Server_Console]: 当前已连接客户端数量:", len(self.parent_obj.clients_list))
            print(f"[-] [Server_Console]: 与 {self.client_address} 的连接已关闭")
            self.client_socket.close()  # 断开与客户端socket连接
