import socket
import json
import traceback
import time
import functools
from typing import Union
import numpy as np
import struct

from PyQt5.QtCore import pyqtSignal, QThread, pyqtSlot
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QByteArray

# 常量
DEFAULT_PORT = 10086    # 默认端口
BUFFER_SIZE = 1024      # 默认缓存大小
TRANSMISSION_FORMAT = '!Q'


class Client_Video_QThread(QThread):
    signal_data_video_recv = pyqtSignal(QPixmap)     # 接收图片/视频信号
    # signal_data_send = pyqtSignal(object)           # 发送内容信号
    signal_connected_flag = pyqtSignal(bool)        # 连接标签信号
    signal_connected_port = pyqtSignal(str)         # 端口号信号
    signal_error_output = pyqtSignal(str)           # 报错信号
    signal_server_close = pyqtSignal()              # 服务端将关闭的信号

    def __init__(self, ip: str) -> None:
        super().__init__()
        self.formatted_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
        print(f"[Client_Video]: [创建Socket] - {self.formatted_time}")
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
            return None

        except Exception as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f"[Client_Video]: [初始化] - {self.formatted_time} \n{e}")
            print(f"[Client_Video]: [初始化] - {self.formatted_time} \n{e}")

    # 发送 客户端发送关闭信号
    @pyqtSlot(dict)
    def send(self, message: dict) -> None:
        try:
            self.client_socket.sendall(json.dumps(message).encode())
        except Exception as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f"[Client_Video]: [发送] - {self.formatted_time} \n{e}")
            print(f"[Client_Video]: [发送] - {self.formatted_time} \n{e}")

    # 接收 客户端接收服务器发送的视频数据或者文本数据
    @pyqtSlot()
    def receive(self) -> QPixmap:
        try:
            data = self.client_socket.recv(BUFFER_SIZE)
            if not data:
                self.signal_connected_flag.emit(False)  # 发送断开连接的信号
                return None
            # 区分两种数据类型，非json数据则是视频数据，json数据为正常文字数据
            try:
                data = json.loads(data.decode())
                if 'JPort' in data:
                    print(f"[Client_Video]: 已经与 {self.server_address} 建立连接")
                    self.signal_connected_port.emit(str(data['JPort']))
                    print("[Client_Video]: 客户端端口: ", data["JPort"])
                    return None
                if 'server_close' in data:
                    print('[Client_Console-server_close]:', data)
                    print(f"[Client_Video]: 服务器将关闭 - {self.formatted_time}")
                    self.signal_server_close.emit()
                    return None
            except (UnicodeDecodeError, json.JSONDecodeError):
                pass
            try:
                if len(data) < struct.calcsize(TRANSMISSION_FORMAT):
                    # 如果数据长度小于传输格式长度，则继续接收数据直到达到长度
                    additional_data = self.client_socket.recv(struct.calcsize(TRANSMISSION_FORMAT) - len(data))
                    data += additional_data
                size = struct.unpack(TRANSMISSION_FORMAT, data[:struct.calcsize(TRANSMISSION_FORMAT)])[0]
                data = data[struct.calcsize(TRANSMISSION_FORMAT):]
                # 接收图像数据，组接字节流数据
                data_bytes_img = data
                while len(data_bytes_img) < size:
                    packet = self.client_socket.recv(size - len(data_bytes_img))
                    if not packet:
                        break
                    data_bytes_img += packet
                pixmap = self.bytes_to_pixmap(data_bytes_img)  # 将二进制图像数据转换为pixmap，用于label显示
                return pixmap
            except struct.error:
                e = traceback.format_exc()
                self.signal_error_output.emit(f"[Client_Video]: [接收][数据解包错误] - {self.formatted_time} \n{e}")
                print(f"[Client_Video]: [接收][数据解包错误] - {self.formatted_time} \n{e}")
        except ConnectionRefusedError as e:     # [WinError 10061] 由于目标计算机积极拒绝，无法连接。
            e = traceback.format_exc()
            time.sleep(1)
            self.client_socket.connect(self.server_address)
            return None

        except (OSError, ConnectionResetError) as e:    # [WinError 10057] 由于套接字没有连接并且(当使用一个 sendto 调用发送数据报套接字时)没有提供地址，发送或接收数据的请求没有被接受。
            e = traceback.format_exc()
            self.flag_running = False
            # self.__init__(self.ip)
            return None

        except Exception as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f"[Client_Video]: [接收] - {self.formatted_time} \n{e}")
            print(f"[Client_Video]: [接收] - {self.formatted_time} \n{e}")
            self.flag_running = False
            self.signal_connected_flag.emit(False)  # 发送断开连接的信号
            return None

    # 将二进制图像数据转换为pixmap
    def bytes_to_pixmap(self, data_bytes_img) -> QPixmap:
        try:
            np_data = np.frombuffer(data_bytes_img, np.uint8)
            byte_array = QByteArray(bytes(np_data))
            image = QImage()
            image.loadFromData(byte_array)
            pixmap = QPixmap.fromImage(image)
            return pixmap
        except Exception as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f"[Client_Video]: [二进制转pixmap] - {self.formatted_time} \n{e}")
            print(f"[Client_Video]: [二进制转pixmap] - {self.formatted_time} \n{e}")

    # 运行线程
    def run(self) -> None:
        try:
            while self.flag_running:
                data = self.receive()
                if data:
                    self.signal_connected_flag.emit(True)
                    self.signal_data_video_recv.emit(data)    # 接收数据信号
                    print('[Client_Video]: 客户端接收：', data)
        except:
            e = traceback.format_exc()
            self.signal_error_output.emit(f"[Client_Video]: [线程run] - {self.formatted_time} \n{e}")
            print(f"[Client_Video]: [线程run] - {self.formatted_time} \n{e}")
        finally:
            print('[Client_Video]: 关闭客户端')
            self.client_socket.close()

    # 终止线程
    def stop(self) -> None:
        self.flag_running = False
        try:
            self.client_socket.shutdown(socket.SHUT_RDWR)  # 关闭 socket 读写操作
        except Exception as e:
            print(f"[Client_Console]: [停止] - {self.formatted_time} 关闭 socket 出错: {e}")
        self.quit()

    # 检查线程是否正在运行
    def isRunning(self) -> bool:
        return super().isRunning() and self.flag_running


class Server_Vedio_QThread(QThread):
    signal_data_video_recv = pyqtSignal(dict)             # 接收内容信号
    signal_connected_list = pyqtSignal(list)        # 连接设备列表信号
    signal_error_output = pyqtSignal(str)          # 报错信号
    signal_final_close = pyqtSignal()           # 关闭服务器信号

    def __init__(self, ip: str) -> None:
        super().__init__()
        self.formatted_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
        self.ip = ip
        self.flag_running = True
        self.connections_allow_number = 10  # 可接受连接数
        self.clients_list = []
        self.flag_close_server_by_client = False

        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 允许端口重用
            self.server_address = (self.ip, DEFAULT_PORT)
            self.server_socket.bind(self.server_address)
            self.server_socket.listen(self.connections_allow_number)
            print(f"[Server_Video]: 服务器在 {self.ip}:{DEFAULT_PORT} 等待连接...")
        except Exception as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f"[Server_Video]: [初始化] - {self.formatted_time} \n{e}")
            print(f"[Server_Video]: [初始化] - {self.formatted_time} \n{e}")

    # 发送 服务器向客户端发送端口号
    def send(self, client_socket: socket.socket, message: dict) -> None:
        try:
            client_socket.sendall(json.dumps(message).encode())
        except Exception as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f"[Server_Video]: [发送] - {self.formatted_time} \n{e}")
            print(f"[Server_Video]: [发送] - {self.formatted_time} \n{e}")

    # 向所有设备发送信息
    def send_all(self, message: dict) -> None:
        for client_socket, _ in self.clients_list:
            client_socket: socket.socket
            self.send(client_socket, message)

    # 发送 服务器向客户端发送图片数据，此处传进来的是图片二进制数据，发送的是打包的二进制数据
    def send_img(self, client_socket: socket.socket, data: list) -> None:
        try:
            data_length: int = len(data)
            # 定义数据长度
            size: bytes = struct.pack(TRANSMISSION_FORMAT, data_length)
            # 发送图像大小
            client_socket.sendall(size)
            # 发送图像数据
            client_socket.sendall(bytearray(data))
        except Exception as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f"[Server_Video]: [发送图片] - {self.formatted_time} \n{e}")
            print(f"[Server_Video]: [发送图片] - {self.formatted_time} \n{e}")

    # 向所有设备发送信息
    def send_img_all(self, img_info: list) -> None:
        for client_socket, _ in self.clients_list:
            client_socket: socket.socket
            self.send_img(client_socket, img_info)

    # 删除clients_list中的元素
    def clients_list_remove(self, client: list) -> None:
        print('[Server_Video]: 之前\t', self.clients_list)
        if client in self.clients_list:
            self.clients_list.remove(client)
        print('[Server_Video]: 之后\t', self.clients_list)

    # 不知道为啥，connect(信号.emit)，信号发送不出去
    # 用于向上中转信号
    def signal_extend(self, obj, *args) -> None:
        obj.emit(*args)

    # 向所有客户端广播，服务器关闭，断开与服务器的连接，当没有客户端与服务器相连时，服务器执行关闭程序
    def close_server_from_clients_command_send(self):
        # print('[close_server_from_clients_command_send] ', len(self.clients_list), self.clients_list)
        close_signal: dict[str, str] = {'server_close': 'Server will be closed'}
        self.send_all(close_signal)
        self.flag_close_server_by_client = True

    # 运行线程

    def run(self) -> None:
        while self.flag_running:
            try:
                client_socket, client_address = self.server_socket.accept()
                if client_socket:
                    # 避免重复连接相同的设备
                    if any(client_address[0] in item[1] for item in self.clients_list):
                        print('[Server_Video]: 重复设备')
                        continue
                    text = {"JPort": client_address[1]}         # 初始发送内容
                    self.send(client_socket, text)                                      # 发送连接端口号作为连接标志
                    self.clients_list.append([client_socket, client_address])           # 最后将客户端添加进发送列表中/客户端列表中
                    print(f"[+] [Server_Video]: 接受来自 {client_address} 的连接")
                    print('[Server_Video]: 当前已连接客户端数量:', len(self.clients_list))
                    # 接收客户端信息
                    recv_thread = Server_Handle_Video_QThread(self, client_socket, client_address)
                    recv_thread.signal_remove_clients.connect(self.clients_list_remove)         # 接收客户端断开后，从列表中移除的信号
                    recv_thread.signal_data_video_recv.connect(functools.partial(self.signal_extend, self.signal_data_video_recv))              # 转接客户端发来的命令行信号
                    recv_thread.signal_connected_list.connect(functools.partial(self.signal_extend, self.signal_connected_list))    # 转接已连接的客户端列表信号
                    recv_thread.signal_error_output.connect(functools.partial(self.signal_extend, self.signal_error_output))        # 转接报错信号
                    recv_thread.signal_final_close.connect(functools.partial(self.signal_extend, self.signal_final_close))
                    recv_thread.signal_close_server_form_client.connect(self.close_server_from_clients_command_send)            # 处理由客户端发起的服务器关闭请求信号

                    recv_thread.start()
            except Exception as e:
                e = traceback.format_exc()
                self.signal_error_output.emit(f"[Server_Video]: [线程run] - {self.formatted_time} \n{e}")
                print(f"[Server_Video]: [线程run] - {self.formatted_time} \n{e}")

    # 终止线程
    def stop(self) -> None:
        self.flag_running = False
        self.wait()

    # 检查线程是否正在运行
    def isRunning(self) -> bool:
        return super().isRunning() and self.flag_running


class Server_Handle_Video_QThread(QThread):
    signal_data_video_recv = pyqtSignal(dict)         # 接收命令行信号
    # signal_connected_port = pyqtSignal(str)
    signal_connected_list = pyqtSignal(list)    # 连接设备列表信号
    signal_remove_clients = pyqtSignal(list)    # 移除断开连接的客户端信号
    signal_error_output = pyqtSignal(str)       # 报错信号
    signal_close_server_form_client = pyqtSignal()           # 关闭服务器信号
    signal_final_close = pyqtSignal()

    def __init__(self, parent, client_socket: socket.socket, client_address: tuple) -> None:
        super().__init__(parent)
        self.formatted_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
        self.parent_obj: QThread = parent               # 父类是服务器主线程
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
            self.signal_error_output.emit(f"[Server_Video]: [线程子线程接收] - {self.formatted_time} \n{e}")
            print(f"[Server_Video]: [线程子线程接收] - {self.formatted_time} \n{e}")
            self.flag_running = False
            return None

    # 运行线程
    def run(self) -> None:
        try:
            while self.flag_running:
                data = self.receive()
                if not data:
                    continue
                else:
                    print(f"[*] [Server_Video]: 来自 {self.client_address} 的消息：\n{data}\n")
                    self.signal_data_video_recv.emit(data)    # 数据信号
                    self.signal_connected_list.emit(self.parent_obj.clients_list)
                    if 'close' in data:
                        print('[Server_Video]: 关闭')
                        break
                    elif 'server_close' in data:
                        print(f"[Server_Video]: 由 {self.client_address} 发出关闭服务器的请求 - {self.formatted_time}")
                        self.signal_close_server_form_client.emit()  # 向服务器主线程发送信号，向所有客户端广播
                        print("[Server_Video]: 服务器请求所有客户端断开连接")

        except ConnectionResetError as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f"[Server_Video]: [线程子线程run] - {self.formatted_time} \n{e}")
            print(f"[Server_Video]: [线程子线程run] - {self.formatted_time} \n{e}")
        finally:
            self.signal_remove_clients.emit([self.client_socket, self.client_address])  # 连接关闭时,从客户端列表中移除断开连接的客户端
            self.signal_connected_list.emit(self.parent_obj.clients_list)  # 发送该客户端断开连接后的客户端列表，用于实例化服务器主线程的进程获取列表数据
            print('[Server_Video]: 当前已连接客户端数量:', len(self.parent_obj.clients_list))
            print(f"[-] [Server_Video]: 与 {self.client_address} 的连接已关闭")
            print('[parent_obj.clients_list]', len(self.parent_obj.clients_list))
            if len(self.parent_obj.clients_list) == 1 and self.parent_obj.flag_close_server_by_client:
                self.signal_final_close.emit()
            self.client_socket.close()       # 断开与客户端socket连接
