import socket
import json
import traceback
import time
import functools
from typing import Union

from PyQt5.QtCore import pyqtSignal, QThread, pyqtSlot

# 常量
DEFAULT_PORT = 10098  # 默认端口
BUFFER_SIZE = 2048  # 默认缓存大小
SLICE_SIZE = 4


class Client_Console_QThread(QThread):
    signal_data_console_recv = pyqtSignal(dict)     # 接收内容信号
    signal_connected_flag = pyqtSignal(bool)        # 连接标签信号
    signal_connected_port = pyqtSignal(str)      # 端口号信号
    signal_error_output = pyqtSignal(str)            # 报错信号
    signal_server_close = pyqtSignal()              # 服务端将关闭的信号

    def __init__(self, ip: str) -> None:
        super().__init__()
        self.formatted_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
        print(f'\n[客户端_文本端口] [创建Socket] - {self.formatted_time}')
        self.ip = ip
        self.flag_running = True
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_address = (self.ip, DEFAULT_PORT)
            self.client_socket.connect(self.server_address)

        # [WinError 10061] 由于目标计算机积极拒绝, 无法连接。
        except ConnectionRefusedError as e:
            e = traceback.format_exc()
            time.sleep(1)
            try:
                self.client_socket.connect(self.server_address)
            except:
                pass
            return None

        # [WinError 10057] 由于套接字没有连接并且(当使用一个 sendto 调用发送数据报套接字时)没有提供地址, 发送或接收数据的请求没有被接受。
        except OSError as e:
            e = traceback.format_exc()
            self.client_socket.close()
            return None

        except Exception as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f'\n[客户端_文本端口] [初始化] - {self.formatted_time} \n[!错误!] {e}')
            print(f'\n[客户端_文本端口] [初始化] - {self.formatted_time} \n[!错误!] {e}')

    # 发送 客户端发送命令行指令
    @pyqtSlot(dict)
    def send(self, message: dict) -> None:
        try:
            serialized_data = json.dumps(message)
            # print('\n[serialized_data]\t', serialized_data)
            total_bytes = len(serialized_data)
            num_chunks = total_bytes // BUFFER_SIZE + 1
            # print('\n[total_bytes]\t', total_bytes)
            # print('\n[num_chunks]\t', num_chunks)
            self.client_socket.sendall(num_chunks.to_bytes(SLICE_SIZE, byteorder='big'))
            for i in range(num_chunks):
                start_idx = i * BUFFER_SIZE
                end_idx = min((i + 1) * BUFFER_SIZE, total_bytes)
                chunk = serialized_data[start_idx:end_idx]
                self.client_socket.sendall(chunk.encode())
        except Exception as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f'\n[客户端_文本端口] [发送] - {self.formatted_time} \n[!错误!] {e}')
            print(f'\n[客户端_文本端口] [发送] - {self.formatted_time} \n[!错误!] {e}')

    # 接收 客户端接收服务器的终端显示
    @pyqtSlot()
    def receive_decode(self) -> json:
        try:
            data = self.client_socket.recv(BUFFER_SIZE)
            if not data:
                self.signal_connected_flag.emit(False)  # 发送断开连接的信号
                return None
            # print(data)
            data = data.decode()
            return data

        except ConnectionRefusedError as e:     # [WinError 10061] 由于目标计算机积极拒绝, 无法连接。
            e = traceback.format_exc()
            time.sleep(1)
            self.client_socket.connect(self.server_address)
            return None

        except (OSError, ConnectionResetError) as e:    # [WinError 10057] 由于套接字没有连接并且(当使用一个 sendto 调用发送数据报套接字时)没有提供地址, 发送或接收数据的请求没有被接受。
            e = traceback.format_exc()
            self.client_socket.close()
            # self.__init__(self.ip)
            return None

        except Exception as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f'\n[客户端_文本端口] [接收] - {self.formatted_time} \n[!错误!] {e}')
            print(f'\n[客户端_文本端口] [接收] - {self.formatted_time} \n[!错误!] {e}')
            self.flag_running = False
            self.signal_connected_flag.emit(False)  # 发送断开连接的信号
            return None

    # 读取并分析数据
    def read_and_analyse_data(self, data: dict):
        try:
            # print(repr(data))
            data = json.loads(data)
            if 'JPort' in data:
                print(f'\n[客户端_文本端口]: 与 {self.server_address} 成功建立连接')
                self.signal_connected_port.emit(str(data['JPort']))
                print(f'\n[客户端_文本端口]: 客户端端口: {data["JPort"]}')
                return
            if 'server_close' in data:
                # print('\n[客户端_文本端口-server_close]:', data)
                print(f'\n[客户端_文本端口]: 请求关闭服务器 - {self.formatted_time}')
                self.signal_server_close.emit()     # 发送服务器关闭时, 与服务器断开连接的信号
                return
            self.send_analysed_data_signal(data)
        except:
            e = traceback.format_exc()
            self.signal_error_output.emit(f'\n[客户端_文本端口] [读取数据包] - {self.formatted_time} \n[!错误!] {e}')
            print(f'\n[客户端_文本端口] [读取数据包] - {self.formatted_time} \n[!错误!] {e}')

    # 发送读取后的内容信号
    def send_analysed_data_signal(self, data):
        self.signal_connected_flag.emit(True)
        self.signal_data_console_recv.emit(data)  # 接收数据信号
        # print('\n[客户端_文本端口] [客户端接收]: ', data)

    # 运行线程
    def run(self) -> None:
        try:
            while self.flag_running:
                data_decode: str = self.receive_decode()
                if data_decode:
                    for i in ['\x00', '\x00', '\x00', '\x01']:      # 莫名其妙的符号，不必管
                        if i in data_decode:
                            data_decode = data_decode.replace(i, '')
                    if not data_decode:
                        continue
                    list_data = data_decode.split('}{')
                    for index, item in enumerate(list_data):
                        if len(list_data) == 1:
                            pass
                        elif index == 0:
                            item = item + '}'
                        elif index == len(list_data)-1:
                            item = '{' + item
                        else:
                            item = '{' + item + '}'
                        self.read_and_analyse_data(item)
        except:
            e = traceback.format_exc()
            self.signal_error_output.emit(f'\n[客户端_文本端口] [线程运行] - {self.formatted_time} \n[!错误!] {e}')
            print(f'\n[客户端_文本端口] [线程运行] - {self.formatted_time} \n[!错误!] {e}')
        finally:
            print('\n[客户端_文本端口]: 关闭客户端')
            self.client_socket.close()

    # 终止线程
    def stop(self):
        self.flag_running = False
        try:
            self.client_socket.shutdown(socket.SHUT_RDWR)  # 关闭 socket 读写操作
        except Exception as e:
            print(f'\n[客户端_文本端口] [停止] - {self.formatted_time} 关闭 socket 出错: {e}')
        self.quit()

    # 检查线程是否正在运行
    def isRunning(self):
        return super().isRunning() and self.flag_running


class Server_Console_QThread(QThread):
    signal_data_console_recv = pyqtSignal(dict)     # 接收命令行信号
    signal_connected_list = pyqtSignal(list)        # 连接设备列表信号
    signal_error_output = pyqtSignal(str)           # 报错信号
    signal_final_close = pyqtSignal()           # 关闭服务器信号

    def __init__(self, ip: str) -> None:
        super().__init__()
        self.formatted_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
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
            print(f'\n[服务器_文本端口]: 服务器在 {self.ip}:{DEFAULT_PORT} 等待连接...')
        except Exception as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f'\n[服务器_文本端口] [初始化] - {self.formatted_time} \n[!错误!] {e}')
            print(f'\n[服务器_文本端口] [初始化] - {self.formatted_time} \n[!错误!] {e}')

    # 发送: 服务器以json形式发送命令行显示
    # def send(self, client_socket: socket.socket, message: dict) -> None:
    #     try:
    #         print(message)
    #         client_socket.sendall(json.dumps(message).encode())
    def send(self, client_socket: socket.socket, message: dict) -> None:
        try:
            serialized_data = json.dumps(message)
            total_bytes = len(serialized_data)
            num_chunks = total_bytes // BUFFER_SIZE + 1
            client_socket.sendall(num_chunks.to_bytes(SLICE_SIZE, byteorder='big'))
            for i in range(num_chunks):
                start_idx = i * BUFFER_SIZE
                end_idx = min((i + 1) * BUFFER_SIZE, total_bytes)
                chunk = serialized_data[start_idx:end_idx]
                client_socket.sendall(chunk.encode())
        except Exception as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f'\n[服务器_文本端口] [发送] - {self.formatted_time} \n[!错误!] {e}')
            print(f'\n[服务器_文本端口] [发送] - {self.formatted_time} \n[!错误!] {e}')

    # 向所有设备发送信息
    @ pyqtSlot(dict)
    def send_all(self, text: dict) -> None:
        # print('send_all', text, self.clients_list)
        for client_socket, client_address in self.clients_list:
            self.send(client_socket, text)

    # 删除clients_list中的元素
    def clients_list_remove(self, client: list) -> None:
        # print('\n[服务器_文本端口]: 之前\t', self.clients_list)
        if client in self.clients_list:
            self.clients_list.remove(client)
        # print('\n[服务器_文本端口]: 之后\t', self.clients_list)
        print('\n[服务器_文本端口] [当前客户端列表]\t', self.clients_list)

    # 不知道为啥, connect(信号.emit), 信号发送不出去
    # 用于向上中转信号
    def signal_extend(self, obj, *args) -> None:
        obj.emit(*args)

    # 向所有客户端广播, 服务器关闭, 断开与服务器的连接, 当没有客户端与服务器相连时, 服务器执行关闭程序
    def close_server_by_clients_command_send(self):
        # print('\n[close_server_by_clients_command_send] ', len(self.clients_list), self.clients_list)
        close_signal: dict[str, str] = {'server_close': 'Server will be closed'}
        self.send_all(close_signal)
        self.flag_close_server_by_client = True

    # 运行线程
    def run(self):
        while self.flag_running:
            try:
                client_socket, client_address = self.server_socket.accept()
                if client_socket:
                    # 避免重复添加
                    if any(client_address[0] in item[1] for item in self.clients_list):
                        print('\n[服务器_文本端口]: 重复设备, 拒绝连接')
                        continue
                    text = {'JPort': client_address[1]}         # 初始发送内容
                    self.send(client_socket, text)                                      # 发送连接端口号作为连接标志
                    self.clients_list.append([client_socket, client_address])           # 最后将客户端添加进发送列表中/客户端列表中
                    print(f'\n[服务器_文本端口] [+]: 接受来自 {client_address} 的连接')
                    print(f'\n[服务器_文本端口]: 当前已连接客户端数量: {len(self.clients_list)}')
                    # 接收客户端信息
                    recv_thread = Server_Handle_Console_QThread(self, client_socket, client_address)
                    recv_thread.signal_remove_clients.connect(self.clients_list_remove)         # 从客户端列表中移除已断开连接的客户端
                    recv_thread.signal_data_console_recv.connect(functools.partial(self.signal_extend, self.signal_data_console_recv))  # 转接客户端发来的命令行信号
                    recv_thread.signal_connected_list.connect(functools.partial(self.signal_extend, self.signal_connected_list))        # 转接已连接的客户端列表信号
                    recv_thread.signal_error_output.connect(functools.partial(self.signal_extend, self.signal_error_output))            # 转接报错信号
                    recv_thread.signal_final_close.connect(functools.partial(self.signal_extend, self.signal_final_close))
                    recv_thread.signal_close_server_form_client.connect(self.close_server_by_clients_command_send)            # 处理由客户端发起的服务器关闭请求信号
                    recv_thread.start()
            except Exception as e:
                e = traceback.format_exc()
                self.signal_error_output.emit(f'\n[服务器_文本端口] [线程运行] - {self.formatted_time} \n[!错误!] {e}')
                print(f'\n[服务器_文本端口] [线程运行] - {self.formatted_time} \n[!错误!] {e}')

    # 终止线程
    def stop(self):
        self.flag_running = False
        self.wait()

    # 检查线程是否正在运行
    def isRunning(self):
        # 覆盖 isRunning 方法
        return super().isRunning() and self.flag_running


class Server_Handle_Console_QThread(QThread):
    signal_data_console_recv = pyqtSignal(dict)     # 接收命令行信号
    signal_connected_list = pyqtSignal(list)        # 连接设备列表信号
    signal_remove_clients = pyqtSignal(list)        # 移除断开连接的客户端信号
    signal_error_output = pyqtSignal(str)           # 报错信号
    signal_close_server_form_client = pyqtSignal()           # 关闭服务器信号
    signal_final_close = pyqtSignal()               # 最后一个客户端返回的信号, 关闭服务器

    def __init__(self, parent, client_socket: socket.socket, client_address: tuple) -> None:
        super().__init__(parent)
        self.formatted_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
        self.parent_obj: QThread = parent
        self.client_socket = client_socket
        self.client_address = client_address
        self.flag_running = True

    # 接收客户端发来的命令
    def receive_decode(self) -> json:
        try:
            num_chunks_bytes = self.client_socket.recv(SLICE_SIZE)
            num_chunks = int.from_bytes(num_chunks_bytes, byteorder='big')
            if num_chunks > 1:
                num_chunks += 1
            # print('\n[num_chunks_bytes]\t', num_chunks_bytes)
            # print('\n[num_chunks]\t', num_chunks)
            data = ''
            for _ in range(num_chunks):
                chunk = self.client_socket.recv(BUFFER_SIZE).decode()
                data += chunk
            #     print('\n[chunk]\t', chunk)
            # print('\n[data]\t', data)
            if not data:
                return None
            return data
        except Exception as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f'\n[服务器_文本端口] [线程子线程接收] - {self.formatted_time} \n[!错误!] {e}')
            print(f'\n[服务器_文本端口] [线程子线程接收] - {self.formatted_time} \n[!错误!] {e}')
            self.flag_running = False
            return None

    def read_and_analyse_data(self, data: dict):
        try:
            print('\n[read_and_analyse_data]', type(data), repr(data))
            data = json.loads(data)
            if 'close' in data:
                print(f'\n[服务器_文本端口]: 客户端 {self.client_address} 请求断开连接')
                self.flag_running = False
            elif 'server_close' in data:
                print(f'\n[服务器_文本端口]: 由 {self.client_address} 发出关闭服务器的请求 - {self.formatted_time}')
                self.signal_close_server_form_client.emit()  # 向服务器主线程发送信号, 向所有客户端广播
                print('\n[服务器_文本端口]: 服务器请求所有客户端断开连接')
            # print('read_and_analyse_data', data)
            self.send_analysed_data_signal(data)
        except:
            e = traceback.format_exc()
            self.signal_error_output.emit(f'\n[服务器_文本端口] [读取数据包] - {self.formatted_time} \n[!错误!] {e}')
            print(f'\n[服务器_文本端口] [读取数据包] - {self.formatted_time} \n[!错误!] {e}')

    def send_analysed_data_signal(self, data):
        self.signal_data_console_recv.emit(data)  # 接收数据信号
        # print('\n[服务器_文本端口]: 客户端接收: ', data)

    # 运行线程

    def run(self) -> None:
        try:
            while self.flag_running:
                data_decode = self.receive_decode()
                print(data_decode)
                if data_decode:
                    for i in ['\x00', '\x00', '\x00', '\x01']:      # 莫名其妙的符号，不必管
                        if i in data_decode:
                            data_decode = data_decode.replace(i, '')
                    if not data_decode:
                        continue
                    list_data = data_decode.split('}{')
                    for index, item in enumerate(list_data):
                        if len(list_data) == 1:
                            pass
                        elif index == 0:
                            item = item + '}'
                        elif index == len(list_data)-1:
                            item = '{' + item
                        else:
                            item = '{' + item + '}'
                        self.read_and_analyse_data(item)

        except ConnectionResetError as e:
            e = traceback.format_exc()
            self.signal_error_output.emit(f'\n[服务器_文本端口] [线程子线程运行] - {self.formatted_time} \n[!错误!] {e}')
            print(f'\n[服务器_文本端口] [线程子线程运行] - {self.formatted_time} \n[!错误!] {e}')
        finally:
            print(f'\n[服务器_文本端口] 客户端 {self.client_address} 已断开连接')
            self.signal_remove_clients.emit([self.client_socket, self.client_address])  # 连接关闭时,从客户端列表中移除断开连接的客户端
            self.signal_connected_list.emit(self.parent_obj.clients_list)  # 发送该客户端断开连接后的客户端列表, 用于实例化服务器主线程的进程获取列表数据
            print('\n[服务器_文本端口]: 当前已连接客户端数量:', len(self.parent_obj.clients_list))
            print(f'\n[服务器_文本端口] [-]: 与 {self.client_address} 的连接已关闭')
            if self.parent_obj.flag_close_server_by_client:
                self.signal_final_close.emit()
            self.client_socket.close()  # 断开与客户端socket连接
