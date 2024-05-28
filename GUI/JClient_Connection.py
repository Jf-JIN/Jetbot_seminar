import socket
import json
import traceback

from PyQt5.QtCore import pyqtSignal, QObject

# 客户端


class Client_Win_Ubuntu(QObject):
    connection_port_signal = pyqtSignal(str)
    connection_flag_signal = pyqtSignal(bool)
    connection_error_signal = pyqtSignal(str)
    data_signal = pyqtSignal(dict)

    def __init__(self, ip) -> None:
        super(Client_Win_Ubuntu, self).__init__()
        self.__default_port = 10086
        self.__ip = ip
        self.__data = None
        # self.client_init()

    @property
    def data(self):
        return self.__data

    def client_connect(self, ip=None):
        try:
            if not ip:
                ip = self.__ip
            else:
                self.__ip = ip
            self.__client_socket = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM)
            server_address = (ip, self.__default_port)
            self.__client_socket.connect(server_address)
            connection_port = self.client_recv()
            if connection_port:
                self.connection_port_signal.emit(str(connection_port['Jetbot_Port']))
                self.connection_flag_signal.emit(True)
        except Exception as e:
            # e = traceback.format_exc()
            self.connection_error_signal.emit(str(e))

    def client_send(self, message: dict):
        self.__client_socket.sendall(json.dumps(message).encode())

    def client_recv(self) -> json:
        data = self.__client_socket.recv(1024)
        if not data:
            return None
        return json.loads(data.decode())

    def listening(self, listen_flag: bool = True):
        while listen_flag:
            data = self.client_recv()
            if not data:
                self.connection_flag_signal.emit(False)
                self.client_connect()
                continue
            self.__data = data
            self.data_signal.emit(data)

    def close(self):
        self.__client_socket.close()