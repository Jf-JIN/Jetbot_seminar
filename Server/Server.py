import socket
import json

class Server_Win_Ubuntu():
    def __init__(self, ip, connections_number) -> None:
        self.__connections_number = connections_number
        self.__ip = ip
        self.__default_port = 10086
        self.__data = None
        self.__connection_flag = False
        self.connected_clients = 0  # 记录当前连接的客户端数量

    @property
    def data(self):
        return self.__data

    @property
    def connection_flag(self):
        return self.__connection_flag

    def server_connect(self, ip:int = None):
        try:
            if not ip:
                ip = self.__ip
            else:
                self.__ip = ip
            self.__server_socket = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM)
            self.__server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 允许端口重用
            server_address = (ip, self.__default_port)
            self.__server_socket.bind(server_address)
            self.__server_socket.listen(self.__connections_number)
            print('等待连接...')
            self.__client_socket, self.__client_address = self.__server_socket.accept()
            self.connected_clients += 1  # 连接建立时增加连接数量
            self.__connection_flag = True
            text = {"Jetbot_Port": self.__client_address[1]}
            self.server_send(text)
            print('已连接：', self.__client_address)
            print('当前已连接客户端数量:', self.connected_clients)
        except Exception as e :
            print(e)

    def server_send(self, message: json):
        self.__client_socket.sendall(json.dumps(message).encode())

    def server_recv(self) -> json:
        data = self.__client_socket.recv(1024)
        if not data:
            self.connected_clients -= 1  # 连接关闭时减少连接数量
            return None
        return json.loads(data.decode())
    
    def listening(self, flag: bool = True):
        while flag:
            try:
                data = self.server_recv()
                if not data:
                    self.__connection_flag = False
                    self.server_connect()
                    continue
                self.__data = data
            except ConnectionResetError as e:
                print(f"连接被重置: {e}")
                self.__connection_flag = False
                self.connected_clients -= 1
                self.server_connect()

    def close(self):
        self.__client_socket.close()
        self.__server_socket.close()


def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
    except Exception as e:
        local_ip = "Unable to get IP address"
        print(f"Error: {e}")
    finally:
        s.close()
    return local_ip


ip = get_local_ip()
test_obj = Server_Win_Ubuntu(ip,10)
test_obj.server_connect()
test_obj.listening(True)
