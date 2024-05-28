
from JClient_UI import *
from JClient_Connection import *
from PyQt5.QtWidgets import QMessageBox


class JClient_Function(JClient_UI):
    def __init__(self) -> None:
        super().__init__()
        self.parameter_init()
        self.signal_connections()
        self.server_client_init()

    def parameter_init(self):
        super().parameter_init()
        self.__server_ip = None

    def signal_connections(self):
        self.pb_connect.clicked.connect(self.connection_client_server)
        

    def server_client_init(self):
        self.__client = Client_Win_Ubuntu(self.__server_ip)
        self.__client.connection_flag_signal.connect(
            lambda x: self.change_client_server_connection_display(x))
        self.__client.connection_port_signal.connect(lambda x: self.load_jetbot_port(x))
        self.__client.connection_error_signal.connect(lambda x: self.append_TB_text(x, self.textBrowser))

    def get_ip(self):
        self.__server_ip = self.le_jetbot_ip.text().strip()

    def connection_client_server(self):
        self.get_ip()
        if not self.le_jetbot_ip:
            QMessageBox.warning(None, '提示', '请填写Jetbot的ip地址和本机端口')
            return
        self.__client.client_connect(self.__server_ip)
    
    def load_jetbot_port(self, port):
        self.lb_jetbot_port.setText(port)
