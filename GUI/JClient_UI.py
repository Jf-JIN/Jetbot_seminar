
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QPixmap, QIcon, QPalette, QColor
from PyQt5.QtCore import QByteArray, QSize

from Jetbot_Client_ui import *
from Icon_setting import *
import socket

class JClient_UI( Ui_MainWindow):
    def __init__(self) -> None:
        super().__init__()
        # self.gui = Ui_MainWindow()
        self.setupUi(self)
        self.parameter_init()
        self.UI_setup()
    
    def parameter_init(self):
        self.green_status = 'rgb(100, 190, 0)'
        self.red_status = 'rgb(255, 66, 0)'
        self.icon_size = QSize(64, 64)
        
    def UI_setup(self):
        self.pb_up.setIcon(self.icon_setup(PB_UP))
        self.pb_down.setIcon(self.icon_setup(PB_DOWN))
        self.pb_left.setIcon(self.icon_setup(PB_LEFT))
        self.pb_right.setIcon(self.icon_setup(PB_RIGHT))
        self.pb_center.setIcon(self.icon_setup(PB_CENTER))
        for i in [self.pb_up, self.pb_down, self.pb_left, self.pb_right, self.pb_center]:
            i.setIconSize(self.icon_size)
        self.change_connection_display(False)
        self.cbb_port.addItem(self.find_free_port())
        self.cbb_task.addItem('Aufgabe 1: Labyrinthlösung')
        self.cbb_task.addItem('Aufgabe 2: Erkundung und Mapping')
        self.cbb_task.addItem('Aufgabe 3: Search and Rescue')
    
    def change_connection_display(self, value: bool)->None:
        if value:
            self.lb_status.setText('已连接')
            self.lb_status.setStyleSheet(f'background-color: {self.green_status}')
        else:
            self.lb_status.setText('未连接')
            self.lb_status.setStyleSheet(f'background-color: {self.red_status}')
    
    def icon_setup(self, icon_code):
        pixmap = QPixmap()
        pixmap.loadFromData(QByteArray(icon_code.encode()))
        return QIcon(pixmap)
    
    
    def find_free_port(self):
        # 创建一个临时的 socket
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            # 绑定到地址 ('localhost', 0)，端口号 0 告诉操作系统自动选择一个空闲端口
            s.bind(('localhost', 0))
            # 获取操作系统选择的端口号
            port = s.getsockname()[1]
            return str(port)