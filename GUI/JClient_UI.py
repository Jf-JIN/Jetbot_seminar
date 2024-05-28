
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QPixmap, QIcon, QPalette, QColor, QTextCursor
from PyQt5.QtCore import QByteArray, QSize, Qt

from Jetbot_Client_ui import *
from Icon_setting import *
import socket
import traceback


class JClient_UI(Ui_MainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setupUi(self)
        self.parameter_init()
        self.UI_setup()

    def parameter_init(self):
        self.green_status = 'rgb(100, 190, 0)'
        self.red_status = 'rgb(255, 66, 0)'
        self.icon_size = QSize(32, 32)
        self.traceback_display_flag = True

    def UI_setup(self):
        self.pb_up.setIcon(self.icon_setup(PB_UP))
        self.pb_down.setIcon(self.icon_setup(PB_DOWN))
        self.pb_left.setIcon(self.icon_setup(PB_LEFT))
        self.pb_right.setIcon(self.icon_setup(PB_RIGHT))
        self.pb_center.setIcon(self.icon_setup(PB_CENTER))
        for i in [self.pb_up, self.pb_down, self.pb_left, self.pb_right, self.pb_center]:
            i.setIconSize(self.icon_size)
        self.change_client_server_connection_display(False)
        self.cbb_task.addItem('Aufgabe 1: Labyrinthlösung')
        self.cbb_task.addItem('Aufgabe 2: Erkundung und Mapping')
        self.cbb_task.addItem('Aufgabe 3: Search and Rescue')
        self.textBrowser.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

    def change_client_server_connection_display(self, value: bool) -> None:
        if value:
            self.lb_status.setText('已连接')
            self.lb_status.setStyleSheet(
                f'background-color: {self.green_status}')
        else:
            self.lb_status.setText('未连接')
            self.lb_status.setStyleSheet(
                f'background-color: {self.red_status}')

    def icon_setup(self, icon_code):
        pixmap = QPixmap()
        pixmap.loadFromData(QByteArray(icon_code.encode()))
        return QIcon(pixmap)

    # 暂时无用
    def find_free_port(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(('localhost', 0))
            port = s.getsockname()[1]
            return str(port)

# ****************************************向Textbrowser添加内容****************************************
    def append_TB_text(self, text_content: str, textBrowser_object: object = None)->None:
        if not textBrowser_object:
            textBrowser_object = self.Win.textBrowser
        try:
            textBrowser_object.moveCursor(QTextCursor.End)
            textBrowser_object.insertPlainText(text_content + "\n")
            textBrowser_object.moveCursor(QTextCursor.End)
        except Exception as e:
            if self.traceback_display_flag:
                e = traceback.format_exc()
            textBrowser_object.moveCursor(QTextCursor.End)
            textBrowser_object.insertPlainText( str(e) + "\n")
            textBrowser_object.moveCursor(QTextCursor.End)