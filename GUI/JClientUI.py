
from PyQt5.QtWidgets import QMenuBar, QActionGroup, QAction, QTextBrowser, QTextEdit, QComboBox, QTabWidget, QSlider
from PyQt5.QtGui import QPixmap, QIcon, QTextCursor, QFont
from PyQt5.QtCore import QByteArray, QSize, Qt

import socket
import traceback

# from Icon_setting import *
from JClient_ui import *
from JConsoleUI import *


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
        self.flag_connection_to_server = False
        self.console_win = JConcole()

    def signal_connections(self):
        self.pb_launch_2.clicked.connect(self.console_win.show)
        self.tabWidget.currentChanged.connect(self.cbb_and_tab_connection)
        self.cbb_task.currentIndexChanged.connect(self.cbb_and_tab_connection)
        self.hs_video_size.valueChanged.connect(self.hs_video_size_display)

    def UI_setup(self):
        self.pb_close_server.setEnabled(False)
        self.lb_connection_status.setText('未连接')
        self.lb_connection_status.setStyleSheet(f'background-color: {self.red_status}')
        self.pb_reconnect_console.setStyleSheet(f'background-color: {self.red_status}')
        self.pb_reconnect_video.setStyleSheet(f'background-color: {self.red_status}')
        self.cbb_task.addItem('Aufgabe 1: Labyrinthlösung')
        self.cbb_task.addItem('Aufgabe 2: Erkundung und Mapping')
        self.cbb_task.addItem('Aufgabe 3: Search and Rescue')
        self.tb_console.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scrollArea.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scrollArea.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scrollArea_2.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scrollArea_2.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scrollArea_3.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scrollArea_3.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.lb_a1_discription.setWordWrap(True)
        self.lb_a2_discription.setWordWrap(True)
        self.lb_a3_discription.setWordWrap(True)
        self.menubar = QMenuBar()
        self.setMenuBar(self.menubar)
        self.menu_language = self.menubar.addMenu('语言')
        self.menu_action_group_language_change = QActionGroup(self)
        self.menu_action_Chinese = QAction('简体中文', self)
        self.menu_action_Chinese.setCheckable(True)
        self.menu_language.addAction(self.menu_action_Chinese)
        self.menu_action_group_language_change.addAction(self.menu_action_Chinese)
        self.menu_action_Deutsch = QAction('Deutsch', self)
        self.menu_action_Deutsch.setCheckable(True)
        self.menu_language.addAction(self.menu_action_Deutsch)
        self.menu_action_group_language_change.addAction(self.menu_action_Deutsch)
        self.hs_video_size.setRange(0, 100)  # 设置范围
        self.hs_video_size.setValue(40)       # 设置初始值
        self.lb_hs_display.setText('40')
        # self.hs_video_size.setTickPosition(QSlider.TicksBelow)  # 设置刻度位置
        self.hs_video_size.setTickInterval(10)  # 设置刻度间隔

    def change_client_server_connection_display(self) -> None:
        if self.lb_console_port.text() != '' and self.lb_video_port.text() != '':
            self.flag_connection_to_server = True
            self.pb_close_server.setEnabled(True)
            self.lb_connection_status.setText('已连接')
            self.lb_connection_status.setStyleSheet(f'background-color: {self.green_status}')
        else:
            self.flag_connection_to_server = False
            self.pb_close_server.setEnabled(False)
            self.lb_connection_status.setText('未连接')
            self.lb_connection_status.setStyleSheet(f'background-color: {self.red_status}')

    def reconnect_display(self):
        if self.lb_console_port.text() != '':
            self.pb_reconnect_console.setStyleSheet(f'background-color: {self.green_status}')
        else:
            self.pb_reconnect_console.setStyleSheet(f'background-color: {self.red_status}')
        if self.lb_video_port.text() != '':
            self.pb_reconnect_video.setStyleSheet(f'background-color: {self.green_status}')
        else:
            self.pb_reconnect_video.setStyleSheet(f'background-color: {self.red_status}')

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

    def cbb_and_tab_connection(self):
        sender = self.sender()
        if isinstance(sender, QComboBox):
            goal_index = self.cbb_task.currentIndex()
            self.tabWidget.setCurrentIndex(goal_index)
        elif isinstance(sender, QTabWidget):
            goal_index = self.tabWidget.currentIndex()
            self.cbb_task.setCurrentIndex(goal_index)

    def hs_video_size_display(self, value):
        self.lb_hs_display.setText(str(value))

        # **************************************** 向 TextBrowser | TextEdit 添加内容 ****************************************

    def append_TB_text(self, text_content: str, textBrowser_object: QTextBrowser = None) -> None:
        if not textBrowser_object:
            textBrowser_object = self.tb_console
        # print(type(textBrowser_object))
        try:
            textBrowser_object.moveCursor(QTextCursor.End)
            textBrowser_object.insertPlainText(text_content + "\n")
            textBrowser_object.moveCursor(QTextCursor.End)
        except Exception as e:
            if self.traceback_display_flag:
                e = traceback.format_exc()
            textBrowser_object.moveCursor(QTextCursor.End)
            textBrowser_object.insertPlainText(str(e) + "\n")
            textBrowser_object.moveCursor(QTextCursor.End)

# **************************************** 子窗口功能 ****************************************
