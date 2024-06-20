#!/usr/bin/python3

from PyQt5.QtWidgets import QApplication
# from PyQt5.QtCore import


from JServer_Function import *
from JClient_Server_Video import *
from JClient_Server_Console import *


class Main(JServer_Function):
    data_video_send_signal = pyqtSignal(object)
    data_terminal_send_signal = pyqtSignal(dict)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle('Jetbot7 服务器')


if __name__ == "__main__":
    app = QApplication(sys.argv)
    exe = Main()
    exe.show()
    sys.exit(app.exec_())
