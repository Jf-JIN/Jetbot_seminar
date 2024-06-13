from JClient_Function import *


import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget


class Main(QMainWindow, JClient_Function):
    def __init__(self) -> None:
        super().__init__()
        # self.le_jetbot_ip.setText('192.168.124.11')
        self.le_jetbot_ip.setText('192.168.124.5')
        # self.le_jetbot_ip.setText('192.168.124.9')
        self.setWindowTitle('JetBot7-Client')

    # 应用关闭事件
    def closeEvent(self, event):
        self.send_close_signal()
        # 关闭子窗口
        if hasattr(self, 'console_win'):
            self.console_win.close()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    exe = Main()
    exe.show()
    sys.exit(app.exec_())
