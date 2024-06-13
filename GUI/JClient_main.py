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

    # 客户端向服务端发送关闭信号
    def send_close_signal(self):
        if hasattr(self, 'client_video') and self.client_video and self.client_video.isRunning():
            close_signal = {'close': 'Client_disconnected'}
            self.signal_data_close_client_send.emit(close_signal)
            self.signal_data_close_client_send.disconnect(self.client_video.send)      # 断开信号连接，避免重复被激活
            self.client_video.stop()
        if hasattr(self, 'client_console') and self.client_console and self.client_console.isRunning():
            close_signal = {'close': 'Client_disconnected'}
            self.signal_data_close_client_send.emit(close_signal)
            self.signal_data_close_client_send.disconnect(self.client_console.send)   # 发送客户端关闭信号
            self.client_console.stop()

    # 应用关闭事件
    def closeEvent(self, event):
        self.send_close_signal()
        if hasattr(self, 'console_win'):
            self.console_win.close()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    exe = Main()
    exe.show()
    sys.exit(app.exec_())
