
from CopyBoard_ui import *
from Client_Server import *

import sys

from PyQt5.QtWidgets import QMainWindow, QApplication, QPlainTextEdit, QMessageBox


class Copy_Board(Ui_MainWindow):
    data_recv_signal = pyqtSignal(dict)
    data_send_signal = pyqtSignal(object)
    connected_flag_signal = pyqtSignal(bool)
    error_output_signal = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        self.setupUi(self)
        self.parameter_init()
        self.signal_connection()
        self.ui_init()

    def parameter_init(self):
        self.green_status = 'rgb(100, 190, 0)'
        self.red_status = 'rgb(255, 66, 0)'
        self.thread_list = []
        self.connected_flag = False

    def signal_connection(self):
        self.pb_clear.clicked.connect(self.pte.clear)
        self.pb_connect.clicked.connect(self.server_client_connection)
        self.pb_send.clicked.connect(self.send_msg)

    def ui_init(self):
        self.lb_ip.hide()
        self.pb_send.setEnabled(False)

    def change_connection_status_label(self, value):
        if value:
            self.lb_connection.setStyleSheet(
                f'background-color: {self.green_status}')
            self.lb_connection.setText('已连接')
            self.pb_send.setEnabled(True)
            self.connected_flag = True
        else:
            self.lb_connection.setStyleSheet(
                f'background-color: {self.red_status}')
            self.lb_connection.setText('未连接')
            self.pb_send.setEnabled(False)
            self.connected_flag = False

    def get_ip(self):
        return self.le_ip.text().strip()

    def update_text(self, text):
        text: dict
        for key, value in text.items():
            self.add_text(value)

    def add_text(self, text):
        if not self.pte.toPlainText():
            self.pte.setPlainText(str(text))
        else:
            self.pte.setPlainText(str(self.pte.toPlainText())+'\n'+str(text))

    def send_msg(self):
        text = {'text': self.pte.toPlainText()}
        if self.lb_connection.text() == '已连接':
            self.data_send_signal.emit(text)

    def error_display(self, error_content):
        pass

    def server_client_connection(self):
        if self.connected_flag == True:
            return
        ip = self.get_ip()
        if not self.le_ip or ip == '':
            QMessageBox.warning(None, '提示', '请填写Jetbot的ip地址和本机端口')
            return
        try:
            self.client_thrd = Client_QThread(ip)
            self.client_thrd.connected_flag_signal.connect(
                self.change_connection_status_label)
            self.client_thrd.data_recv_signal.connect(self.update_text)
            self.client_thrd.error_output_signal.connect(self.error_display)
            self.data_send_signal.connect(self.client_thrd.send)
            self.client_thrd.start()
        except Exception as e:
            print(e)


class Main(QMainWindow, Copy_Board):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle('粘贴板_客户端')

    def send_close_signal(self):
        if hasattr(self, 'client_thrd') and self.client_thrd and self.client_thrd.isRunning():
            close_signal = {'close': 'Client disconnected'}
            self.data_send_signal.emit(close_signal)

    def closeEvent(self, event):
        self.send_close_signal()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    exe = Main()
    exe.show()
    sys.exit(app.exec_())
