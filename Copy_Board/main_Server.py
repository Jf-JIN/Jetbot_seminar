
from CopyBoard_ui import *
from Client_Server import *

import sys


from PyQt5.QtWidgets import QMainWindow, QApplication, QPlainTextEdit
from PyQt5.QtCore import QMetaObject, Qt


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
        self.server_init()
        self.ui_init()

    def parameter_init(self):
        self.green_status = 'rgb(100, 190, 0)'
        self.red_status = 'rgb(255, 66, 0)'
        self.server_socket = None
        self.thread_list = []

    def signal_connection(self):
        self.pb_clear.clicked.connect(self.pte.clear)
        self.pb_send.clicked.connect(self.send_text)

    def ui_init(self):
        self.le_ip.hide()
        self.pb_connect.hide()
        self.lb_connection.setStyleSheet(
            f'background-color: {self.red_status}')
        self.lb_connection.setText('未连接')
        self.lb_ip.setText(self.get_local_ip())
        # self.pb_send.setEnabled(False)

    def get_local_ip(self) -> str:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
        except Exception as e:
            local_ip = None
            print(f"Error: {e}")
        finally:
            s.close()
        return local_ip

    @pyqtSlot(bool)
    def change_connection_status_label(self, value):
        print('here')
        if value:
            self.lb_connection.setStyleSheet(
                f'background-color: {self.green_status}')
            self.lb_connection.setText('已连接')
            self.pb_send.setEnabled(True)
        else:
            self.lb_connection.setStyleSheet(
                f'background-color: {self.red_status}')
            self.lb_connection.setText('未连接')
            self.pb_send.setEnabled(False)

    @pyqtSlot(dict)
    def update_text(self, text):
        text: dict
        for key, value in text.items():
            self.add_text(value)

    def add_text(self, text):
        if 'close' in text:
            pass
        if not self.pte.toPlainText():
            self.pte.setPlainText(str(text))
        else:
            self.pte.setPlainText(str(self.pte.toPlainText()+'\n'+text))

    @pyqtSlot(str)
    def error_display(self, error_content):
        pass

    def send_text(self):
        text = {'text': self.pte.toPlainText()}
        if self.lb_connection.text() == '已连接':
            self.data_send_signal.emit(text)

    def test(self, x):
        print('test', x)

    def server_init(self):
        ip = self.get_local_ip()
        self.server_thrd = Server_QThread(ip)
        self.server_thrd.data_recv_signal.connect(self.update_text)
        self.server_thrd.connected_flag_signal.connect(
            self.change_connection_status_label)
        self.server_thrd.error_output_signal.connect(self.error_display)
        self.data_send_signal.connect(self.server_thrd.send_all)
        self.server_thrd.start()


class Main(QMainWindow, Copy_Board):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle('粘贴板_服务端')


if __name__ == "__main__":
    app = QApplication(sys.argv)
    exe = Main()
    exe.show()
    sys.exit(app.exec_())
