from PyQt5.QtWidgets import QWidget


from JConsole_ui import *


class JConcole(QWidget, Ui_Form):
    def __init__(self) -> None:
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle('JetBot7-Console')

    def build_console_dict(self):
        console_dict = {
            'roscore': self.le_roscore.text(),
            'camera': self.le_ros_camera.text(),
        }
        return console_dict
