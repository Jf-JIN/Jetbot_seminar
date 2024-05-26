from JClient_UI import *


import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget

class Main(QMainWindow, JClient_UI):
    def __init__(self) -> None:
        super().__init__()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    exe = Main()
    exe.show()
    sys.exit(app.exec_())
