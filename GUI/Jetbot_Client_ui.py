# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'e:\10_Programm\0002_Python\0902_Jetbot_seminar\UI\Jetbot_Client.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1000, 1051)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.centralwidget.sizePolicy().hasHeightForWidth())
        self.centralwidget.setSizePolicy(sizePolicy)
        self.centralwidget.setStyleSheet("QWidget#centralwidget{\n"
"    background-color: rgb(85, 170, 255);\n"
"    min-width: 1000px;\n"
"    min-height: 1000px;\n"
"}\n"
"\n"
"QPushButton{\n"
"    border: 1px solid;\n"
"    border-radius: 20px;\n"
"    font: 18px \'幼圆\';\n"
"}\n"
"\n"
"QPushButton#pb_launch{\n"
"    background-color: rgb(100, 190, 0)\n"
"}\n"
"\n"
"QPushButton#pb_launch:hover{\n"
"    background-color: rgb(0, 150, 0);\n"
"    padding-bottom: 5px;\n"
"    font: 25px;\n"
"    font-weight: bold;\n"
"}\n"
"\n"
"QPushButton#pb_stop{\n"
"    background-color: rgb(255, 85, 0);\n"
"}\n"
"\n"
"QPushButton#pb_stop:hover{\n"
"    background-color: rgb(180, 0, 0);\n"
"    padding-bottom: 5px;\n"
"    font: 25px;\n"
"    font-weight: bold;\n"
"}\n"
"\n"
"QPushButton#pb_up, QPushButton#pb_down, QPushButton#pb_left, QPushButton#pb_right, QPushButton#pb_center{\n"
"    background-color: rgb(180, 180, 180);\n"
"}\n"
"\n"
"QPushButton#pb_up:hover, QPushButton#pb_down:hover, QPushButton#pb_left:hover, QPushButton#pb_right:hover, QPushButton#pb_center:hover{\n"
"    background-color: rgb(100, 100, 100);\n"
"    padding-bottom: 5px;\n"
"\n"
"}\n"
"\n"
"QFrame#frame_left{\n"
"    max-width: 500px;\n"
"}\n"
"\n"
"QFrame#frame_left QLabel{\n"
"    min-height: 35px;\n"
"    max-height: 40px;\n"
"    font: 18px \'幼圆\'\n"
"}\n"
"\n"
"QFrame#frame_left QLabel#lb_title{\n"
"    font: 30px;\n"
"}\n"
"\n"
"QFrame#frame_left QLabel#lb_status{\n"
"    padding: 5px;\n"
"    font-weight: bold;\n"
"}\n"
"\n"
"QFrame#frame_task{\n"
"    max-height: 40px;\n"
"}\n"
"\n"
"QFrame#frame_left QLabel#lb_jetbot_port{\n"
"    font: 18px \'Arial\';\n"
"}\n"
"\n"
"QComboBox, QLineEdit{\n"
"    font: 18px \'Arial\';\n"
"    min-height: 35px;\n"
"    max-height: 40px;\n"
"}")
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.frame_left = QtWidgets.QFrame(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_left.sizePolicy().hasHeightForWidth())
        self.frame_left.setSizePolicy(sizePolicy)
        self.frame_left.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_left.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_left.setObjectName("frame_left")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.frame_left)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.frame_3 = QtWidgets.QFrame(self.frame_left)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_3.sizePolicy().hasHeightForWidth())
        self.frame_3.setSizePolicy(sizePolicy)
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.frame_3)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setSpacing(0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.frame_6 = QtWidgets.QFrame(self.frame_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_6.sizePolicy().hasHeightForWidth())
        self.frame_6.setSizePolicy(sizePolicy)
        self.frame_6.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_6.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_6.setObjectName("frame_6")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.frame_6)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setSpacing(0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.lb_title = QtWidgets.QLabel(self.frame_6)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lb_title.sizePolicy().hasHeightForWidth())
        self.lb_title.setSizePolicy(sizePolicy)
        self.lb_title.setAlignment(QtCore.Qt.AlignCenter)
        self.lb_title.setObjectName("lb_title")
        self.horizontalLayout_2.addWidget(self.lb_title)
        self.verticalLayout_3.addWidget(self.frame_6)
        self.frame_7 = QtWidgets.QFrame(self.frame_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_7.sizePolicy().hasHeightForWidth())
        self.frame_7.setSizePolicy(sizePolicy)
        self.frame_7.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_7.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_7.setObjectName("frame_7")
        self.formLayout = QtWidgets.QFormLayout(self.frame_7)
        self.formLayout.setContentsMargins(-1, 0, 0, 0)
        self.formLayout.setSpacing(10)
        self.formLayout.setObjectName("formLayout")
        self.label_6 = QtWidgets.QLabel(self.frame_7)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_6.sizePolicy().hasHeightForWidth())
        self.label_6.setSizePolicy(sizePolicy)
        self.label_6.setObjectName("label_6")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_6)
        self.le_jetbot_ip = QtWidgets.QLineEdit(self.frame_7)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.le_jetbot_ip.sizePolicy().hasHeightForWidth())
        self.le_jetbot_ip.setSizePolicy(sizePolicy)
        self.le_jetbot_ip.setObjectName("le_jetbot_ip")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.le_jetbot_ip)
        self.label_4 = QtWidgets.QLabel(self.frame_7)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_4.sizePolicy().hasHeightForWidth())
        self.label_4.setSizePolicy(sizePolicy)
        self.label_4.setObjectName("label_4")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_4)
        self.cbb_port = QtWidgets.QComboBox(self.frame_7)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.cbb_port.sizePolicy().hasHeightForWidth())
        self.cbb_port.setSizePolicy(sizePolicy)
        self.cbb_port.setObjectName("cbb_port")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.cbb_port)
        self.label_3 = QtWidgets.QLabel(self.frame_7)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_3.sizePolicy().hasHeightForWidth())
        self.label_3.setSizePolicy(sizePolicy)
        self.label_3.setObjectName("label_3")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.LabelRole, self.label_3)
        self.lb_jetbot_port = QtWidgets.QLabel(self.frame_7)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lb_jetbot_port.sizePolicy().hasHeightForWidth())
        self.lb_jetbot_port.setSizePolicy(sizePolicy)
        self.lb_jetbot_port.setObjectName("lb_jetbot_port")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.lb_jetbot_port)
        self.label_8 = QtWidgets.QLabel(self.frame_7)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_8.sizePolicy().hasHeightForWidth())
        self.label_8.setSizePolicy(sizePolicy)
        self.label_8.setObjectName("label_8")
        self.formLayout.setWidget(5, QtWidgets.QFormLayout.LabelRole, self.label_8)
        self.frame_4 = QtWidgets.QFrame(self.frame_7)
        self.frame_4.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_4.setObjectName("frame_4")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.frame_4)
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_4.setSpacing(0)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.lb_status = QtWidgets.QLabel(self.frame_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lb_status.sizePolicy().hasHeightForWidth())
        self.lb_status.setSizePolicy(sizePolicy)
        self.lb_status.setObjectName("lb_status")
        self.horizontalLayout_4.addWidget(self.lb_status)
        spacerItem = QtWidgets.QSpacerItem(290, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem)
        self.formLayout.setWidget(5, QtWidgets.QFormLayout.FieldRole, self.frame_4)
        self.verticalLayout_3.addWidget(self.frame_7)
        self.verticalLayout.addWidget(self.frame_3)
        self.frame = QtWidgets.QFrame(self.frame_left)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame.sizePolicy().hasHeightForWidth())
        self.frame.setSizePolicy(sizePolicy)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.frame)
        self.verticalLayout_4.setContentsMargins(0, 0, 0, -1)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.frame_2 = QtWidgets.QFrame(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(50)
        sizePolicy.setHeightForWidth(self.frame_2.sizePolicy().hasHeightForWidth())
        self.frame_2.setSizePolicy(sizePolicy)
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.frame_2)
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_5.setSpacing(10)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.frame_task = QtWidgets.QFrame(self.frame_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(100)
        sizePolicy.setHeightForWidth(self.frame_task.sizePolicy().hasHeightForWidth())
        self.frame_task.setSizePolicy(sizePolicy)
        self.frame_task.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_task.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_task.setObjectName("frame_task")
        self.formLayout_2 = QtWidgets.QFormLayout(self.frame_task)
        self.formLayout_2.setContentsMargins(10, 0, 0, 0)
        self.formLayout_2.setSpacing(10)
        self.formLayout_2.setObjectName("formLayout_2")
        self.label_9 = QtWidgets.QLabel(self.frame_task)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_9.sizePolicy().hasHeightForWidth())
        self.label_9.setSizePolicy(sizePolicy)
        self.label_9.setObjectName("label_9")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_9)
        self.cbb_task = QtWidgets.QComboBox(self.frame_task)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.cbb_task.sizePolicy().hasHeightForWidth())
        self.cbb_task.setSizePolicy(sizePolicy)
        self.cbb_task.setObjectName("cbb_task")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.cbb_task)
        self.verticalLayout_5.addWidget(self.frame_task)
        spacerItem1 = QtWidgets.QSpacerItem(20, 20, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_5.addItem(spacerItem1)
        self.frame_5 = QtWidgets.QFrame(self.frame_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(100)
        sizePolicy.setHeightForWidth(self.frame_5.sizePolicy().hasHeightForWidth())
        self.frame_5.setSizePolicy(sizePolicy)
        self.frame_5.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_5.setObjectName("frame_5")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.frame_5)
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_3.setSpacing(10)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.pb_launch = QtWidgets.QPushButton(self.frame_5)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pb_launch.sizePolicy().hasHeightForWidth())
        self.pb_launch.setSizePolicy(sizePolicy)
        self.pb_launch.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.pb_launch.setObjectName("pb_launch")
        self.horizontalLayout_3.addWidget(self.pb_launch)
        self.pb_stop = QtWidgets.QPushButton(self.frame_5)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pb_stop.sizePolicy().hasHeightForWidth())
        self.pb_stop.setSizePolicy(sizePolicy)
        self.pb_stop.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.pb_stop.setObjectName("pb_stop")
        self.horizontalLayout_3.addWidget(self.pb_stop)
        self.verticalLayout_5.addWidget(self.frame_5)
        self.verticalLayout_4.addWidget(self.frame_2)
        self.frame_controll = QtWidgets.QFrame(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(100)
        sizePolicy.setHeightForWidth(self.frame_controll.sizePolicy().hasHeightForWidth())
        self.frame_controll.setSizePolicy(sizePolicy)
        self.frame_controll.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_controll.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_controll.setObjectName("frame_controll")
        self.gridLayout = QtWidgets.QGridLayout(self.frame_controll)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setSpacing(10)
        self.gridLayout.setObjectName("gridLayout")
        self.pb_up = QtWidgets.QPushButton(self.frame_controll)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pb_up.sizePolicy().hasHeightForWidth())
        self.pb_up.setSizePolicy(sizePolicy)
        self.pb_up.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.pb_up.setText("")
        self.pb_up.setObjectName("pb_up")
        self.gridLayout.addWidget(self.pb_up, 0, 1, 1, 1)
        self.pb_left = QtWidgets.QPushButton(self.frame_controll)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pb_left.sizePolicy().hasHeightForWidth())
        self.pb_left.setSizePolicy(sizePolicy)
        self.pb_left.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.pb_left.setText("")
        self.pb_left.setObjectName("pb_left")
        self.gridLayout.addWidget(self.pb_left, 1, 0, 1, 1)
        self.pb_center = QtWidgets.QPushButton(self.frame_controll)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pb_center.sizePolicy().hasHeightForWidth())
        self.pb_center.setSizePolicy(sizePolicy)
        self.pb_center.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.pb_center.setText("")
        self.pb_center.setObjectName("pb_center")
        self.gridLayout.addWidget(self.pb_center, 1, 1, 1, 1)
        self.pb_right = QtWidgets.QPushButton(self.frame_controll)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pb_right.sizePolicy().hasHeightForWidth())
        self.pb_right.setSizePolicy(sizePolicy)
        self.pb_right.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.pb_right.setText("")
        self.pb_right.setObjectName("pb_right")
        self.gridLayout.addWidget(self.pb_right, 1, 2, 1, 1)
        self.pb_down = QtWidgets.QPushButton(self.frame_controll)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pb_down.sizePolicy().hasHeightForWidth())
        self.pb_down.setSizePolicy(sizePolicy)
        self.pb_down.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.pb_down.setText("")
        self.pb_down.setObjectName("pb_down")
        self.gridLayout.addWidget(self.pb_down, 2, 1, 1, 1)
        self.verticalLayout_4.addWidget(self.frame_controll)
        self.verticalLayout.addWidget(self.frame)
        self.horizontalLayout.addWidget(self.frame_left)
        self.frame_right = QtWidgets.QFrame(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_right.sizePolicy().hasHeightForWidth())
        self.frame_right.setSizePolicy(sizePolicy)
        self.frame_right.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_right.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_right.setObjectName("frame_right")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.frame_right)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setSpacing(0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.lb_maze = QtWidgets.QLabel(self.frame_right)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lb_maze.sizePolicy().hasHeightForWidth())
        self.lb_maze.setSizePolicy(sizePolicy)
        self.lb_maze.setText("")
        self.lb_maze.setObjectName("lb_maze")
        self.verticalLayout_2.addWidget(self.lb_maze)
        self.lb_image = QtWidgets.QLabel(self.frame_right)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lb_image.sizePolicy().hasHeightForWidth())
        self.lb_image.setSizePolicy(sizePolicy)
        self.lb_image.setText("")
        self.lb_image.setObjectName("lb_image")
        self.verticalLayout_2.addWidget(self.lb_image)
        self.lb_command = QtWidgets.QLabel(self.frame_right)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lb_command.sizePolicy().hasHeightForWidth())
        self.lb_command.setSizePolicy(sizePolicy)
        self.lb_command.setText("")
        self.lb_command.setObjectName("lb_command")
        self.verticalLayout_2.addWidget(self.lb_command)
        self.horizontalLayout.addWidget(self.frame_right)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1000, 26))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.lb_title.setText(_translate("MainWindow", "JetBot 客户端"))
        self.label_6.setText(_translate("MainWindow", "JetBot IP"))
        self.label_4.setText(_translate("MainWindow", "本机端口"))
        self.label_3.setText(_translate("MainWindow", "JetBot端口"))
        self.lb_jetbot_port.setText(_translate("MainWindow", "12345"))
        self.label_8.setText(_translate("MainWindow", "连接状态"))
        self.lb_status.setText(_translate("MainWindow", "未连接"))
        self.label_9.setText(_translate("MainWindow", "任务内容"))
        self.pb_launch.setText(_translate("MainWindow", "运行"))
        self.pb_stop.setText(_translate("MainWindow", "停止"))