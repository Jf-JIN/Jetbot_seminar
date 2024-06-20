from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import QObject, Qt, QEvent, pyqtSignal


from JConsole_ui import *


class JConcole(QWidget, Ui_Form):
    signal_enter_pressed = pyqtSignal()

    def __init__(self) -> None:
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle('JetBot7-Console')
        self.signal_connections()
        self.line_edit_init()

    def signal_connections(self):
        self.pb_roscore_clear.clicked.connect(self.tb_roscore.clear)
        self.pb_ros_camera_clear.clicked.connect(self.tb_ros_camera.clear)
        self.pb_ros_rectify_clear.clicked.connect(self.tb_ros_rectify.clear)
        self.pb_ros_apriltag_detection_clear.clicked.connect(self.tb_ros_apriltag_detection.clear)
        self.pb_ros_imu_clear.clicked.connect(self.tb_ros_imu.clear)
        self.pb_ros_motor_clear.clicked.connect(self.tb_ros_motor.clear)
        self.pb_ros_algorithm_clear.clicked.connect(self.tb_ros_algorithm.clear)

    def build_console_dict(self):
        console_dict = {
            'roscore': self.le_roscore,
            'ros_camera': self.le_ros_camera,
            'ros_rectify': self.le_ros_rectify,
            'ros_apriltag_detection': self.le_ros_apriltag_detection,
            'ros_imu': self.le_ros_imu,
            'ros_motor': self.le_ros_motor,
            'ros_algorithm': self.le_ros_algorithm
        }
        return console_dict

    def line_edit_init(self):
        self.le_roscore.setClearButtonEnabled(True)
        self.le_roscore.setPlaceholderText('roscore')
        self.le_ros_camera.setClearButtonEnabled(True)
        self.le_ros_camera.setPlaceholderText('rosrun demo camera.py')
        self.le_ros_rectify.setClearButtonEnabled(True)
        self.le_ros_rectify.setPlaceholderText('roslaunch camera mono_cam_rect.launch ')
        self.le_ros_apriltag_detection.setClearButtonEnabled(True)
        self.le_ros_apriltag_detection.setPlaceholderText('roslaunch apriltag_ros continuous_detection.launch ')
        self.le_ros_imu.setClearButtonEnabled(True)
        self.le_ros_imu.setPlaceholderText('rosrun demo imu_publisher.py ')
        self.le_ros_motor.setClearButtonEnabled(True)
        self.le_ros_motor.setPlaceholderText('rosrun demo motor_driver.py ')
        self.le_ros_algorithm.setClearButtonEnabled(True)
        self.le_ros_algorithm.setPlaceholderText('')
