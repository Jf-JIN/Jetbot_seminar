from PyQt5.QtWidgets import QWidget, QHBoxLayout
from PyQt5.QtCore import QObject, Qt, QEvent, pyqtSignal


from JConsole_ui import *
from JClient_Const import *

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from JLogger import *

logger_dict_client = JLog('Client', 'JClient')
log_info = logger_dict_client['info']
log_error = logger_dict_client['error']


class JConcole(QWidget, Ui_Form):
    signal_enter_pressed = pyqtSignal()

    def __init__(self) -> None:
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle('JetBot7-Console')
        self.signal_connections()
        self.line_edit_init()
        self.ui_init()

    def signal_connections(self):
        self.pb_roscore_clear.clicked.connect(self.tb_roscore.clear)
        self.pb_ros_camera_clear.clicked.connect(self.tb_ros_camera.clear)
        self.pb_ros_rectify_clear.clicked.connect(self.tb_ros_rectify.clear)
        self.pb_ros_apriltag_detection_clear.clicked.connect(self.tb_ros_apriltag_detection.clear)
        self.pb_ros_imu_clear.clicked.connect(self.tb_ros_imu.clear)
        self.pb_ros_imu_calib_clear.clicked.connect(self.tb_ros_imu_calib.clear)
        self.pb_ros_motor_clear.clicked.connect(self.tb_ros_motor.clear)
        self.pb_ros_algorithm_clear.clicked.connect(self.tb_ros_algorithm.clear)

    def ui_init(self):
        self.cb_save_last_data.setEnabled(True)
        self.cb_save_last_data.setChecked(False)
        self.figure_init()

    def figure_init(self):
        self.figure_front = Figure()
        self.canvas_front = FigureCanvas(self.figure_front)
        layout_front = QHBoxLayout()
        layout_front.addWidget(self.canvas_front)
        self.gb_filter_front.setLayout(layout_front)

        # self.figure_left = Figure()
        # self.canvas_left = FigureCanvas(self.figure_left)
        # layout_left = QHBoxLayout()
        # layout_left.addWidget(self.canvas_left)
        # self.gb_filter_left.setLayout(layout_left)

        # self.figure_right = Figure()
        # self.canvas_right = FigureCanvas(self.figure_right)
        # layout_right = QHBoxLayout()
        # layout_right.addWidget(self.canvas_right)
        # self.gb_filter_right.setLayout(layout_right)

        self.figure_imu_ori = Figure()
        self.canvas_imu_ori = FigureCanvas(self.figure_imu_ori)
        layout_imu_ori = QHBoxLayout()
        layout_imu_ori.addWidget(self.canvas_imu_ori)
        self.gb_filter_imu_ori.setLayout(layout_imu_ori)

        # self.figure_imu_acc = Figure()
        # self.canvas_imu_acc = FigureCanvas(self.figure_imu_acc)
        # layout_imu_acc = QHBoxLayout()
        # layout_imu_acc.addWidget(self.canvas_imu_acc)
        # self.gb_filter_imu_acc.setLayout(layout_imu_acc)

        # self.figure_imu_ang_spd = Figure()
        # self.canvas_imu_ang_spd = FigureCanvas(self.figure_imu_ang_spd)
        # layout_imu_ang_spd = QHBoxLayout()
        # layout_imu_ang_spd.addWidget(self.canvas_imu_ang_spd)
        # self.gb_filter_imu_ang_spd.setLayout(layout_imu_ang_spd)

        # self.figure_imu_mag = Figure()
        # self.canvas_imu_mag = FigureCanvas(self.figure_imu_mag)
        # layout_imu_mag = QHBoxLayout()
        # layout_imu_mag.addWidget(self.canvas_imu_mag)
        # self.gb_filter_imu_mag.setLayout(layout_imu_mag)

    def build_console_dict(self):
        console_dict = {
            'roscore': self.le_roscore,
            'ros_camera': self.le_ros_camera,
            'ros_rectify': self.le_ros_rectify,
            'ros_apriltag_detection': self.le_ros_apriltag_detection,
            'ros_imu': self.le_ros_imu,
            'ros_imu_calib': self.le_ros_imu_calib,
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
        self.le_ros_imu_calib.setClearButtonEnabled(True)
        self.le_ros_imu_calib.setPlaceholderText('rosrun imu calibration_node.py ')
        self.le_ros_motor.setClearButtonEnabled(True)
        self.le_ros_motor.setPlaceholderText('rosrun demo motor_driver.py ')
        self.le_ros_algorithm.setClearButtonEnabled(True)
        self.le_ros_algorithm.setPlaceholderText('')
