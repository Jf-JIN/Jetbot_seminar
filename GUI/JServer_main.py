#!/usr/bin/python3
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow

from JServer_Function import *

# import socket
# from JClient_Server_Video import *
# from JClient_Server_Console import *
# from PyQt5.QtCore import QObject, Qt, QSocketNotifier
# from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget
# import threading
# import numpy as np
# from Video_Const import *
# import rospy
# from std_msgs.msg import String
# from sensor_msgs.msg import CompressedImage

import subprocess


# class Cam_listen_QThread(QThread):
#     video_signal = pyqtSignal(object)

#     def __init__(self) -> None:
#         super().__init__()

#     def callback(self, data):
#         # time.sleep(1)
#         self.video_signal.emit(data.data)

#     def run(self):
#         try:
#             rospy.Subscriber("/tag_detections_image/compressed", CompressedImage, self.callback)
#             print('成功订阅')
#             rospy.spin()  # 保持节点运行，等待消息到来
#         except Exception as e:
#             e = traceback.format_exc()
#             print(e)


class Main(JServer_Function):
    data_video_send_signal = pyqtSignal(object)
    data_terminal_send_signal = pyqtSignal(dict)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle('Jetbot7客户端')
        # self.video_data = None
        # self.video_data = VIDEO_DATA_EXAMPLE
        # self.roscore = 'roscore'
        # self.roscam = 'rosrun demo camera.py'
        # self.rosrect = 'roslaunch camera mono_cam_rect.launch'
        # self.rosapril = 'roslaunch apriltag_ros continuous_detection.launch'
        # self.rqt = 'rqt'
        # self.process = subprocess.Popen(self.roscore, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        # time.sleep(1)
        # self.process = subprocess.Popen(self.roscam, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        # time.sleep(1)
        # self.process = subprocess.Popen(self.rosrect, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        # time.sleep(1)
        # self.process = subprocess.Popen(self.rosapril, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        # time.sleep(1)
        # self.process = subprocess.Popen(self.rqt, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        # time.sleep(1)
        # rospy.init_node('listener', anonymous=True)

        # self.vlisten = Cam_listen_QThread()
        # self.vlisten.video_signal.connect(self.send_video)
        # self.vlisten.start()

        # a = threading.Thread(target=self.send_video)
        # a.start()

    # 服务器发送视频信号
    # def send_video(self, video_data):
    #     self.data_video_send_signal.emit(video_data)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    exe = Main()
    exe.show()
    sys.exit(app.exec_())
