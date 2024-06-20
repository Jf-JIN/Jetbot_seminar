#!/usr/bin/python3

from PyQt5.QtCore import QObject
from PyQt5.QtWidgets import QApplication
# from PyQt5.QtCore import


from JServer_Function import *
from JClient_Server_Video import *
from JClient_Server_Console import *


import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage


class Camera_Listener_QThread(QThread):
    video_signal = pyqtSignal(object)

    def __init__(self, frequency=30) -> None:
        super().__init__()
        self.frequency = frequency  # 订阅处理的最大频率 (Hz)
        self.last_processed_time = 0  # 初始化上次处理的时间

    def callback(self, data):
        current_time = time.time()
        time_elapsed = current_time - self.last_processed_time

        if time_elapsed >= 1.0 / self.frequency:  # 频率控制
            self.last_processed_time = current_time
            self.video_signal.emit(data.data)
        else:
            print(f"跳过帧, 频率太高: {1 / time_elapsed} Hz")

    def run(self):
        try:
            rospy.Subscriber("/tag_detections_image/compressed", CompressedImage, self.callback)
            print('成功订阅')
            rospy.spin()  # 保持节点运行，等待消息到来
        except Exception as e:
            e = traceback.format_exc()
            print(e)
