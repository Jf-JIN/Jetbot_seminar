
'''
该类是数据收集类，主要是相机识别二维码，识别两侧前方的Apritag，处理IMU数据，并将数据进行封装打包
'''

from JLocation import *
from scipy.spatial.transform import Rotation
import rospy
from sensor_msgs.msg import Imu,MagneticField
from apriltag_ros.msg import AprilTagDetectionArray
import math
import threading
from PyQt5.QtCore import QObject,pyqtSignal


class Imu_Data_Collection(QObject):
    signal_error_output = pyqtSignal(str)
    def __init__(self):
        super().__init__()
        self.imu_data = None
        self.magnetic_field_data = None

        # 初始化ROS节点
        rospy.init_node('imu_data_collection_node', anonymous=True)

        # 订阅IMU数据和磁场数据
        rospy.Subscriber("/imu/data_raw", Imu, self.imu_callback)
        rospy.Subscriber("/imu/mag", MagneticField, self.mag_callback)
        rospy.spin()  # 保持节点运行

    def imu_callback(self, data):
        try:
            # 从IMU消息中提取数据
            self.imu_data = {
                "linear_acceleration": [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z],
                "angular_velocity": [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z],
                "orientation": [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
            }

            rospy.loginfo("IMU data received")
        except AttributeError as e:
            rospy.logerr("Attribute error in imu_callback: %s", e)
        except Exception as e:
            rospy.logerr("Unexpected error in imu_callback: %s", e)

    def mag_callback(self, data):
        try:
            # 从磁场消息中提取数据
            self.magnetic_field_data = [data.magnetic_field.x, data.magnetic_field.y, data.magnetic_field.z]

            rospy.loginfo("Magnetic field data received")
        except AttributeError as e:
            error_text = f'[IMU] [!错误!]: {str(e)}'
            self.signal_error_output.emit(error_text)
            rospy.logerr("Attribute error in mag_callback: %s", e)
        except Exception as e:
            rospy.logerr("Unexpected error in mag_callback: %s", e)

    def get_imu_data(self):
        return self.imu_data

    def get_magnetic_field_data(self):
        return self.magnetic_field_data

    def run(self):
        rospy.spin()  # 保持节点运行？？？？？


class Camera_Data_Collection(QObject):
    signal_error_output = pyqtSignal(str)
    def __init__(self):
        rospy.init_node('camera_data_collection_node', anonymous=True)

        super().__init__()
        # 存储检测到的 Apriltag 信息
        self.apriltag_detections = []

        # 订阅 Apriltag 话题
        rospy.Subscriber("/apriltag_detections", AprilTagDetectionArray, self.apriltag_callback)

    def apriltag_callback(self, data):
        # 清空当前的 Apriltag 数据
        self.apriltag_detections = []

        # 处理每个检测到的 Apriltag
        for detection in data.detections:
            if len(detection.id) > 1:
                continue  # 跳过包含多个 ID 的检测信息

            tag_id = detection.id[0]
            tag_position = detection.pose.pose.pose.position
            tag_orientation = detection.pose.pose.pose.orientation

            tag_info = {
                'id': tag_id,
                'position': [
                    tag_position.x,
                    tag_position.y,
                    tag_position.z
                ],
                'orientation': [
                    tag_orientation.x,
                    tag_orientation.y,
                    tag_orientation.z,
                    tag_orientation.w
                ]
            }
            self.apriltag_detections.append(tag_info)

    def get_detections(self):
        return self.apriltag_detections


class JData_Collection(QObject):
    signal_error_output = pyqtSignal(str)

    def __init__(self):
        rospy.init_node('jdata_collection_node', anonymous=True)

        super().__init__()
        # 初始化 Camera_Data_Collection 和 Imu_Data_Collection
        self.camera_collector = Camera_Data_Collection()
        self.imu_collector = Imu_Data_Collection()
        self.imu_collector.signal_error_output.connect(self.signal_error_output.emit)

        # 初始化 JLocation 对象
        self.jlocation = JLocation()

        # 在单独的线程中运行 IMU 数据收集
        imu_thread = threading.Thread(target=self.imu_collector.run)
        imu_thread.start()

    # 四元数转换为欧拉角
    def quaternion_to_euler(self, quaternion):
        r = Rotation.from_quat(quaternion)
        euler = r.as_euler('xyz', degrees=True)
        return euler

    def update_jlocation_all(self):
        new_jlocation = JLocation()

        # 从摄像头收集检测数据
        detections = self.camera_collector.get_detections()

        # 清空新 JLocation 中的 Apriltag 数据
        new_jlocation.set_left_list([])
        new_jlocation.set_right_list([])
        new_jlocation.set_front(None)

        max_distance = -1
        front_tag = None

        # 处理摄像头检测数据
        for detection in detections:
            tag_info = JApril_Tag_Info()
            tag_info.set_id(detection['id'])
            tag_info.set_distance(detection['position'])

            # 转换 orientation 为欧拉角
            euler_orientation = self.quaternion_to_euler(detection['orientation'])
            tag_info.set_orientation(euler_orientation)

            position = detection['position']
            distance = math.sqrt(position[0] ** 2 + position[1] ** 2 + position[2] ** 2)

            if distance > max_distance:
                max_distance = distance
                front_tag = tag_info

            if position[0] < 0:  # 根据 x 坐标分类
                new_jlocation.left_list.append(tag_info)
            else:
                new_jlocation.right_list.append(tag_info)

        if front_tag:
            new_jlocation.set_front(front_tag)

        # 收集 IMU 数据
        imu_data = self.imu_collector.get_imu_data()
        magnetic_field_data = self.imu_collector.get_magnetic_field_data()

        # 处理 IMU 数据
        if imu_data:
            imu_info = JImu_Info()
            imu_info.set_velocity(imu_data["linear_acceleration"])
            imu_info.set_angular_velocity(imu_data["angular_velocity"])

            # 转换 orientation 为欧拉角
            euler_orientation = self.quaternion_to_euler(imu_data["orientation"])
            imu_info.set_orientation(euler_orientation)

            new_jlocation.set_imu(imu_info)

        if magnetic_field_data:
            new_jlocation.set_magnetic_field(magnetic_field_data)

        return new_jlocation