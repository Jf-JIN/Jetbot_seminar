
'''
该类是数据收集类，主要是相机识别二维码，识别两侧前方的Apritag，处理IMU数据，并将数据进行封装打包
'''

from JLocation import *
from scipy.spatial.transform import Rotation
import rospy
from sensor_msgs.msg import Imu
from apriltag_ros.msg import AprilTagDetectionArray
import math


class Imu_Data_Collection():
    def __init__(self):
        self.imu_data = None  # 初始化IMU数据属性
        rospy.Subscriber("/imu/data_raw", Imu, self.callback)  # 订阅IMU数据
        rospy.spin()  # 保持节点运行

        def callback(self, data):
            # 从IMU消息中提取数据
            linear_acceleration = data.linear_acceleration
            angular_velocity = data.angular_velocity
            orientation = data.orientation
            magnetic_field = data.magnetic_field
            angular_acceleration = data.angular_acceleration

            # 保存IMU数据到JImu_Info对象
            self.imu_data.set_acceleration([linear_acceleration.x, linear_acceleration.y, linear_acceleration.z])
            self.imu_data.set_angular_velocity([angular_velocity.x, angular_velocity.y, angular_velocity.z])
            self.imu_data.set_magnetic_field([magnetic_field.x, magnetic_field.y, magnetic_field.z])
            self.imu_data.set_angular_acceleration([angular_acceleration.x, angular_acceleration.y, angular_acceleration.z])
            self.imu_data.set_velocity([data.velocity.x, data.velocity.y, data.velocity.z])

            rospy.loginfo("IMU data received")

        def get_imu_data(self):
            return self.imu_data




class Camera_Data_Collection():
    def __init__(self):
        self.camera_data = None
        self.jlocation = JLocation()

        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.camera_callback)

    def camera_callback(self, data):
        self.camera_data = data
        rospy.loginfo("Received /tag_detections message")
        self.process_tags(data)

    def process_tags(self, data):
        tags = data.detections  # 假设detections是包含所有Tag检测信息的列表
        angles = []

        # 计算每个Tag的角度
        for detection in tags:
            if hasattr(detection, 'id') and hasattr(detection, 'pose'):
                if len(detection.id) == 1:
                    tag_id = detection.id[0]
                    position = detection.pose.pose.pose.position
                    orientation = detection.pose.pose.pose.orientation

                    # 计算角度
                    angle = self.calculate_angle(position)
                    angles.append((angle, detection, tag_id, position, orientation))

        # 根据角度对Tag进行分类
        if angles:
            front_tag = min(angles, key=lambda x: abs(x[0]))[1]
            left_tags = [tag for angle, tag, _, _, _ in angles if angle > 0]
            right_tags = [tag for angle, tag, _, _, _ in angles if angle < 0]

            # 更新JLocation对象中的信息
            self.jlocation.set_front(front_tag)
            self.jlocation.set_left_list(left_tags)
            self.jlocation.set_right_list(right_tags)

    def calculate_angle(self, orientation):
        # 将四元数转换为欧拉角
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        r = Rotation.from_quat(quaternion)
        euler = r.as_euler('xyz', degrees=True)
        # 返回绕y轴的旋转角度???
        return euler[1]

    def get_camera_data(self):
        return self.camera_data

    def get_jlocation(self):
        return self.jlocation


class JData_Collection():
    def __init__(self):
        self.imu_collector = Imu_Data_Collection()
        self.camera_collector = Camera_Data_Collection()

    # 四元数转换为欧拉角
    def quaternion_to_euler(self, quaternion):
        r = Rotation.from_quat(quaternion)
        euler = r.as_euler('xyz', degrees=True)
        return euler

    # 从IMU数据中获取欧拉角
    def get_imu_orientation_euler(self):
        imu_data = self.imu_collector.get_imu_data()

        # 错误的代码
        quaternion = [imu_data.magnetic_field.x(), imu_data.magnetic_field.y(), imu_data.magnetic_field.z(), 1.0]  # 假设 w = 1.0, 需要确认
        return self.quaternion_to_euler(quaternion)

    # 从Apriltag检测数据中获取欧拉角
    def get_apriltag_orientation_euler(self):
        jlocation = self.camera_collector.get_jlocation()
        quaternion = [jlocation.front.orientation.x(), jlocation.front.orientation.y(), jlocation.front.orientation.z(), jlocation.front.orientation.w()]
        return self.quaternion_to_euler(quaternion)

    # 更新数据
    def update(self):
        # 获取IMU和相机数据的欧拉角
        imu_euler = self.get_imu_orientation_euler()
        apriltag_euler = self.get_apriltag_orientation_euler()

        # 计算相机法线与Apriltag法线之间的夹角
        angle_diff = [imu - apriltag for imu, apriltag in zip(imu_euler, apriltag_euler)]

        rospy.loginfo(f"IMU Euler angles: {imu_euler}")
        rospy.loginfo(f"Apriltag Euler angles: {apriltag_euler}")
        rospy.loginfo(f"Angle difference: {angle_diff}")

    # 更新属性数据和return数据
    # 由近到远给两边的墙的数组排序