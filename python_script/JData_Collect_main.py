
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


class Imu_Data_Collection:
    def __init__(self):
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
            rospy.logerr("Attribute error in mag_callback: %s", e)
        except Exception as e:
            rospy.logerr("Unexpected error in mag_callback: %s", e)

    def get_imu_data(self):
        return self.imu_data

    def get_magnetic_field_data(self):
        return self.magnetic_field_data

    def run(self):
        rospy.spin()  # 保持节点运行？？？？？

    class Camera_Data_Collection:
        def __init__(self):
            rospy.init_node('camera_data_collection_node', anonymous=True)

            # 存储检测到的 Apriltag 信息
            self.apriltag_detections = []

            # 订阅 Apriltag 话题
            rospy.Subscriber("/apriltag_detections", AprilTagDetectionArray, self.apriltag_callback)

        def apriltag_callback(self, data):
            # 清空当前的 Apriltag 数据
            self.apriltag_detections = []

            # 处理每个检测到的 Apriltag
            for detection in data.detections:
                tag_info = {
                    'id': detection.id[0],
                    'position': [
                        detection.pose.pose.pose.position.x,
                        detection.pose.pose.pose.position.y,
                        detection.pose.pose.pose.position.z
                    ],
                    'orientation': [
                        detection.pose.pose.pose.orientation.x,
                        detection.pose.pose.pose.orientation.y,
                        detection.pose.pose.pose.orientation.z,
                        detection.pose.pose.pose.orientation.w
                    ]
                }
                self.apriltag_detections.append(tag_info)

        def get_detections(self):
            return self.apriltag_detections

        def run(self):
            rospy.spin()  # 保持节点运行？？？




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


class Camera_Data_Collection:
    def __init__(self):
        rospy.init_node('camera_data_collection_node', anonymous=True)

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


class JData_Collection:
    def __init__(self):
        rospy.init_node('jdata_collection_node', anonymous=True)

        # 初始化 Camera_Data_Collection 和 Imu_Data_Collection
        self.camera_collector = Camera_Data_Collection()
        self.imu_collector = Imu_Data_Collection()

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

    def update_jlocation_with_camera_data(self):
        detections = self.camera_collector.get_detections()

        # 清空 JLocation 中的 Apriltag 数据
        self.jlocation.set_left_list([])
        self.jlocation.set_right_list([])
        self.jlocation.set_front(None)

        max_distance = -1
        front_tag = None

        for detection in detections:
            tag_info = JApril_Tag_Info()
            tag_info.set_id(detection['id'])
            tag_info.set_distance(detection['position'])
            tag_info.set_orientation(detection['orientation'])

            position = detection['position']
            distance = math.sqrt(position[0] ** 2 + position[1] ** 2 + position[2] ** 2)

            if distance > max_distance:
                max_distance = distance
                front_tag = tag_info

            if position[0] < 0:  # 根据 x 坐标分类
                self.jlocation.left_list.append(tag_info)
            else:
                self.jlocation.right_list.append(tag_info)

        if front_tag:
            self.jlocation.set_front(front_tag)

    def update_jlocation_with_imu_data(self):
        imu_data = self.imu_collector.get_imu_data()
        magnetic_field_data = self.imu_collector.get_magnetic_field_data()

        if imu_data:
            imu_info = JImu_Info()
            imu_info.set_velocity(imu_data["linear_acceleration"])
            imu_info.set_angular_velocity(imu_data["angular_velocity"])
            imu_info.set_orientation(imu_data["orientation"])
            self.jlocation.set_imu(imu_info)

        if magnetic_field_data:
            self.jlocation.set_magnetic_field(magnetic_field_data)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz  运行频率？？？
        while not rospy.is_shutdown():
            self.update_jlocation_with_camera_data()
            self.update_jlocation_with_imu_data()
            rate.sleep()


# class JData_Collection():
#     def __init__(self):
#         self.imu_collector = Imu_Data_Collection()
#         self.camera_collector = Camera_Data_Collection()
#         self.jlocation = JLocation()
#
#     # 四元数转换为欧拉角
#     def quaternion_to_euler(self, quaternion):
#         r = Rotation.from_quat(quaternion)
#         euler = r.as_euler('xyz', degrees=True)
#         return euler
#
#     # 从IMU数据中获取欧拉角
#     def get_imu_orientation_euler(self):
#         imu_data = self.imu_collector.get_imu_data()
#
#         # 错误的代码
#         quaternion = [imu_data.magnetic_field.x(), imu_data.magnetic_field.y(), imu_data.magnetic_field.z(), 1.0]  # 假设 w = 1.0, 需要确认
#         return self.quaternion_to_euler(quaternion)
#
#     # 从Apriltag检测数据中获取欧拉角
#     def get_apriltag_orientation_euler(self):
#         jlocation = self.camera_collector.get_jlocation()
#         quaternion = [jlocation.front.orientation.x(), jlocation.front.orientation.y(), jlocation.front.orientation.z(), jlocation.front.orientation.w()]
#         return self.quaternion_to_euler(quaternion)
#
#     # 更新数据
#     def update(self):
#         # 获取IMU和相机数据的欧拉角
#         imu_euler = self.get_imu_orientation_euler()
#         apriltag_euler = self.get_apriltag_orientation_euler()
#
#         # 计算相机法线与Apriltag法线之间的夹角
#         angle_diff = [imu - apriltag for imu, apriltag in zip(imu_euler, apriltag_euler)]
#
#         rospy.loginfo(f"IMU Euler angles: {imu_euler}")
#         rospy.loginfo(f"Apriltag Euler angles: {apriltag_euler}")
#         rospy.loginfo(f"Angle difference: {angle_diff}")

    # 更新属性数据和return数据
    # 由近到远给两边的墙的数组排序

    # def process_tags(self, data):
    #     tags = data.detections  # 假设detections是包含所有Tag检测信息的列表
    #     angles = []
    #
    #     # 计算每个Tag的角度
    #     for detection in tags:
    #         if hasattr(detection, 'id') and hasattr(detection, 'pose'):
    #             if len(detection.id) == 1:
    #                 tag_id = detection.id[0]
    #                 position = detection.pose.pose.pose.position
    #                 orientation = detection.pose.pose.pose.orientation
    #
    #                 # 计算角度
    #                 angle = self.calculate_angle(position)
    #                 angles.append((angle, detection, tag_id, position, orientation))
    #
    #     # 根据角度对Tag进行分类
    #     if angles:
    #         front_tag = min(angles, key=lambda x: abs(x[0]))[1]
    #         left_tags = [tag for angle, tag, _, _, _ in angles if angle > 0]
    #         right_tags = [tag for angle, tag, _, _, _ in angles if angle < 0]
    #
    #         # 更新JLocation对象中的信息
    #         self.jlocation.set_front(front_tag)
    #         self.jlocation.set_left_list(left_tags)
    #         self.jlocation.set_right_list(right_tags)

    # class Camera_Data_Collection():
    #     def __init__(self):
    #         self.camera_data = None
    #         self.jlocation = JLocation()
    #
    #         rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.camera_callback)
    #
    #     def camera_callback(self, data):
    #         self.camera_data = data
    #         rospy.loginfo("Received /tag_detections message")
    #         self.process_tags(data)
    #
    #     def process_tags(self, data):
    #         tags = data.detections  # 假设detections是包含所有Tag检测信息的列表
    #
    #         left_tags = []
    #         right_tags = []
    #         front_tag = None
    #
    #         for detection in tags:
    #             tag_id = detection.id[0]
    #             position = detection.pose.pose.pose.position
    #             orientation = detection.pose.pose.pose.orientation
    #
    #             apriltag_info = JApril_Tag_Info()
    #             apriltag_info.set_distance([position.x, position.y, position.z])
    #             apriltag_info.set_orientation([orientation.x, orientation.y, orientation.z, orientation.w])
    #             apriltag_info.set_id(tag_id)
    #
    #             # 根据AprilTag的方向将其分配到不同的列表
    #             euler_angle = self.calculate_angle([orientation.x, orientation.y, orientation.z, orientation.w])
    #
    #             if -45 <= euler_angle <= 45:  # 朝向右边
    #                 right_tags.append(apriltag_info)
    #             elif 135 <= euler_angle <= 180 or -180 <= euler_angle <= -135:  # 朝向左边
    #                 left_tags.append(apriltag_info)
    #             else:  # 朝向前方
    #                 front_tag = apriltag_info
    #
    #         self.jlocation.set_left_list(left_tags)
    #         self.jlocation.set_right_list(right_tags)
    #         if front_tag:
    #             self.jlocation.set_front(front_tag)