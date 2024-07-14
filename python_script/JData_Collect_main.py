
'''
该类是数据收集类，主要是相机识别二维码，识别两侧前方的Apritag，处理IMU数据，并将数据进行封装打包
'''

from JLocation import *
from scipy.spatial.transform import Rotation
import rospy
from sensor_msgs.msg import Imu, MagneticField
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import CompressedImage
# from motor.msg import MotorPWM, MotorRPM
from PyQt5.QtCore import QObject, pyqtSignal, QThread
import yaml
import numpy as np

SIDE_DIFF = 0.05

logger_dict_collector = JLog('Client', 'JClient')
log_info = logger_dict_collector['info']
log_error = logger_dict_collector['error']


class JCollector_QThread(QThread):

    def __init__(self, imu_callback, mag_callback, apriltag_callback, video_callback) -> None:
        super().__init__()
        self.imu_callback = imu_callback
        self.mag_callback = mag_callback
        self.apriltag_callback = apriltag_callback
        self.video_callback = video_callback

    def run(self):
        rospy.Subscriber("/imu/data_calib", Imu, self.imu_callback)
        rospy.Subscriber("/imu/mag", MagneticField, self.mag_callback)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.apriltag_callback)
        rospy.Subscriber("/tag_detections_image/compressed", CompressedImage, self.video_callback)
        rospy.spin()  # 保持节点运行


class JData_Collection(QObject):
    signal_error_output = pyqtSignal(str)
    video_signal = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.imu_data = None
        self.magnetic_field_data = None
        self.apriltag_data = None
        self.current_position = None
        # 初始化 JLocation 对象
        self.jlocation = JLocation()
        self.collector_thread = JCollector_QThread(self.imu_callback, self.mag_callback, self.apriltag_callback, self.video_callback)
        self.collector_thread.start()

    def imu_callback(self, data):
        try:
            # 从IMU消息中提取数据
            self.imu_data = {
                "linear_acceleration": [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z],
                "angular_velocity": [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z],
                "orientation": [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
            }
            # rospy.loginfo("IMU data received")
        except AttributeError as e:
            rospy.logerr("Attribute error in imu_callback: %s", e)
        except Exception as e:
            rospy.logerr("Unexpected error in imu_callback: %s", e)

    def mag_callback(self, data):
        try:
            # 从磁场消息中提取数据
            self.magnetic_field_data = [data.magnetic_field.x, data.magnetic_field.y, data.magnetic_field.z]

            # rospy.loginfo("Magnetic field data received")
        except AttributeError as e:
            error_text = f'[IMU] [!错误!]: {str(e)}'
            self.signal_error_output.emit(error_text)
            rospy.logerr("Attribute error in mag_callback: %s", e)
        except Exception as e:
            rospy.logerr("Unexpected error in mag_callback: %s", e)

    def apriltag_callback(self, data):
        # 清空当前的 Apriltag 数据
        apriltag_detections = []
        # 处理每个检测到的 Apriltag
        for detection in data.detections:
            if len(detection.id) > 1:
                continue  # 跳过包含多个 ID 的检测信息
            tag_id = int(detection.id[0])
            if tag_id % 4 == 0 or tag_id % 4 == 2:
                tag_id_list = [tag_id, tag_id+1]
            elif tag_id % 4 == 1 or tag_id % 4 == 3:
                tag_id_list = [tag_id-1, tag_id]
            tag_position = detection.pose.pose.pose.position
            tag_orientation = detection.pose.pose.pose.orientation
            tag_info = {
                'id': tag_id_list,
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
            apriltag_detections.append(tag_info)
        self.apriltag_data = apriltag_detections

    def video_callback(self, data):
        self.video_signal.emit(data.data)

    # 四元数转换为欧拉角
    def quaternion_to_euler(self, quaternion):
        # 检查四元数的输入是否为单位四元数
        q = np.array(quaternion, dtype=np.float32, copy=True)
        # 用的是float32，还可以继续调整
        n = np.dot(q, q)
        if n < np.finfo(q.dtype).eps:
            return [0.0, 0.0, 0.0]  # 如果四元数长度接近0，则返回零向量
        else:
            r = Rotation.from_quat(quaternion)
            euler = r.as_euler('xyz', degrees=True)
        return list(euler)

    def update_jlocation_all(self):
        apriltag_data = copy.deepcopy(self.apriltag_data)
        imu_data = copy.deepcopy(self.imu_data)
        magnetic_field_data = copy.deepcopy(self.magnetic_field_data)
        current_position = copy.deepcopy(self.current_position)

        location = JLocation()
        temp_left = JApril_Tag_Info()
        temp_right = JApril_Tag_Info()
        left_list = []
        right_list = []

        max_distance = 0
        if apriltag_data:
            # print(apriltag_data)
            # for vor_item in apriltag_data:
            #     euler_orientation = self.quaternion_to_euler(vor_item['orientation'])
            #     if max_distance <= vor_item['position'][2] and abs(vor_item['position'][0]) < 0.125 and abs(euler_orientation[1]) < 45:
            #         max_distance = vor_item['position'][2]
            for item in apriltag_data:
                euler_orientation = self.quaternion_to_euler(item['orientation'])
                # 置front.front

                # print(item['id'], item['position'][0] < 0, abs(item['position'][0]) < 0.150, (max_distance - SIDE_DIFF) > item['position'][2], euler_orientation[1] < -45, '\n', max_distance, '\n')
                if max_distance <= item['position'][2] and abs(item['position'][0]) < 0.150 and abs(euler_orientation[1]) < 45:
                    max_distance = item['position'][2]
                    location.front.front.distance.set_x(item['position'][0])
                    location.front.front.distance.set_y(item['position'][1])
                    location.front.front.distance.set_z(item['position'][2])
                    location.front.front.orientation.set_x(euler_orientation[0])
                    location.front.front.orientation.set_y(euler_orientation[1])
                    location.front.front.orientation.set_z(euler_orientation[2])
                    location.front.front.set_id(item['id'])
                # 置左侧
                # elif item['position'][0] < 0 and abs(item['position'][0]) < 0.150 and (max_distance - SIDE_DIFF) > item['position'][2] and euler_orientation[1] < -45:
                elif item['position'][0] < 0 and abs(item['position'][0]) < 0.150 and euler_orientation[1] < -45 and abs(item['position'][2]) < 0.275:
                    # print(item['position'][2])
                    # print(item['position'][2] - 0.1)
                    # print('左侧\n')
                    temp_left.distance.set_x(item['position'][0])
                    temp_left.distance.set_y(item['position'][1])
                    temp_left.distance.set_z(item['position'][2])
                    temp_left.orientation.set_x(euler_orientation[0])
                    temp_left.orientation.set_y(euler_orientation[1])
                    temp_left.orientation.set_z(euler_orientation[2])
                    temp_left.set_id(item['id'])
                    if not left_list or (left_list[0].distance.z() < item['position'][2] and len(left_list) == 1 and item['id'] != left_list[0].id):
                        left_list.append(copy.deepcopy(temp_left))
                    elif left_list[0].distance.z() > item['position'][2] and item['id'] == left_list[0].id:
                        left_list[0] = copy.deepcopy(temp_left)
                    elif left_list[0].distance.z() > item['position'][2] and item['id'] != left_list[0].id:
                        left_list.insert(0, copy.deepcopy(temp_left))
                    elif len(left_list) == 2 and left_list[0].distance.z() < item['position'][2] and left_list[1].distance.z() > item['position'][2]:
                        left_list[1] = copy.deepcopy(temp_left)
                # 置右侧
                # elif item['position'][0] > 0 and abs(item['position'][0]) < 0.150 and (max_distance - SIDE_DIFF) > item['position'][2] and euler_orientation[1] > 45:
                elif item['position'][0] > 0 and abs(item['position'][0]) < 0.150 and euler_orientation[1] > 45 and abs(item['position'][2]) < 0.275:
                    # print('右侧\n')
                    temp_right.distance.set_x(item['position'][0])
                    temp_right.distance.set_y(item['position'][1])
                    temp_right.distance.set_z(item['position'][2])
                    temp_right.orientation.set_x(euler_orientation[0])
                    temp_right.orientation.set_y(euler_orientation[1])
                    temp_right.orientation.set_z(euler_orientation[2])
                    temp_right.set_id(item['id'])
                    if not right_list or (right_list[0].distance.z() < item['position'][2] and len(right_list) == 1 and item['id'] != right_list[0].id):
                        right_list.append(copy.deepcopy(temp_right))
                    elif right_list[0].distance.z() > item['position'][2] and item['id'] == right_list[0].id:
                        right_list[0] = copy.deepcopy(temp_right)
                    elif right_list[0].distance.z() > item['position'][2] and item['id'] != right_list[0].id:
                        right_list.insert(0, copy.deepcopy(temp_right))
                    elif len(right_list) == 2 and right_list[0].distance.z() < item['position'][2] and right_list[1].distance.z() > item['position'][2]:
                        right_list[1] = copy.deepcopy(temp_right)
            location.set_left_list(left_list)
            location.set_right_list(right_list)

        if imu_data:
            euler_orientation = self.quaternion_to_euler(imu_data["orientation"])
            imu_info = JImu_Info()
            imu_info.set_acceleration(imu_data["linear_acceleration"])
            imu_info.set_angular_velocity(imu_data["angular_velocity"])
            imu_info.set_orientation(euler_orientation)
            location.set_imu(imu_info)

        if magnetic_field_data:
            location.imu.set_magnetic_field(magnetic_field_data)

        if current_position:
            location.set_current_position(current_position)

        return location
