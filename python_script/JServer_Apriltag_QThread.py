
from PyQt5.QtCore import QThread, pyqtSignal
from JData_Collect_main import *


class JServer_Apriltag_QThread(QThread):
    signal_jlocation_package = pyqtSignal(dict)

    def __init__(self) -> None:
        self.flag_running = True
        self.data_collector = JData_Collection()

    def jlocation_unpack(self, data: JLocation):
        temp = {
            'jlocation_package': {
                'left_list': [None, None],
                'right_list': [None, None],
                'front': {
                    'front': {
                        'distance': [data.front.front.distance.x(), data.front.front.distance.y(), data.front.front.distance.z()],
                        'orientation': [data.front.front.orientation.x(), data.front.front.orientation.y(), data.front.front.orientation.z()],
                        'id': data.front.front.id
                    },
                    'left': {
                        'distance': [data.front.left.distance.x(), data.front.left.distance.y(), data.front.left.distance.z()],
                        'orientation': [data.front.left.orientation.x(), data.front.left.orientation.y(), data.front.left.orientation.z()],
                        'id': data.front.left.id
                    },
                    'right': {
                        'distance': [data.front.right.distance.x(), data.front.right.distance.y(), data.front.right.distance.z()],
                        'orientation': [data.front.right.orientation.x(), data.front.right.orientation.y(), data.front.right.orientation.z()],
                        'id': data.front.right.id
                    }
                },
                'imu': {
                    'orientation': [data.imu.orientation.x(), data.imu.orientation.y(), data.imu.orientation.z()],
                    'velocity': [data.imu.velocity.x(), data.imu.velocity.y(), data.imu.velocity.z()],
                    'angular_velocity': [data.imu.angular_velocity.x(), data.imu.angular_velocity.y(), data.imu.angular_velocity.z()],
                    'acceleration': [data.imu.acceleration.x(), data.imu.acceleration.y(), data.imu.acceleration.z()],
                    'angular_acceleration': [data.imu.angular_acceleration.x(), data.imu.angular_acceleration.y(), data.imu.angular_acceleration.z()],
                    'magnetic_field': [data.imu.magnetic_field.x(), data.imu.magnetic_field.y(), data.imu.magnetic_field.z()]
                }
            }
        }
        if data.left_list[0]:
            l0 = {
                'distance': [data.left_list[0].distance.x(), data.left_list[0].distance.y(), data.left_list[0].distance.z()],
                'orientation': [data.left_list[0].orientation.x(), data.left_list[0].orientation.y(), data.left_list[0].orientation.z()],
                'id': data.left_list[0].id
            }
            temp['jlocation_package']['left_list'][0] = l0
        if data.left_list[1]:
            l1 = {
                'distance': [data.left_list[1].distance.x(), data.left_list[1].distance.y(), data.left_list[1].distance.z()],
                'orientation': [data.left_list[1].orientation.x(), data.left_list[1].orientation.y(), data.left_list[1].orientation.z()],
                'id': data.left_list[1].id
            }
            temp['jlocation_package']['left_list'][0] = l1
        if data.right_list[0]:
            r0 = {
                'distance': [data.right_list[0].distance.x(), data.right_list[0].distance.y(), data.right_list[0].distance.z()],
                'orientation': [data.right_list[0].orientation.x(), data.right_list[0].orientation.y(), data.right_list[0].orientation.z()],
                'id': data.right_list[0].id
            }
            temp['jlocation_package']['left_list'][0] = r0
        if data.right_list[1]:
            r1 = {
                'distance': [data.right_list[1].distance.x(), data.right_list[1].distance.y(), data.right_list[1].distance.z()],
                'orientation': [data.right_list[1].orientation.x(), data.right_list[1].orientation.y(), data.right_list[1].orientation.z()],
                'id': data.right_list[1].id
            }
            temp['jlocation_package']['left_list'][0] = r1
        return temp

    def run(self):
        while self.flag_running:
            data: JLocation = self.data_collector.update_jlocation_all()
            data_send = self.jlocation_unpack(data)
            self.signal_jlocation_package.emit(data_send)

    def stop(self):
        self.flag_running = False
        self.wait()
