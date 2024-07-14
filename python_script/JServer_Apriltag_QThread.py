
from PyQt5.QtCore import QThread, pyqtSignal
from JData_Collect_main import *
import time


class JServer_Apriltag_QThread(QThread):
    signal_jlocation_package = pyqtSignal(dict)
    signal_current_location = pyqtSignal(dict)

    def __init__(self, collector) -> None:
        super().__init__()
        self.flag_running = True
        self.data_collector = collector

    def jlocation_unpack(self, data: JLocation):
        temp = {
            'jlocation_package': {
                'left_list': [],
                'right_list': [],
                'front': {
                    'front': {
                        'distance': [data.front.front.distance.xStr(), data.front.front.distance.yStr(), data.front.front.distance.zStr()],
                        'orientation': [data.front.front.orientation.xStr(), data.front.front.orientation.yStr(), data.front.front.orientation.zStr()],
                        'id': data.front.front.id
                    },
                    'left': {
                        'distance': [data.front.left.distance.xStr(), data.front.left.distance.yStr(), data.front.left.distance.zStr()],
                        'orientation': [data.front.left.orientation.xStr(), data.front.left.orientation.yStr(), data.front.left.orientation.zStr()],
                        'id': data.front.left.id
                    },
                    'right': {
                        'distance': [data.front.right.distance.xStr(), data.front.right.distance.yStr(), data.front.right.distance.zStr()],
                        'orientation': [data.front.right.orientation.xStr(), data.front.right.orientation.yStr(), data.front.right.orientation.zStr()],
                        'id': data.front.right.id
                    }
                },
                'imu': {
                    'orientation': [data.imu.orientation.xStr(), data.imu.orientation.yStr(), data.imu.orientation.zStr()],
                    'velocity': [data.imu.velocity.xStr(), data.imu.velocity.yStr(), data.imu.velocity.zStr()],
                    'angular_velocity': [data.imu.angular_velocity.xStr(), data.imu.angular_velocity.yStr(), data.imu.angular_velocity.zStr()],
                    'acceleration': [data.imu.acceleration.xStr(), data.imu.acceleration.yStr(), data.imu.acceleration.zStr()],
                    'angular_acceleration': [data.imu.angular_acceleration.xStr(), data.imu.angular_acceleration.yStr(), data.imu.angular_acceleration.zStr()],
                    'magnetic_field': [data.imu.magnetic_field.xStr(), data.imu.magnetic_field.yStr(), data.imu.magnetic_field.zStr()]
                },
                # 'current_position': data.current_position
            }
        }
        if len(data.left_list) > 0:
            temp['jlocation_package']['left_list'] = [None]
            if data.left_list[0]:
                l0 = {
                    'distance': [data.left_list[0].distance.xStr(), data.left_list[0].distance.yStr(), data.left_list[0].distance.zStr()],
                    'orientation': [data.left_list[0].orientation.xStr(), data.left_list[0].orientation.yStr(), data.left_list[0].orientation.zStr()],
                    'id': data.left_list[0].id
                }
                temp['jlocation_package']['left_list'][0] = l0
            if len(data.left_list) == 2:
                temp['jlocation_package']['left_list'].append(None)
                if data.left_list[1]:
                    l1 = {
                        'distance': [data.left_list[1].distance.xStr(), data.left_list[1].distance.yStr(), data.left_list[1].distance.zStr()],
                        'orientation': [data.left_list[1].orientation.xStr(), data.left_list[1].orientation.yStr(), data.left_list[1].orientation.zStr()],
                        'id': data.left_list[1].id
                    }
                    temp['jlocation_package']['left_list'][1] = l1
        if len(data.right_list) > 0:
            temp['jlocation_package']['right_list'] = [None]
            if data.right_list[0]:
                r0 = {
                    'distance': [data.right_list[0].distance.xStr(), data.right_list[0].distance.yStr(), data.right_list[0].distance.zStr()],
                    'orientation': [data.right_list[0].orientation.xStr(), data.right_list[0].orientation.yStr(), data.right_list[0].orientation.zStr()],
                    'id': data.right_list[0].id
                }
                temp['jlocation_package']['right_list'][0] = r0
            if len(data.right_list) == 2:
                temp['jlocation_package']['right_list'].append(None)
                if data.right_list[1]:
                    r1 = {
                        'distance': [data.right_list[1].distance.xStr(), data.right_list[1].distance.yStr(), data.right_list[1].distance.zStr()],
                        'orientation': [data.right_list[1].orientation.xStr(), data.right_list[1].orientation.yStr(), data.right_list[1].orientation.zStr()],
                        'id': data.right_list[1].id
                    }
                    temp['jlocation_package']['right_list'][1] = r1
        return temp

    def run(self):
        while self.flag_running:
            time.sleep(0.1)
            data: JLocation = self.data_collector.update_jlocation_all()
            data_send = self.jlocation_unpack(data)
            self.signal_jlocation_package.emit(data_send)
            if data.front.front.id and data.front.front.distance.zStr():
                data_current_location = {'current_pos_float': [data.front.front.id, data.front.front.distance.zStr()]}
                self.signal_current_location.emit(data_current_location)
            else:
                data_current_location = {'current_pos_float': None}
                self.signal_current_location.emit(data_current_location)

    def stop(self):
        self.flag_running = False
        self.wait()
