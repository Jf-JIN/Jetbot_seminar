#!/usr/bin/env python3
'''
该类为电机驱动类,  主要是电机控制, PID或者其他控制算法, 过程姿态矫正, (可能也会有滤波器)
'''

from JData_Collect_main import *
from JMotor_driver import MotorDriver
from JLocation import JLocation

import numpy as np
import math
import time
from JetBot_Parameter_Const import *
from PyQt5.QtCore import QObject, QThread, pyqtSignal

from motor.msg import MotorPWM

logger_dict_motor = JLog('Motor', 'Motor')
log_info = logger_dict_motor['info']
log_error = logger_dict_motor['error']


class PID_Controller(QObject):
    def __init__(self, parent, kp=1, ki=0, kd=0):

        super().__init__()
        self.__kp = kp
        self.__ki = ki
        self.__kd = kd
        self.parent_obj = parent
        self.previous_error = 0.0
        self.integral = 0.0
        self.flag_i_run = True

    def set_kpid(self, data: list):
        self.__kp = data[0]
        self.__ki = data[1]
        self.__kd = data[2]
        log_info(f'[PID_Controller][set_kpid]: 当前 pid 参数为： {self.__kp},  {self.__ki},  {self.__kd}')

    def reset_error_and_integral(self):
        self.previous_error = 0.0
        self.integral = 0.0

    # 缩放 PWM 信号
    def scale_and_limit_pwm(self, control_output):
        scaled_pwm = control_output * MOTOR_MAX_SPEED_RATE
        if min(MOTOR_MAX_SPEED_RATE, scaled_pwm) >= 0:
            result = max(0, min(MOTOR_MAX_SPEED_RATE, scaled_pwm))
        else:
            result = min(0, min(MOTOR_MAX_SPEED_RATE, scaled_pwm))
        return result

    def compute(self, distance_diff, x_value, psi, delta_time):
        if distance_diff >= 0.200:
            distance_diff = 0.200
        if self.flag_i_run and self.integral < 2:
            # self.integral += (x_value - psi) * delta_time
            self.integral += (x_value - psi + 0.01 * distance_diff) * delta_time
        control_left_pi = self.__kp * distance_diff - self.__ki * (self.integral)
        control_right_pi = self.__kp * distance_diff + self.__ki * (self.integral)

        control_left = self.scale_and_limit_pwm(control_left_pi)
        control_right = self.scale_and_limit_pwm(control_right_pi)

        return control_left, control_right, control_left_pi, control_right_pi


class JMotor_Run_QThread(QThread):
    signal_finished = pyqtSignal()
    signal_flag_running = pyqtSignal(bool)

    def __init__(self, parent_object, action_dict) -> None:
        super().__init__()
        # 继承自parent_object
        self.parent_object: QObject = parent_object
        self.server_send = self.parent_object.server_send
        self.motor_driver = self.parent_object.motor_driver
        self.pi_controler = PID_Controller(self, kp=action_dict['pid'][0], ki=action_dict['pid'][1], kd=action_dict['pid'][2])  # 初始化距离PID控制器
        self.pwm_publisher = rospy.Publisher("/motor/pwm_cmd", MotorPWM, queue_size=1)  # 初始化电机转动发布
        self.data_collector = self.parent_object.data_collector

        self.action_dict = action_dict
        self.motor_left_calib = self.action_dict['motor_calib'][0]
        self.motor_right_calib = self.action_dict['motor_calib'][1]
        self.location: JLocation = self.data_collector.update_jlocation_all()
        self.front_apriltag = self.location.front.front
        self.current_angle = self.location.imu.orientation.z()
        self.pid_left = 0
        self.pid_right = 0
        self.turn_wheel_speed_rate = 1
        self.flag_running = False

    def change_flag_running(self, flag):
        self.flag_running = flag
        self.signal_flag_running.emit(flag)

    def update_location(self) -> JLocation:
        self.location = self.data_collector.update_jlocation_all()
        return self.location

    def check_front_id_and_update_apriltag(self, target_id) -> bool:
        self.update_location()
        front_id = self.location.front.front.id
        if target_id == front_id:
            self.front_apriltag = self.location.front.front
            return True
        else:
            self.front_apriltag = None
            log_info(
                f'无前方 Apriltag 码,请检查 JLocation, 当前 front(id, distance) 为:{self.location.front.front.id}, {self.location.front.front.distance.z()}, 目标 id 为: {target_id}')
            return False

    def check_front_tag_and_turn_to_search(self, target_id, turn_time=PATH_SEARCH_APRILTAG_TURN_TIME, turn_degrees=PATH_SEARCH_APRILTAG_DEGREES):
        start_time = time.time()
        dispaly_degrees = 0
        # self.rotation_fixed_time(0.2, -10, radius=0, turn_speed=PATH_SEARCH_APRILTAG_SPEED)
        # time.sleep(1)
        # if self.check_front_id_and_update_apriltag(target_id):
        #     return True
        # self.rotation_fixed_time(turn_time, -turn_degrees, radius=0, turn_speed=PATH_SEARCH_APRILTAG_SPEED)
        # time.sleep(1)
        # if self.check_front_id_and_update_apriltag(target_id):
        #     return True
        right_num = 3
        while not self.check_front_id_and_update_apriltag(target_id) and time.time()-start_time <= (360/turn_degrees * (1+1 + 0.5)*4 * turn_time) and self.flag_running:
            if right_num == 2:
                self.rotation_fixed_time(0.2, -10, radius=0, turn_speed=PATH_SEARCH_APRILTAG_SPEED)
                time.sleep(1)
                right_num += 1
                if self.check_front_id_and_update_apriltag(target_id):
                    return True
            # log_info(f'check_front_tag_and_turn_to_search-Debug: flag_running = {self.flag_running}')
            dispaly_degrees += turn_degrees
            log_info(f'[搜寻]: 正在搜寻 id {target_id}, 当前搜寻时间{time.time()-start_time}, 当前搜寻角度{dispaly_degrees}')
            self.rotation_fixed_time(turn_time, turn_degrees, radius=0, turn_speed=PATH_SEARCH_APRILTAG_SPEED)
            time.sleep(1)
            if self.front_apriltag:
                break
        if self.front_apriltag:
            # log_info(f'[搜寻完成]: 已搜寻到 Apriltag-id {target_id}')
            return True
        else:
            log_info(f'[搜寻失败]: 未能搜寻到 Apriltag-id {target_id}')
            return False

    # def is_target_reached(self, target_tag: JApril_Tag_Info, target_distance):
    #     current_distance = target_tag.distance.z()  # 从目标AprilTag获取当前距离
    #     distance_diff = current_distance - (target_distance - JB_DISTANCE_FROM_CAMERA_TO_CENTER)
    #     flag_reached = distance_diff < PATH_TOLERANCE_DISTANCE
    #     log_info('is_target_reached', current_distance, distance_diff)
    #     if flag_reached:
    #         log_info(f'[已到达]: 已到达 Apriltag-id {target_tag.id}')
    #     return flag_reached

    def calib_pwm_msg(self, left, right, pid_left='0.0001', pid_right='0.0001'):
        pwm_msg = MotorPWM()
        pwm_msg.pwm_left = left + self.motor_left_calib
        pwm_msg.pwm_right = -(right + self.motor_right_calib)
        # if pid_left == 0 or pid_right == 0:
        #     if pid_left == 0:
        #         pwm_msg.pwm_left = 0
        #     if pid_right == 0:
        #         pwm_msg.pwm_right = 0
        return pwm_msg

    def get_motor_volt(self, pwm_obj):
        def calculate(value):
            scaled_value = min(abs(value) * SCALE_RATIO, SCALE_RATIO)
            if value < 0:
                scaled_value = -scaled_value
            return scaled_value * DRIVER_VOLTAGE
        left = calculate(pwm_obj.pwm_left)
        right = calculate(pwm_obj.pwm_right)
        return left, right

    def send_motor_data(self, pwm_msg, distance_diff=' ', psi=' ', delta_time=' '):
        volt_l, volt_r = self.get_motor_volt(pwm_msg)
        text = {'motor_data': {
            'volt_l': volt_l,
            'volt_r': volt_r,
            'pwm_left': pwm_msg.pwm_left,
            'pwm_right': pwm_msg.pwm_right,
            'pid_left': self.pid_left,
            'pid_right': self.pid_right,
            'distance_diff': distance_diff,
            'psi': psi,
            'delta_time': delta_time,
            'integral': self.pi_controler.integral,
            'turn_rate': self.turn_wheel_speed_rate}}
        self.server_send(text)

    def publish_pwm(self, left_default, right_default, distance_diff=' ', psi=' ', delta_time=' ', send_emit=True):
        pwm_msg = self.calib_pwm_msg(left_default, right_default, self.pid_left, self.pid_right)
        self.pwm_publisher.publish(pwm_msg)
        if send_emit:
            self.send_motor_data(pwm_msg, distance_diff, psi, delta_time)

    def rotation_rate_calculation(self, degrees, radius, basic_speed):
        self.turn_wheel_speed_rate = (radius+JB_WHEEL_CENTER_TO_CENTER)/(radius-JB_WHEEL_CENTER_TO_CENTER)
        # 左转
        if (degrees/abs(degrees)) < 0:
            left = basic_speed
            right = basic_speed * self.turn_wheel_speed_rate
        # 右转
        else:
            left = basic_speed * self.turn_wheel_speed_rate
            right = basic_speed
        # log_info(f'{degrees}, {(degrees/abs(degrees))}, {left}, {right}')
        return left, right

    def rotation_fixed_time(self, time_diff, degrees, radius=(0.125 + JB_WHEEL_CENTER_TO_CENTER), turn_speed=PATH_TURN_SPEED, end_speed=0, target_id=[]):
        self.pid_left = None
        self.pid_right = None
        target_id_flag = False
        if len(target_id) > 0:
            target_id_flag = True
        # radius = 0.125 + JB_WHEEL_CENTER_TO_CENTER
        # log_info(degrees)
        left_default, right_default = self.rotation_rate_calculation(degrees=degrees, radius=radius, basic_speed=turn_speed)
        self.publish_pwm(left_default, right_default)
        start_time = time.time()
        self.update_location()
        start_degrees = self.location.imu.orientation.z()
        degrees_diff = 999
        while (time.time() - start_time < time_diff and abs(degrees_diff) > PATH_TOLERANCE_DEGREES) and self.flag_running:
            if target_id_flag:
                self.update_location()
                if self.location.front.front.id and target_id == self.location.front.front.id:
                    break
            self.publish_pwm(left_default, right_default)
            self.update_location()
            self.current_degrees = self.location.imu.orientation.z()
            degrees_diff = (start_degrees + degrees - self.current_degrees + 180) % 360 - 180
            pass
        log_info(f'[完成旋转]: 已完成旋转动作 {time_diff}s, {degrees}°')
        self.publish_pwm(end_speed, end_speed, send_emit=False)
        return True

    def rotation(self, orientation, radius, degrees, turn_speed=PATH_TURN_SPEED, end_speed=0):
        self.pid_left = 0
        self.pid_right = 0
        if orientation == 'r':
            degrees = -degrees
        left_default, right_default = self.rotation_rate_calculation(radius, degrees, turn_speed)
        self.publish_pwm(left_default, right_default)
        self.update_location()
        start_degrees = self.update_location().imu.orientation.z()
        degrees_diff = 999
        while abs(degrees_diff) > PATH_TOLERANCE_DEGREES and self.flag_running:
            self.publish_pwm(left_default, right_default)
            self.update_location()
            self.current_degrees = self.location.imu.orientation.z()
            degrees_diff = (start_degrees + degrees - self.current_degrees + 180) % 360 - 180
            pass
        log_info(f'[完成旋转]: 已完成旋转动作 {degrees}°')
        self.publish_pwm(end_speed, end_speed, send_emit=False)
        return True

    # def straight_pi(self, target_id, target_distance, distance_before_turn=0, end_orientation=None):
    #     self.__last_time = time.time()
    #     self.__current_time = time.time()
    #     while self.check_front_tag_and_turn_to_search(target_id) and not self.is_target_reached(self.front_apriltag, target_distance) and self.flag_running:
    #         current_distance = self.front_apriltag.distance.z()
    #         self.__last_time = self.__current_time
    #         self.__current_time = time.time()
    #         delta_time = self.__current_time - self.__last_time
    #         distance_diff = current_distance - (target_distance + distance_before_turn)  # 当前距离与目标距离之间的差值
    #         psi = math.atan2(self.location.front.front.distance.x(), self.location.front.front.distance.z()) + math.radians(self.location.front.front.orientation.y())
    #         pwm_left, pwm_right, self.pid_left, self.pid_right = self.pi_controler.compute(distance_diff, self.location.front.front.distance.x(), psi, delta_time)
    #         if not self.flag_running:
    #             break
    #         self.publish_pwm(pwm_left, pwm_right, distance_diff, psi, delta_time)
    #         self.pi_controler.reset_error_and_integral()
    #         time.sleep(1/60)
    #         if current_distance <= PATH_DEFAULT_MIN_DISTANCE:
    #             log_info(f'[急停]: 距离墙过近，电机急停')
    #             self.pi_controler.reset_error_and_integral()
    #             self.publish_pwm(0, 0, send_emit=False)
    #             break
    #     if not end_orientation:
    #         log_info(f'[结束]: 清空 PID 数值, 停止电机')
    #         self.pi_controler.reset_error_and_integral()
    #         self.publish_pwm(0, 0, send_emit=False)
    #     else:
    #         log_info(f'[阶段结束]: 直线阶段结束')
    #         self.pi_controler.reset_error_and_integral()

    def straight(self, target_id, target_distance, distance_before_turn=0, end_orientation=None, end_id=[]):
        self.__start_time = time.time()
        target_distance = target_distance - JB_DISTANCE_FROM_CAMERA_TO_CENTER
        while self.check_front_tag_and_turn_to_search(target_id) and self.flag_running:
            current_distance = self.front_apriltag.distance.z()
            distance_diff = current_distance - (target_distance+0.05 + distance_before_turn)  # 当前距离与目标距离之间的差值
            log_info(f'current_distance, distance_diff {current_distance}, {distance_diff}')
            if distance_diff < PATH_TOLERANCE_DISTANCE or not self.flag_running:
                log_info('直线已抵达')
                break
            # if time.time() - self.__start_time <= 0.5 and distance_diff > 0.125:
            #     self.publish_pwm(0.2, 0.2)
            # elif time.time() - self.__start_time <= 0.75 and distance_diff > 0.125:
            #     self.publish_pwm(0.35, 0.35)
            # elif time.time() - self.__start_time <= 1 and distance_diff > 0.125:
            #     self.publish_pwm(0.4, 0.4)
            if distance_diff > 0.125:
                if self.location.front.front.distance.x() > 0.005:
                    self.publish_pwm(0.3, 0.26)
                elif self.location.front.front.distance.x() < -0.005:
                    self.publish_pwm(0.26, 0.3)
                else:
                    self.publish_pwm(0.3, 0.3)
            else:
                self.publish_pwm(0.2, 0.2)
            if current_distance <= PATH_DEFAULT_MIN_DISTANCE:
                log_info('[急停]: 距离墙过近，电机急停')
                self.pi_controler.reset_error_and_integral()
                self.publish_pwm(0, 0, send_emit=False)
                break
        log_info(f'end_orientation {end_orientation}')
        if not end_orientation:
            log_info('[结束]: 清空 PID 数值, 停止电机')
            self.publish_pwm(0, 0, send_emit=False)
        elif end_orientation == 'turn_left':
            log_info('直线接左转')
            log_info('left')
            self.rotation_fixed_time(1.6, 85, radius=0, end_speed=PATH_TURN_SPEED, target_id=end_id)
        elif end_orientation == 'turn_right':
            log_info('right')
            log_info('直线接右转')
            self.rotation_fixed_time(1.6, -85, radius=0, end_speed=PATH_TURN_SPEED, target_id=end_id)
        log_info('[阶段结束]: 直线阶段结束')

    def straight_slow(self, target_id, target_distance, distance_before_turn=0, end_orientation=None, end_id=None):
        target_distance = target_distance - JB_DISTANCE_FROM_CAMERA_TO_CENTER
        # if target_distance < 0.07:
        #     target_distance = 0.07
        log_info(f'缓慢前行 target_distance {target_distance}')
        while self.check_front_tag_and_turn_to_search(target_id) and self.flag_running:
            self.update_location()
            current_distance = self.front_apriltag.distance.z()
            distance_diff = current_distance - (target_distance + distance_before_turn)  # 当前距离与目标距离之间的差值
            log_info(f'current_distance, target_distance, distance_diff {current_distance}, {target_distance}, {distance_diff} {PATH_TOLERANCE_DISTANCE + 0.005}')
            if distance_diff < PATH_TOLERANCE_DISTANCE + 0.01 or not self.flag_running:
                break
            if self.location.front.front.distance.x() > 0.004:
                self.publish_pwm(0.25, 0.2)
            elif self.location.front.front.distance.x() < -0.004:
                self.publish_pwm(0.2, 0.25)
            else:
                self.publish_pwm(0.2, 0.2)
            if current_distance <= PATH_DEFAULT_MIN_DISTANCE:
                log_info('[急停]: 距离墙过近，电机急停')
                self.pi_controler.reset_error_and_integral()
                self.publish_pwm(0, 0, send_emit=False)
                break
        log_info(f'结束缓慢直行 {distance_diff < PATH_TOLERANCE_DISTANCE + 0.005} {self.flag_running} {current_distance <= PATH_DEFAULT_MIN_DISTANCE}')
        self.publish_pwm(0, 0, send_emit=False)

    def straight_return(self, target_id, target_distance, distance_before_turn=0, end_orientation=None):
        log_info(f'开始执行直线后退动作 {self.flag_running}')
        target_distance = target_distance - JB_DISTANCE_FROM_CAMERA_TO_CENTER
        while self.check_front_tag_and_turn_to_search(target_id) and self.flag_running:
            log_info('直线后退循环')
            self.update_location()
            current_distance = self.front_apriltag.distance.z()
            distance_diff = current_distance - (target_distance + distance_before_turn)  # 当前距离与目标距离之间的差值
            log_info(f'current_distance, distance_diff {current_distance}, {distance_diff}')
            if abs(distance_diff) < PATH_TOLERANCE_DISTANCE+0.005 or not self.flag_running:
                break
            if self.location.front.front.orientation.y() > 2:
                self.publish_pwm(-0.2, -0.23)
            elif self.location.front.front.orientation.y() < -2:
                self.publish_pwm(-0.23, -0.2)
            else:
                self.publish_pwm(-0.2, -0.2)
            if current_distance <= PATH_DEFAULT_MIN_DISTANCE:
                log_info('[急停]: 距离墙过近，电机急停')
                self.pi_controler.reset_error_and_integral()
                self.publish_pwm(0, 0, send_emit=False)
                break
        self.publish_pwm(0, 0, send_emit=False)

    def straight_return_fix_time(self, time_diff):
        # target_distance = target_distance - JB_DISTANCE_FROM_CAMERA_TO_CENTER
        start_time = time.time()
        while time.time()-start_time < time_diff and self.flag_running:
            self.publish_pwm(-0.2, -0.2)
        self.publish_pwm(0, 0, send_emit=False)

    def straight_slow_fix_time(self, time_diff):
        start_time = time.time()
        while (time.time() - start_time) < time_diff:
            self.publish_pwm(0.2, 0.2)
        self.publish_pwm(0, 0, send_emit=False)

    # def straight_return_fix_time(self, time_diff):
    #     start_time = time.time()
    #     while (time.time() - start_time) < time_diff:
    #         self.publish_pwm(-0.2, -0.2)
    #     self.publish_pwm(0, 0, send_emit=False)

    # 单路径

    def path_single(self, current_segment):
        log_info(f'[当前路径动作]: {current_segment}')
        mode = current_segment["mode"]
        target_id = current_segment["goal_id"]
        target_distance = current_segment["end_pos"]
        end_orientation = current_segment["end_orientation"]
        end_id = current_segment["end_id"]
        distance_before_turn = 0
        if 'turn' in end_orientation:
            distance_before_turn = PATH_DISTANCE_BEFORE_TURN

        # 处理路径段操作
        if mode == "line":
            self.straight(target_id, target_distance, distance_before_turn, end_orientation, end_id=end_id)
            log_info('[执行路径]: 直线行进')
            # 根据end_orientation进行相应的自转操作
            # if end_orientation == "turn_left" and self.flag_running:
            #     self.rotation_rate_calculation(90, 0, PATH_TURN_SPEED)  # 左转
            # elif end_orientation == "turn_right" and self.flag_running:
            #     self.rotation_rate_calculation(-90, 0, PATH_TURN_SPEED)  # 右转
            # else:
            #     log_info('[停止]: 停止动作, 停止电机')
            #     self.publish_pwm(0, 0, send_emit=False)
            #     self.change_flag_running(False)  # 结束, 停止电机
            #     return

    # 多路径
    def path_list_launch(self, list):
        path_list = copy.deepcopy(list)
        log_info(path_list)
        if len(path_list) < 1:
            log_info('[停止]: 路径为空')
            self.publish_pwm(0, 0, send_emit=False)
            self.change_flag_running(False)  # 结束, 停止电机
            return True
        for index, item in enumerate(path_list):
            if index > 0:
                path_list[index-1]['end_id'] = item['goal_id']
            if index == len(path_list)-1:
                item['end_id'] = []
        log_info(f'运行标志, 路径个数: {self.flag_running} {len(path_list)}')
        while self.flag_running and len(path_list):
            current_segment = path_list.pop(0)
            self.path_single(current_segment)
        log_info('完成路径')
        self.publish_pwm(0, 0, send_emit=False)
        self.change_flag_running(False)  # 结束, 停止电机

    # 停止
    def stop_function(self):
        self.change_flag_running(False)
        log_info(f'self.flag_running {self.flag_running}')
        self.publish_pwm(0, 0, send_emit=False)
        self.motor_driver.destroy_node()
        # self.pi_controler.deleteLater()
        self.signal_finished.emit()
        log_info('电机已停止')

    def stop_function_set_flag(self):
        self.change_flag_running(False)
        self.publish_pwm(0, 0, send_emit=False)
        log_info('标志已经置否, 电机已停止')

    def run(self):
        mode = self.action_dict['mode']
        path = self.action_dict['path']
        direction = self.action_dict['direction']
        time_diff = self.action_dict['time_diff']
        radius = self.action_dict['radius']
        degrees = self.action_dict['degrees']
        target_id = self.action_dict['target_id']
        target_distance = self.action_dict['target_distance']
        end_id = self.action_dict['end_id']
        if mode == 'line':
            self.flag_running = True
            log_info(f'当前模式 {mode}')
            self.straight(target_id, target_distance, end_id=end_id)
        elif mode == 'line_slow':
            self.flag_running = True
            log_info(f'当前模式 {mode}')
            self.straight_slow(target_id, target_distance)
        elif mode == 'line_fix_time':
            self.flag_running = True
            log_info(f'当前模式 {mode}')
            self.straight_slow_fix_time(time_diff)
        elif mode == 'line_return':
            self.flag_running = True
            log_info(f'当前模式 {mode}')
            self.straight_return(target_id, target_distance)
        elif mode == 'line_return_fix_time':
            self.flag_running = True
            log_info(f'当前模式 {mode}')
            self.straight_return_fix_time(time_diff)
        elif mode == 'circle':
            self.flag_running = True
            log_info(f'当前模式 {mode}')
            log_info(degrees)
            self.rotation_fixed_time(time_diff, degrees, radius=radius)
        elif mode == 'path':
            self.flag_running = True
            log_info(f'当前模式 {mode}')
            self.path_list_launch(path)
        else:
            log_info(f'请检查输入的模式mode, 当前为: {mode}')
        self.stop_function()
        log_info('结束run线程')


class JMotor(QObject):
    # signal_set_pi_controler = pyqtSignal(list)
    # signal_set_pid_angle = pyqtSignal(list)
    signal_finished = pyqtSignal(bool)

    def __init__(self, parent, data_collector, server_send) -> None:
        super().__init__()
        self.parent_obj = parent
        self.flag_accept = True     # 运行标签
        self.path_segments = []  # 初始化路径信息
        self.distance_tolerance = PATH_TOLERANCE_DISTANCE   # 允许的距离误差范围
        self.thread_action_dict = {
            'mode': None,
            'path': None,
            'direction': None,
            'time_diff': None,
            'radius': None,
            'degrees': None,
            'target_id': None,
            'target_distance': None,
            'pid': [PID_DISTANCE_DEFAULT_KP, PID_DISTANCE_DEFAULT_KI, PID_DISTANCE_DEFAULT_KD],
            'motor_calib': [MOTOR_LEFT_CALIB, MOTOR_RIGHT_CALIB],
            'end_id': []
        }
        self.server_send = server_send
        self.data_collector: JData_Collection = data_collector      # 初始化传感器数据
        self.motor_driver = MotorDriver()

    def set_flag_accept(self, data: bool):
        self.flag_accept = not data

    def on_thread_finished(self):
        # 清理线程和工作对象
        # log_info(f'开始清理线程')
        self.motor_thread.quit()
        # log_info(f'已退出线程')
        self.motor_thread.wait()
        self.signal_finished.emit(False)
        log_info('线程清理已完成')

    def qthread_init(self, action_dict):
        log_info(action_dict)
        self.motor_thread = JMotor_Run_QThread(self, action_dict)
        self.motor_thread.signal_flag_running.connect(self.set_flag_accept)
        self.motor_thread.signal_finished.connect(self.on_thread_finished)
        self.motor_thread.flag_running = True
        self.motor_thread.start()

        # 实际调用函数
    def run_path(self, data):
        path = data[0]
        pid = data[1]
        motor_calib = data[2]
        action_dict = copy.deepcopy(self.thread_action_dict)
        action_dict['mode'] = 'path'
        action_dict['path'] = path
        action_dict['pid'] = pid
        action_dict['motor_calib'] = motor_calib
        self.qthread_init(action_dict)

    def run_circle(self, time_diff: float, degrees=0, radius=0,  pid=None, direction: str = None, motor_calib=None):
        action_dict = copy.deepcopy(self.thread_action_dict)
        if pid:
            action_dict['pid'] = pid
        if motor_calib:
            action_dict['motor_calib'] = motor_calib
        action_dict['mode'] = 'circle'
        action_dict['direction'] = direction
        action_dict['radius'] = radius
        action_dict['degrees'] = degrees
        action_dict['time_diff'] = time_diff
        self.qthread_init(action_dict)

    def run_line(self, target_id, distance, pid=None, motor_calib=None):
        action_dict = copy.deepcopy(self.thread_action_dict)
        if pid:
            action_dict['pid'] = pid
        if motor_calib:
            action_dict['motor_calib'] = motor_calib
        action_dict['mode'] = 'line'
        action_dict['target_id'] = target_id
        action_dict['target_distance'] = distance
        self.qthread_init(action_dict)

    def run_line_slow(self, target_id, distance, pid=None, motor_calib=None):
        action_dict = copy.deepcopy(self.thread_action_dict)
        if pid:
            action_dict['pid'] = pid
        if motor_calib:
            action_dict['motor_calib'] = motor_calib
        action_dict['mode'] = 'line_slow'
        action_dict['target_id'] = target_id
        action_dict['target_distance'] = distance
        self.qthread_init(action_dict)

    def run_line_slow_fix_time(self, time_diff, pid=None, motor_calib=None):
        action_dict = copy.deepcopy(self.thread_action_dict)
        if pid:
            action_dict['pid'] = pid
        if motor_calib:
            action_dict['motor_calib'] = motor_calib
        action_dict['mode'] = 'line_fix_time'
        action_dict['time_diff'] = time_diff
        self.qthread_init(action_dict)

    def run_return_slow(self, target_id, distance, pid=None, motor_calib=None):
        action_dict = copy.deepcopy(self.thread_action_dict)
        if pid:
            action_dict['pid'] = pid
        if motor_calib:
            action_dict['motor_calib'] = motor_calib
        action_dict['mode'] = 'line_return'
        action_dict['target_id'] = target_id
        action_dict['target_distance'] = distance
        self.qthread_init(action_dict)

    def run_return_fix_time(self, time_diff, pid=None, motor_calib=None):
        action_dict = copy.deepcopy(self.thread_action_dict)
        if pid:
            action_dict['pid'] = pid
        if motor_calib:
            action_dict['motor_calib'] = motor_calib
        action_dict['mode'] = 'line_return_fix_time'
        action_dict['time_diff'] = time_diff
        self.qthread_init(action_dict)

    def stop_motor(self):
        # if hasattr(self, 'motor_thread'):
        #     self.motor_thread.stop_function_set_flag()
        self.motor_thread.stop_function()

    # # 路径矫正
    # def path_correction(self, front_apriltag: JApril_Tag_Info, turn_angle=45):
    #     deviation = front_apriltag.distance.x()
    #     distance = front_apriltag.distance.z()
    #     degrees = front_apriltag.orientation.y()  # 偏转角度
    #     if distance < (WALL_WIDTH+WALL_WIDTH_HALF):
    #         tolerance = PATH_CALIBRATION_TOLERANCE_DISTANCE_NEAR
    #     else:
    #         tolerance = PATH_CALIBRATION_TOLERANCE_DISTANCE_FAR
    #     if abs(deviation) <= tolerance:
    #         return None
    #     if abs(degrees) <= PATH_CALIBRATION_TOLERANCE_ANGLE:
    #         return None
    #     # 前方允许矫正的最小距离
    #     distance_limit = deviation / 2 * math.sin(math.radians(turn_angle)) / (1 - math.cos(math.radians(turn_angle))) + PATH_DEFAULT_MIN_DISTANCE
    #     if distance < distance_limit:
    #         return None
    #     radius = deviation / (4 * (1 - math.cos(math.radians(turn_angle))))
    #     # r>0 为右转, r<0 为左转
    #     rate = (radius + JB_WHEEL_CENTER_TO_CENTER) / (radius - JB_WHEEL_CENTER_TO_CENTER)
    #     return rate, radius, turn_angle
