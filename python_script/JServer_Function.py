from PyQt5.QtWidgets import QMainWindow
# from PyQt5.QtGui import
# from PyQt5.QtCore import

import os
import sys
import yaml
import socket
import rosgraph
import rosnode

from JClient_Server_Video import *
from JClient_Server_Console import *
from JServer_Console_QThread import *
from JServer_Apriltag_QThread import *
from JMotor_Drive_main import *
from JAlgorithm_JPath_Search import *


log_info = logger_dict_server['info']
log_error = logger_dict_server['error']


class JServer_Function(QMainWindow):
    signal_data_video_send = pyqtSignal(object)
    signal_data_console_send = pyqtSignal(dict)
    signal_close_server_send = pyqtSignal(dict)
    signal_set_pid_distance = pyqtSignal(list)
    signal_set_pid_angle = pyqtSignal(list)

    def __init__(self) -> None:
        super().__init__()
        os.system('clear')
        self.parameter_init()
        self.server_init()
        self.motor = None
        # time.sleep(1)

    def parameter_init(self) -> None:
        self.video_clients_list = []
        self.console_clients_list = []
        self.active_threads = {}

    # 获取服务器的ip地址
    def get_local_ip(self) -> str:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
        except Exception as e:
            local_ip = None
            log_error(e)
        finally:
            s.close()
        return local_ip

    # 服务器初始化
    def server_init(self):
        ip = self.get_local_ip()
        self.server_video = Server_Video_QThread(ip)
        self.server_video.signal_connected_list.connect(self.get_video_connected_list)    # 获取 Video 服务器中的客户端列表
        self.server_video.signal_final_close.connect(self.close_server_by_client)          # 接收客户端传来的关闭信号
        self.signal_data_video_send.connect(self.server_video.send_img_all)                 # 向所有客户端发送视频数据
        self.signal_close_server_send.connect(self.server_video.send_all)                   # 向所有客户端发送关闭客户端的信号
        self.server_video.start()

        self.server_console = Server_Console_QThread(ip)
        self.server_console.signal_connected_list.connect(self.get_console_connected_list)      # 获取 Console 服务器中的客户端列表
        self.server_console.signal_data_console_recv.connect(self.console_unpacking)            # 获取客户端发送来的命令行信号
        self.server_console.signal_final_close.connect(self.close_server_by_client)            # 接收客户端传来的关闭信号
        self.signal_close_server_send.connect(self.server_console.send_all)                     # 向所有客户端发送关闭服务端的信号
        self.server_console.start()

    # 服务器终止，急停电机

    def stop_motor_by_server_close(self, flag):
        if flag:
            self.console_unpacking({'motor_action': 'stop'})

# **************************************** 命令行功能 ****************************************
    # 解包命令行信号
    @pyqtSlot(dict)
    def console_unpacking(self, data: dict):
        log_info(f'[执行解包]: {data}')
        key_list = [
            'camera_listener', 'jlocation_listener', 'ros_node_init', 'a1_map_yaml_dict', 'motor_action', 'a1_path_dict', 'roscore', 'ros_camera', 'ros_rectify',
            'ros_apriltag_detection', 'ros_imu', 'ros_imu_calib', 'ros_motor', 'ros_algorithm', 'motor_parameter', 'force_accept', 'a2_map_build'
        ]
        key = None
        for item in key_list:
            if item in data:
                key = item
                break
        self.console_threading_sort(key, data)

    def console_threading_sort(self, key, data):
        if not key:
            return
        log_info(f'[执行解包分类]: {key in data}, {key}, {data}')
        if key == 'camera_listener':
            self.listener_init()
        elif key == 'jlocation_listener':
            self.listener_init()
        elif key == 'ros_node_init':
            if not rospy.core.is_initialized():
                rospy.init_node('jdata_collection_node', anonymous=False)
        elif key == 'motor_action':
            log_info('here')
            data_sub = data[key]
            dict_key = ['mode', 'direction', 'radius', 'degrees', 'id', 'distance']
            if not isinstance(self.motor, JMotor):
                self.motor = JMotor(self, self.data_collector, self.server_console.send_all)
                # self.signal_set_pid_distance.connect(self.motor.signal_set_pid_distance.emit)
                # self.signal_set_pid_angle.connect(self.motor.signal_set_pid_angle.emit)
            # 补全字典缺失
            for item in dict_key:
                if item not in data_sub:
                    data_sub[item] = None
            if data_sub['mode'] == 'stop' and self.motor:
                log_info('stop')
                self.motor.stop_motor()
                log_info('finish')
            if self.motor.flag_accept:
                if data_sub['mode'] == 'circle':
                    log_info('circle')
                    if data_sub['direction'] and (data_sub['radius'] or data_sub['radius'] == 0) and (data_sub['degrees'] or data_sub['degrees'] == 0):
                        self.motor.run_circle(direction=data_sub['direction'], time_diff=data_sub['time_diff'], radius=data_sub['radius'], degrees=data_sub['degrees'])
                    else:
                        log_info(f'[电机驱动-旋转][参数缺失]: direction:{data_sub["direction"]}, radius:{data_sub["radius"]}, degrees:{data_sub["degrees"]}')
                elif data_sub['mode'] == 'line':
                    log_info('line')
                    if data_sub['id'] and data_sub['distance']:
                        self.motor.run_line(id=data_sub['id'], distance=data_sub['distance'])
                    else:
                        log_info(f'[电机驱动-直行][参数缺失]: id:{data_sub["id"]}, distance:{data_sub["distance"]}')
        elif key == 'force_accept':
            if hasattr(self, 'motor'):
                self.motor.flag_accept = True
                log_info(f'[电机驱动-强制接收参数]: 当前 self.motor.flag_accept : {self.motor.flag_accept}')

        elif key == 'a1_path_dict':
            # 发送信号, 控制小车
            if not isinstance(self.motor, JMotor):
                self.motor = JMotor(self, self.data_collector, self.server_console.send_all)
                # self.signal_set_pid_distance.connect(self.motor.signal_set_pid_distance.emit)
                # self.signal_set_pid_angle.connect(self.motor.signal_set_pid_angle.emit)
            if self.motor.flag_accept:
                log_info('[电机驱动]: 执行路径')
                self.motor.run_path(data[key])
        elif key == 'a1_map_yaml_dict':
            loaded_data = data[key]
            current_time_item = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            if 'standalone_tags' not in loaded_data:
                loaded_data['standalone_tags'] = data[key]['tag_bundles'][0]['layout']
            loaded_data = {'time': current_time_item, **loaded_data}
            downloads_path = os.path.join(os.path.expanduser('~'), 'workspace')
            yaml_filename = 'tags.yaml'
            yaml_file_path = os.path.join(downloads_path, yaml_filename)
            with open(yaml_file_path, 'w', encoding='utf-8') as yaml_file:
                yaml.dump(loaded_data, yaml_file, default_flow_style=False, sort_keys=False)
            copy_command = f'echo jetson | sudo -S cp {yaml_file_path} /opt/ros/noetic/share/apriltag_ros/config && sudo -S rm {yaml_file_path}'    # 此处可修改 tags.yaml 的路径
            subprocess.Popen(copy_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        elif key == 'motor_parameter':
            for sub_key, sub_value in data[key].items():
                if sub_key == 'pid_distance' and hasattr(self, 'motor') and hasattr(self.motor, 'motor_thread_object') and hasattr(self.motor.motor_thread_object, 'pid_distance'):
                    self.motor.motor_thread_object.pid_distance.set_kpid(sub_value)
                elif sub_key == 'motor_calib' and hasattr(self, 'motor') and hasattr(self.motor, 'motor_thread_object'):
                    self.motor.motor_thread_object.motor_left_calib = sub_value[0]
                    self.motor.motor_thread_object.motor_right_calib = sub_value[1]
        elif key == 'a2_map_build':
            self.path_search = JPath_Search(self.data_collector, self.server_console.send_all)
            self.path_search.start()
        elif key not in self.active_threads and data[key] != 'Emergency_Stop':
            self.concole_thread(key, data)
        elif key in self.active_threads and data[key] == 'Emergency_Stop':
            self.active_threads[key].stop_command()
        # elif key == 'ros_camera':
        #     pass
        elif key in self.active_threads:
            self.active_threads[key].send_command_to_subprocess(data[key])

    # 构建命令行线程
    def concole_thread(self, key, data):
        log_info(f'[执行命令行线程初始化]: {data}')
        thread_under_concole_thread = JConsole_QThread(key, data[key])
        thread_under_concole_thread.signal_console_line.connect(self.server_console.send_all)
        thread_under_concole_thread.signal_finished.connect(lambda: self.thread_finish(key))
        # thread_under_concole_thread.signal_video_watcher_init.connect(self.camera_listener_init)
        thread_under_concole_thread.start()
        self.active_threads[key] = thread_under_concole_thread  # 保持对活动线程的引用
        log_info('[命令行线程] [启动之后active_thread]: ', self.active_threads)

    # 结束命令行线程
    def thread_finish(self, key):
        log_info('结束命令行线程')
        sender_thrd = self.sender()
        log_info(f'[sender类型]: {type(sender_thrd)}')
        sender_thrd.wait()
        sender_thrd.quit()  # 停止线程的事件循环
        log_info('[删除之前active_thread]: ', self.active_threads)
        if key in self.active_threads:
            self.active_threads.pop(key)  # 删除对象的引用
        else:
            log_info(f'[命令行线程]: 线程键名 {key} 不存在于 active_threads 中')
        log_info('[删除之后active_thread]: ', self.active_threads)

    # 检查ROSCORE是否运行
    def is_roscore_running(self):
        try:
            master = rosgraph.Master('/roscore')
            master.getPid()
            return True
        except socket.error:
            return False

    # 检查节点是否开启
    def is_node_running(self, node_name):
        try:
            node_list = rosnode.get_node_names()
            return node_name in node_list
        except Exception as e:
            return False

    def listener_init(self):
        if not self.is_roscore_running() or not self.is_node_running('/apriltag_ros_continuous_node'):
            tag_init_roscore = not self.is_roscore_running()
            tag_init_apriltag = not self.is_node_running('/apriltag_ros_continuous_node')
            log_info(f'未打开ROSCORE: {tag_init_roscore} / 未打开Apriltag识别: {tag_init_apriltag}')
            return
        if not self.is_node_running('/jdata_collection_node') and (self.is_roscore_running() and self.is_node_running('/apriltag_ros_continuous_node')):
            rospy.init_node('jdata_collection_node', anonymous=False)
        if hasattr(self, 'data_collector') or hasattr(self, 'jlocation_listener_Thread'):
            log_info('重复初始化，已忽略')
            return
        self.data_collector = JData_Collection()
        self.data_collector.video_signal.connect(self.send_video)
        self.jlocation_listener_Thread = JServer_Apriltag_QThread(self.data_collector)
        self.jlocation_listener_Thread.signal_jlocation_package.connect(self.server_console.send_all)
        self.jlocation_listener_Thread.signal_current_location.connect(self.server_console.send_all)
        self.jlocation_listener_Thread.start()

        # 服务器发送视频信号
    def send_video(self, video_data):
        self.signal_data_video_send.emit(video_data)

# **************************************** 关闭服务器 ****************************************

    # 获取视频客户端列表

    def get_video_connected_list(self, clients_list: list) -> None:
        self.video_clients_list = clients_list

    # 获取命令行客户端列表
    def get_console_connected_list(self, clients_list: list) -> None:
        self.console_clients_list = clients_list

    # 从客户端关闭服务器
    def close_server_by_client(self):
        self.close_client_socket()
        # 此处关闭其他的线程
        log_info('关闭服务器')
        sys.exit()

    # 关闭所有的客户端Socket
    def close_client_socket(self):
        if self.video_clients_list:
            for vclient in self.video_clients_list:
                vsocket: socket.socket = vclient[0]
                vsocket.close()
        if self.console_clients_list:
            for cclient in self.console_clients_list:
                csocket: socket.socket = cclient[0]
                csocket.close()
        log_info('断开所有客户端socket')

    # 服务器关闭事件
    def closeEvent(self, event):
        super().closeEvent(event)
        self.close_client_socket()
