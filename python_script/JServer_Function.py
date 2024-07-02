from PyQt5.QtWidgets import QMainWindow
# from PyQt5.QtGui import
# from PyQt5.QtCore import

import sys
import yaml
import socket
import rosgraph
import rosnode

from JClient_Server_Video import *
from JClient_Server_Console import *
from JServer_Console_QThread import *
from JServer_Apriltag_QThread import *


class JServer_Function(QMainWindow):
    signal_data_video_send = pyqtSignal(object)
    signal_data_console_send = pyqtSignal(dict)
    signal_close_server_send = pyqtSignal(dict)

    def __init__(self) -> None:
        super().__init__()
        self.parameter_init()
        self.server_init()
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
            print(f"Error: {e}")
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

# **************************************** 命令行功能 ****************************************
    # 解包命令行信号
    @pyqtSlot(dict)
    def console_unpacking(self, data: dict):
        print(f'\n[执行解包]: {data}')
        key_list = [
            'camera_listener', 'jlocation_listener', 'ros_node_init', 'a1_map_yaml_dict', 'a1_path_dict', 'roscore', 'ros_camera', 'ros_rectify',
            'ros_apriltag_detection', 'ros_imu', 'ros_motor', 'ros_algorithm'
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
        print(f'\n[执行解包分类]: {key in data}, {key}, {data}')
        if key == 'camera_listener':
            self.listener_init()
        elif key == 'jlocation_listener':
            self.listener_init()
        elif key == 'ros_node_init':
            rospy.init_node('jdata_collection_node', anonymous=False)
        elif key == 'a1_path_dict':
            # 发送信号, 控制小车
            pass
        elif key == 'a1_map_yaml_dict':
            # print('\n[a1_map_yaml_dict]', data[key])
            loaded_data = data[key]
            if 'standalone_tags' not in loaded_data:
                loaded_data['standalone_tags'] = data[key]['tag_bundles'][0]['layout']
            downloads_path = os.path.join(os.path.expanduser('~'), 'workspace')
            yaml_filename = 'tags.yaml'
            yaml_file_path = os.path.join(downloads_path, yaml_filename)
            with open(yaml_file_path, 'w', encoding='utf-8') as yaml_file:
                yaml.dump(loaded_data, yaml_file, default_flow_style=False, sort_keys=False)
            copy_command = f'echo jetson | sudo -S cp {yaml_file_path} /opt/ros/noetic/share/apriltag_ros/config && sudo -S rm {yaml_file_path}'    # 此处可修改 tags.yaml 的路径
            subprocess.Popen(copy_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
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
        print(f'\n[执行命令行线程初始化]: {data}')
        thread_under_concole_thread = JConsole_QThread(key, data[key])
        thread_under_concole_thread.signal_console_line.connect(self.server_console.send_all)
        thread_under_concole_thread.signal_finished.connect(lambda: self.thread_finish(key))
        # thread_under_concole_thread.signal_video_watcher_init.connect(self.camera_listener_init)
        thread_under_concole_thread.start()
        self.active_threads[key] = thread_under_concole_thread  # 保持对活动线程的引用
        print('\n[命令行线程] [启动之后active_thread]: ', self.active_threads)

    # 结束命令行线程
    def thread_finish(self, key):
        print('\n[命令行线程]: 结束命令行线程')
        sender_thrd = self.sender()
        print(f'\n[命令行线程] [sender类型]: {type(sender_thrd)}')
        sender_thrd.wait()
        sender_thrd.quit()  # 停止线程的事件循环
        print('\n[命令行线程] [删除之前active_thread]: ', self.active_threads)
        if key in self.active_threads:
            self.active_threads.pop(key)  # 删除对象的引用
        else:
            print(f"[命令行线程]: 线程键名 {key} 不存在于 active_threads 中")
        print('\n[命令行线程] [删除之后active_thread]: ', self.active_threads)

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
            print(f'\n[jlocation_listener_init] 未打开ROSCORE: {tag_init_roscore} / 未打开Apriltag识别: {tag_init_apriltag}')
            return
        if not self.is_node_running('/jdata_collection_node') and (self.is_roscore_running() and self.is_node_running('/apriltag_ros_continuous_node')):
            rospy.init_node('jdata_collection_node', anonymous=False)
        if hasattr(self, 'listener_object') or hasattr(self, 'jlocation_listener_Thread'):
            print('\n[listener_init]: 重复初始化，已忽略')
            return
        self.listener_object = JData_Collection()
        self.listener_object.video_signal.connect(self.send_video)
        self.jlocation_listener_Thread = JServer_Apriltag_QThread(self.listener_object)
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
        print('\n[服务器端口]: 关闭服务器')
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
        print('\n[服务器端口] 断开所有客户端socket')

    # 服务器关闭事件
    def closeEvent(self, event):
        super().closeEvent(event)
        self.close_client_socket()
