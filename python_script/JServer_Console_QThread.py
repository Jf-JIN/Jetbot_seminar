
from PyQt5.QtCore import QThread, pyqtSignal


import re
import os
import pty
import time
import signal
import psutil
import traceback
import functools
import subprocess


class JConsole_Read_QThread(QThread):
    signal_console_line = pyqtSignal(dict)
    signal_finished = pyqtSignal()
    signal_error_output = pyqtSignal(dict)
    signal_running_flag = pyqtSignal(bool)

    def __init__(self, flag_widget, master_fd) -> None:
        super().__init__()
        self.master_fd = master_fd
        self.key_command = flag_widget
        self.flag_running = True

    def convert_to_plain_text(self, output):
        ansi_escape = re.compile(r'\x1B[@-_][0-?]*[ -/]*[@-~]')
        plain_text = ansi_escape.sub('', output)
        return ansi_escape.sub('', plain_text)

    def run(self):
        while self.flag_running:
            try:
                output_line = os.read(self.master_fd, 1024).decode()
                if output_line == '':
                    break
                if output_line:
                    output_line = self.convert_to_plain_text(output_line)
                    # print('[output_line]:\t', repr(output_line))
                    print('[output_line]:\t', output_line)
                    self.signal_console_line.emit({self.key_command: output_line.strip()})
            except OSError as e:
                self.signal_error_output.emit({self.key_command: str(e)})
                break
        self.signal_console_line.emit({self.key_command: '结束所有命令显示'})
        self.signal_running_flag.emit(False)
        self.signal_finished.emit()

    def stop(self):
        self.flag_running = False
        if self.master_fd:
            os.close(self.master_fd)

# 主线程


class JConsole_QThread(QThread):
    signal_console_line = pyqtSignal(dict)
    signal_finished = pyqtSignal()
    signal_error_output = pyqtSignal(dict)
    signal_running_flag = pyqtSignal(bool)
    signal_video_watcher_init = pyqtSignal()

    def __init__(self, key_command: str, value_command: dict):
        super().__init__()
        self.value_command = value_command
        self.formatted_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
        self.signal_running_flag.emit(True)
        self.key_command = key_command
        self.process = None
        self.pid = None

    def signal_extend(self, obj, *items) -> None:
        obj.emit(*items)

    def send_command_to_subprocess(self, command):
        if self.process:
            self.process.stdin.write(command + '\n')
            self.process.stdin.flush()

    def run(self):
        full_command = self.value_command
        try:
            master_fd, slave_fd = pty.openpty()
            self.process = subprocess.Popen(full_command, shell=True, stdin=subprocess.PIPE, stdout=slave_fd, stderr=subprocess.STDOUT, universal_newlines=True)
            os.close(slave_fd)
            self.console_thread = JConsole_Read_QThread(self.key_command, master_fd)
            self.console_thread.signal_console_line.connect(functools.partial(self.signal_extend, self.signal_console_line))
            self.console_thread.signal_finished.connect(functools.partial(self.signal_extend, self.signal_finished))
            self.console_thread.signal_error_output.connect(functools.partial(self.signal_extend, self.signal_error_output))
            self.console_thread.signal_running_flag.connect(functools.partial(self.signal_extend, self.signal_running_flag))
            self.console_thread.start()
            self.signal_video_watcher_init.emit()
            self.console_thread.wait()
            self.signal_finished.emit()
        except subprocess.CalledProcessError as e:
            e = traceback.format_exc()
            self.signal_error_output.emit({self.key_command: f"[JConsole_QThread]: [{self.key_command}] - {self.formatted_time} \n{e}"})
            print(f"[JConsole_QThread]: [{self.key_command}] - {self.formatted_time} \n{e}")
        finally:
            self.cleanup()

    def cleanup(self):
        if self.process:
            try:
                if self.process.stdout:
                    self.process.stdout.close()
                if self.process.stderr:
                    self.process.stderr.close()
                self.process.terminate()
                self.process.wait()
            except Exception as e:
                print(f"[JConsole_QThread]: [{self.key_command}][停止子线程失败] - {self.formatted_time} \n{e}")
            finally:
                self.process = None

    def get_process_id(self):
        for process_element in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = process_element.info['cmdline']
                if self.key_command == 'roscore':
                    if cmdline and any('/opt/ros/noetic/bin/roscore' in item for item in cmdline):
                        print('roscore')
                        return process_element.info['pid']
                elif 'roslaunch' in self.value_command:
                    if cmdline and any('/opt/ros/noetic/bin/roslaunch' in item for item in cmdline):
                        return process_element.info['pid']
                elif 'rosrun' in self.value_command:
                    if cmdline and any(self.value_command in item for item in cmdline):
                        return process_element.info['pid']
                else:
                    return self.process.pid
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        return None

    def stop_command(self):
        pid = self.get_process_id()
        if pid:
            os.kill(pid, signal.SIGTERM)
            self.cleanup()
