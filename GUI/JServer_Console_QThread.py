
from PyQt5.QtCore import QObject, QThread, pyqtSignal

import re
import subprocess
import traceback
import functools
import signal
import select
import time
import os
import pty


class JConsole_Read_QThread(QThread):
    signal_console_line = pyqtSignal(dict)
    signal_finished = pyqtSignal()
    signal_error_output = pyqtSignal(dict)
    signal_running_flag = pyqtSignal(bool)
    signal_roscore_pid = pyqtSignal(str)

    def __init__(self, flag_widget, master_fd) -> None:
        super().__init__()
        self.master_fd = master_fd
        self.flag_widget = flag_widget
        self.flag_running = True

    def convert_to_plain_text(self, output):
        ansi_escape = re.compile(r'\x1B[@-_][0-?]*[ -/]*[@-~]')
        plain_text = ansi_escape.sub('', output)
        return ansi_escape.sub('', plain_text)

    def run(self):
        while self.flag_running:
            try:
                r, _, _ = select.select([self.master_fd], [], [], 0.1)
                if r:
                    output_line = os.read(self.master_fd, 1024).decode(errors='ignore')
                    if output_line == '':
                        break
                    if output_line:
                        output_line = self.convert_to_plain_text(output_line)
                        if 'process[master]: started with pid' in output_line:
                            pid = output_line.split('[')[2].split(']')[0]
                            self.signal_roscore_pid.emit(pid)
                            print('[pid]: ', pid)
                        # print('[output_line]:\t', repr(output_line))
                        print('[output_line]:\t', output_line)
                        self.signal_console_line.emit({self.flag_widget: output_line.strip()})
            except OSError as e:
                self.signal_error_output.emit({self.flag_widget: str(e)})
                break
        self.signal_running_flag.emit(False)
        self.signal_finished.emit()

    def stop(self):
        self.flag_running = False
        if self.master_fd:
            os.close(self.master_fd)


class JConsole_QThread(QThread):
    signal_console_line = pyqtSignal(dict)
    signal_finished = pyqtSignal()
    signal_error_output = pyqtSignal(dict)
    signal_running_flag = pyqtSignal(bool)

    def __init__(self, flag_widget: str, init_command: dict):
        super().__init__()
        self.init_command = init_command
        self.formatted_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
        self.signal_running_flag.emit(True)
        self.flag_widget = flag_widget
        self.process = None
        self.pid = None

    def signal_extend(self, obj, *args) -> None:
        obj.emit(*args)

    def send_command_to_subprocess(self, command):
        if self.process:
            self.process.stdin.write(command + '\n')
            self.process.stdin.flush()

    def set_pid(self, pid):
        self.pid = pid
        print('[pid]: ', self.pid)

    def run(self):
        full_command = self.init_command  # 具体命令
        try:
            master_fd, slave_fd = pty.openpty()
            self.process = subprocess.Popen(full_command, shell=True, stdout=slave_fd, stderr=subprocess.STDOUT, universal_newlines=True)
            os.close(slave_fd)
            self.console_thread = JConsole_Read_QThread(self.flag_widget, master_fd)
            self.console_thread.signal_console_line.connect(functools.partial(self.signal_extend, self.signal_console_line))
            self.console_thread.signal_finished.connect(functools.partial(self.signal_extend, self.signal_finished))
            self.console_thread.signal_error_output.connect(functools.partial(self.signal_extend, self.signal_error_output))
            self.console_thread.signal_running_flag.connect(functools.partial(self.signal_extend, self.signal_running_flag))
            self.console_thread.signal_roscore_pid.connect(self.set_pid)
            self.console_thread.start()
            self.console_thread.wait()  # 等待读取线程完成
            # self.signal_finished.emit()
        except subprocess.CalledProcessError as e:
            e = traceback.format_exc()
            self.signal_error_output.emit({self.flag_widget: f"[JConsole_QThread]: [{self.flag_widget}] - {self.formatted_time} \n{e}"})
            print(f"[JConsole_QThread]: [{self.flag_widget}] - {self.formatted_time} \n{e}")
        finally:
            self.cleanup()

    def cleanup(self):
        if self.process:
            try:
                if self.process.stdout:
                    self.process.stdout.close()  # 关闭标准输出
                if self.process.stderr:
                    self.process.stderr.close()  # 关闭标准错误输出
                self.process.terminate()  # 终止子进程
                self.process.wait()  # 等待子进程终止
            except Exception as e:
                print(f"[JConsole_QThread]: [{self.flag_widget}][停止子线程失败] - {self.formatted_time} \n{e}")
            finally:
                self.process = None

    def stop_command(self):
        print('收到结束程序信号')
        command = f'kill {self.pid}'
        print('kill', command)
        self.process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.cleanup()
