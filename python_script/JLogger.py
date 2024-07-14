
import time
import logging
import os


def JLog(folder_name, log_name):
    __folder_name = folder_name
    __log_name = log_name
    __format = '[%(asctime)s] [%(name)s] [%(funcName)s] - %(levelname)s\n%(message)s\n'
    __time = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime(time.time()))
    __basic_path = os.path.join(os.path.dirname(__file__), 'Log')
    __log_folder_path = os.path.join(__basic_path, __folder_name)
    if not os.path.exists(__log_folder_path):
        os.makedirs(__log_folder_path)
    __logger_path = os.path.join(__log_folder_path, f'[{__log_name}]_log_{__time}.log')
    __logger = logging.getLogger(__log_name)
    __logger.setLevel(logging.DEBUG)

    __file_handler = logging.FileHandler(__logger_path)
    __file_handler.setLevel(logging.DEBUG)
    __file_handler.setFormatter(logging.Formatter(__format, datefmt='%Y-%m-%d %H:%M:%S'))

    __console_handler = logging.StreamHandler()
    __console_handler.setLevel(logging.DEBUG)
    __console_handler.setFormatter(logging.Formatter(__format, datefmt='%Y-%m-%d %H:%M:%S'))

    __logger.handlers.clear()
    __logger.addHandler(__file_handler)
    __logger.addHandler(__console_handler)
    return {
        'debug': __logger.debug,
        'info': __logger.info,
        'warning': __logger.warning,
        'error': __logger.error,
        'critical': __logger.critical
    }
