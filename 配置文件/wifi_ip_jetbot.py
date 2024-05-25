import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import socket
import datetime
import time
import requests
import platform
import subprocess


def get_wifi_ssid_windows():
    try:
        result = subprocess.run(
            ["netsh", "wlan", "show", "interfaces"],
            capture_output=True, text=True, check=True
        )
        for line in result.stdout.split("\n"):
            if "SSID" in line and "BSSID" not in line:
                ssid = line.split(":")[1].strip()
                return ssid
    except subprocess.CalledProcessError as e:
        print(f"Failed to get Wi-Fi SSID: {e}")
        return None

def get_wifi_ssid_linux():
    try:
        result = subprocess.run(
            ["iwgetid", "-r"],
            capture_output=True, text=True, check=True
        )
        ssid = result.stdout.strip()
        return ssid.decode("utf-8") if isinstance(ssid, bytes) else ssid
    except subprocess.CalledProcessError as e:
        print(f"Failed to get Wi-Fi SSID: {e}")
        return None

def get_wifi_ssid():
    system = platform.system()
    if system == "Windows":
        return get_wifi_ssid_windows()
    elif system == "Linux":
        return get_wifi_ssid_linux()
    else:
        raise NotImplementedError(f"Unsupported OS: {system}")

def get_local_ip():
    try:
        # 创建一个UDP套接字
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 连接到外部地址（不实际发送数据）
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
    except Exception as e:
        local_ip = "Unable to get IP address"
        print(f"Error: {e}")
    finally:
        s.close()
    return local_ip

def check_network_connection():
    try:
        # 尝试进行HTTP请求
        response = requests.get("http://www.google.com", timeout=5)
        # 检查响应状态码
        return response.status_code == 200
    except (requests.ConnectionError, requests.Timeout):
        return False

# 使用示例
if check_network_connection():
    print("Network is connected")
else:
    print("No network connection")


def send_anonymous_email(to_email, subject, body):

    smtp_server = 'smtp.office365.com'  # SMTP服务器
    smtp_port = 587  # 端口
    from_email = 'jetbotG7@outlook.com'  # 邮件地址
    password = 'JetBot_Gruppe7'  

    # 创建邮件
    msg = MIMEMultipart()
    msg['From'] = from_email
    msg['To'] = to_email
    msg['Subject'] = subject

    msg.attach(MIMEText(body, 'plain'))

    try:
        # 连接到SMTP服务器
        server = smtplib.SMTP(smtp_server, smtp_port)
        server.starttls()  # 启用TLS加密
        server.login(from_email, password)
        
        # 发送邮件
        server.sendmail(from_email, to_email, msg.as_string())
        print("Email sent successfully")
    except Exception as e:
        print(f"Failed to send email: {e}")
    finally:
        server.quit()
        
wifi = get_wifi_ssid()
text ='当前时间:\t\t' + str(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")) +\
    '\n当前WIFI:\t\t' + str(wifi) + \
    '\nip-Address:\t' + get_local_ip()
# print(text)
while not check_network_connection():
    time.sleep(1)
    
send_anonymous_email('jetbotG7@outlook.com', 'Jetbot-Ip-Address', text)
