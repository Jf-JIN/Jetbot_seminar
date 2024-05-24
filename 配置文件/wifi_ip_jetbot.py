import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import socket
import datetime
import time
import requests

def get_local_ip():
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
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

# net_flag = check_network_connection()
text = '当前时间:\t\t' + str(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")) + '\nip-Address:\t' + get_local_ip()
# print(text)
while not check_network_connection():
    time.sleep(1)
    
send_anonymous_email('jetbotG7@outlook.com', 'Jetbot-Ip-Address', text)
