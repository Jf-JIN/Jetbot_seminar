'''
该文件目的是实现以下功能：
1定义基于速度控制手段的具体需求，收集信息，计算出所需转速和姿态控制，转换成PWM发布
2控制部分如何建立节点，建立主题，发布PWM控制信息
3执行部分如何从主题接收控制信息，然后将其输出给小车驱动
（我想直接写反馈控制，本来准备的左转/右转不作为主要目标实现手段，而是作为辅助减少误差的手段）
------------------------------------------------

'''
import rospy
from Adafruit_MotorHAT import Adafruit_MotorHAT
from motor.msg import MotorPWM


#定义位置控制节点，访问位置信息和目标位置信息（假设组员完成了对当前位置信息和目标位置信息主题的建立与发布），并定义函数用于判断当前位置与目标位置是否一致
#该类功能是获取位置信息，获得current_position和target_position的信息，赋值给中间变量，然后
class PositionController:
    def __init__(self):
        rospy.init_node("position_controller", anonymous=True)

        # 订阅当前位置信息主题
        self.position_sub = rospy.Subscriber('/position', Position, self.position_callback)

        # 订阅目标位置信息主题
        self.target_position_sub = rospy.Subscriber('/target_position', Position, self.target_position_callback)

        # 发布反馈信息主题
        self.feedback_pub = rospy.Publisher('/feedback', Bool, queue_size=10)

        # 用于存储当前位置信息
        self.current_position = Position()
        # 用于存储目标位置信息
        self.target_position = Position()
        # 达到目标的容忍范围（epsilon）
        self.epsilon = 0.1

    def position_callback(self, msg):
        # 更新当前位置信息
        self.current_position.x = msg.x
        self.current_position.y = msg.y
        self.current_position.z = msg.z
        # 发布当前位置信息
        rospy.loginfo(
            f"Current position: x={self.current_position.x}, y={self.current_position.y}, z={self.current_position.z}")

        # 检查是否达到目标位置
        self.check_target_reached()

    def target_position_callback(self, msg):
        # 更新目标位置信息
        self.target_position.x = msg.x
        self.target_position.y = msg.y
        self.target_position.z = msg.z
        # 发布目标位置信息
        rospy.loginfo(
            f"Target position: x={self.target_position.x}, y={self.target_position.y}, z={self.target_position.z}")

    def calculate_distance(self):
    # 计算当前位置信息和目标位置信息之间的二位距离。
        dx = self.current_position.x - self.target_position.x
        dy = self.current_position.y - self.target_position.y
        distance = math.sqrt(dx ** 2 + dy ** 2)
        return distance

    # 计算当前位置信息和目标位置信息之间的角度信息。
    def calculate_target_angle(self):
        dx = self.target_position.x - self.current_position.x
        dy = self.target_position.y - self.current_position.y
        angle = math.atan2(dy, dx)
        return angle
    def check_target_reached(self):
        # 计算当前和目标位置的距离
        distance = self.calculate_distance()
        rospy.loginfo(f"Distance to target: {distance}")

        # 判断是否达到目标位置
        reached = distance < self.epsilon

        if reached:
            rospy.loginfo("Target reached!")
            self.send_feedback(True)
        else:
            rospy.loginfo("Target not reached yet.")
            self.send_feedback(False)

    def send_feedback(self, target_reached):
        # 创建Bool消息
        feedback_msg = Bool()
        feedback_msg.data = target_reached

        # 发布反馈消息
        self.feedback_pub.publish(feedback_msg)
        rospy.loginfo(f"Feedback sent: target_reached={feedback_msg.data}")

    def run(self):
        rospy.spin()











#基于坐标信息的前进与反馈，以及如何将这些计算出来的信息上传给PWM主题

class SpeedController:
    def __init__(self):
        rospy.init_node('speed_controller', anonymous=True)#初始化节点
        self.base_speed = 100  # 基础移动速度
        self.stop_distance = 10  # 距离小于该值时进行反向制动
        self.deceleration_distance = 50  # 距离小于该值时开始减速
        self.max_pwm = 255  # PWM最大值
        self.pub_pwm = rospy.Publisher('/motor/pwm_cmd', MotorPWM, queue_size=10)#上传信息到/motor/pwm_cmd主题


    def get_current_angle(self):
        # 这里准备从IMU和apriltag中获取当前角度信息，但还没想好怎么写
        return 0.0

    def publish_pwm(self, left_pwm, right_pwm):
        pwm_msg = MotorPWM()
        pwm_msg.pwm_left = left_pwm
        pwm_msg.pwm_right = right_pwm
        #这种比如MotorPWM()自定义消息类型的实现好像需要在ros catkin里面去定义实现？？？暂时留置这个问题




    def calculate_speeds(self, current_position, target_position):
        # 计算到目标的距离
        distance_to_target = math.sqrt(
            (target_position.x - current_position.x) ** 2 +
            (target_position.y - current_position.y) ** 2
        )

        # 计算目标角度
        dx = target_position.x - current_position.x
        dy = target_position.y - current_position.y
        target_angle = math.atan2(dy, dx)

        # 获取当前角度
        current_angle = self.get_current_angle()

        # 计算需要调整的角度
        angle_difference = target_angle - current_angle
        angle_difference = np.arctan2(np.sin(angle_difference), np.cos(angle_difference))  # 保持在 -pi 到 pi 之间

        # 根据距离调整速度
        if distance_to_target < self.stop_distance:
            # 反向制动
            linear_speed = -self.base_speed
            angular_speed = 0  # 停止转向
        elif distance_to_target < self.deceleration_distance:
            # 减速，比如减速要50开始，但是现在40，当前距离与目标距离比例可以作为减速依据
            linear_speed = self.base_speed * (distance_to_target / self.deceleration_distance)
        else:
            # 非危险距离保持基础移动速度
            linear_speed = self.base_speed

        # 根据角度差调整左右轮的速度
        if abs(angle_difference) > 0.1:  # 角度误差较大时，需要进行转向调整
            angular_speed = X * angle_difference  # X都是调整系数，未定
            left_pwm = X*(linear_speed - angular_speed)
            right_pwm =X*(linear_speed + angular_speed)
        else:
            # 角度误差较小，直行
            left_pwm = linear_speed
            right_pwm = linear_speed

        # 确保PWM在有效范围内
        left_pwm = max(-self.max_pwm, min(left_pwm, self.max_pwm))
        right_pwm = max(-self.max_pwm, min(right_pwm, self.max_pwm))

        # 调整小车的轮速
        self.adjust_wheel_speeds(left_pwm, right_pwm)


    rospy.loginfo(f"Published PWM: Left={pwm_left}, Right={pwm_right}")

    def run(self):
        while not rospy.is_shutdown():
            self.pub_pwm.publish(pwm_msg)  # 计算出来左右驱动PWM信息后，由于.pub_pwm是rospy.publisher类，故可以直接调用publish命令进行驱动的信息发布
            rospy.sleep(0.1)


    #主程序将两个类实例化，从而可以调用POSOTION中的位置信息给SPEED中来用
    def main(self):
        position_controller=PositionController()
        speed_controller = SpeedController()
        speed_controller.calculate_speeds(position_controller.position_callback,position_controller.target_position_callback)

        #让sSpeedController的calculate_speeds方法能够调用PositionController类中的position_callback赋值信息


#基于距离信息的反馈
#基于坐标信息的旋转
#基于角度信息的旋转
#基于APRILTAG左右侧信息的反馈控制
#基于APRILTAG前方信息的反馈控制
#基于IMU信息的角度校准
#基于IMU信息的速度校准





#驱动节点PWM信息的接收与对motor的控制
class MotorDriver:
    MOTOR_LEFT = 1  # 左电机标识
    MOTOR_RIGHT = 2  # 右电机标识

    def __init__(self):
        #初始化节点
        rospy.init_node("motor_driver", anonymous=True)

        self.max_pwm = 255  # 最大PWM值

        # 初始化 Adafruit MotorHAT 驱动器
        self.driver = Adafruit_MotorHAT(i2c_bus=1)

        # 获取电机对象
        self.motors = {
            self.MOTOR_LEFT: self.driver.getMotor(self.MOTOR_LEFT),
            self.MOTOR_RIGHT: self.driver.getMotor(self.MOTOR_RIGHT)
        }

        # 订阅PWM命令主题
        self.sub = rospy.Subscriber("/motor/pwm_cmd", MotorPWM, self._pwm_listener, queue_size=1)

    #对输入的PWM值的进行符号和放大系数上面的调整
    def _set_pwm(self, motor: int, value: float) -> None:

        pwm_value = int(min(max(abs(value) * self.max_pwm, 0.0), self.max_pwm))

        self.motors[motor].setSpeed(pwm_value)

        if value > 0:
            self.motors[motor].run(Adafruit_MotorHAT.BACKWARD)
        elif value < 0:
            self.motors[motor].run(Adafruit_MotorHAT.FORWARD)
        else:
            self.motors[motor].run(Adafruit_MotorHAT.RELEASE)

    def _pwm_listener(self, msg: MotorPWM) -> None:#接受speedcontrol放出的PWM信息，然后对马达进行赋值

        pwm_l = msg.pwm_left
        pwm_r = msg.pwm_right

        self.set_speed(pwm_l, pwm_r)

def main() -> None:

    motor_controller = MotorDriver()
    rospy.loginfo("正在监听 PWM 命令...")
    rospy.spin()

