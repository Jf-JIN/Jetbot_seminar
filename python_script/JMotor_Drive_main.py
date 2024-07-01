'''
该类为电机驱动类， 主要是电机控制，PID或者其他控制算法，过程姿态矫正，(可能也会有滤波器)
'''

from JData_Collect_main import *
from motor_driver import set_speed import stop



# 姿态校正 (内部引用)
class JPose_Correction():
    def __init__(self) -> None:

    # 向左微调    
    def fineleft(self, speed=0.5):
        self.left = -speed
        self.right = speed
        set_speed(left, right)
        time.sleep(0.2)
        
        self.left = speed
        self.right = -speed
        set_speed(left, right)
        time.sleep(0.2)
        
        self.left = 0
        self.right = 0
        set_speed(left, right)
        
    # 向右微调
    def fineright(self, speed=0.5):
        self.left = speed
        self.right = -speed
        set_speed(left, right)

        time.sleep(0.2)
        self.left = -speed
        self.right = speed
        set_speed(left, right)
        time.sleep(0.2)

        self.left = 0
        self.right = 0
        set_speed(left, right)





# 电机控制算法 (内部引用)
class JMotor_Controller():
    def __init__(self) -> None:
        
    # 前进    
    def forward(self, speed=1.0):
         self.left = speed
         self.right = speed
         set_speed(left,right)
        
    # 后退
    def backward(self, speed=1.0):
          self.left = -speed
          self.right = -speed
          set_speed(left,right)
        
    # 刹车     
    def check(self, speed=1.0)
        
          self.left = -speed
          self.right = -speed
          set_speed(left,right)
              
          time.sleep(0.2)
                
          self.left = 0
          self.right = 0
          set_speed(left,right)
        
     # 90°左转 需要标定持续时间
     def turn_left(self, speed=1.0):
         self.left = -speed
         self.right = speed
         set_speed(left,right)
         time.sleep(0.2)
         
    # 90°右转 需要标定持续时间
    def turn_right(self, speed=1.0):
         self.left = speed
         self.right= -speed
         set_speed(left,right)
        
     # 停机
     def stop(self):
         self.left = 0
         self.right = 0
         set_speed(left,right)



class P_Controller

    kP = 0.5

    target_speed_left = left
    target_speed_right = right

    current_speed_left = rpm_left
    current_speed_right = rpm_right

    controller_left = kP * (target_speed_left-current_speed_left )
    controller_right = kP * (current_speed_right - current_speed_right)

    left = rpm_left + controller_left
    right = rpm_right + controller_right

    return left
    return right   



# 滤波器算法 (内部引用)
class JFilter():
    def __init__(self) -> None:
        pass



# 电机驱动主算法 (外部可调用)
class JMotor_Drive():
    def __init__(self, input) -> None: # 这里写类的输入(input)
        pass
