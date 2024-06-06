'''
该类为电机驱动类， 主要是电机控制，PID或者其他控制算法，过程姿态矫正，(可能也会有滤波器)
'''

from JData_Collect_main import *



# 姿态校正 (内部引用)
class JPose_Correction():
    def __init__(self) -> None:
        pass



# 电机控制算法 (内部引用)
class JMotor_Controller():
    def __init__(self) -> None:
        
            def forward(self, speed=1.0):
                self.left = speed
                self.right = speed

            def backward(self, speed=1.0):
                self.left = -speed
                self.right = -speed

            def left(self, speed=1.0):
                self.left = -speed
                self.right = speed

            def right(self, speed=1.0):
                self.left = speed
                self.right= -speed

            def stop(self):
                self.left = 0
                self.right = 0
        pass



# 滤波器算法 (内部引用)
class JFilter():
    def __init__(self) -> None:
        pass



# 电机驱动主算法 (外部可调用)
class JMotor_Drive():
    def __init__(self, input) -> None: # 这里写类的输入(input)
        pass
