
'''
该类是自定义的数据类，类主要关于小车的物理信息
'''

from types import UnionType
from typing import Any

# 坐标系基础类
class JCoordinate_def():
    def __init__(self) -> None:
        self.__x = None
        self.__y = None
        self.__z = None

    # 向Coordinate的 x 属性赋值
    def set_x(self, x: float | int) -> None:
        # 如果输入不是浮点数或者整数，则触发报错
        if not isinstance(x, (float, int)):
            raise ValueError('x值必须为浮点数或者整数 !')
        self.__x = x

    # 向Coordinate的 y 属性赋值
    def set_y(self, y: float) -> None:
        if not isinstance(y, (float, int)):
            raise ValueError('y值必须为浮点数或者整数 !')
        self.__y = y

    # 向Coordinate的 z 属性赋值
    def set_z(self, z: float) -> None:
        if not isinstance(z, (float, int)):
            raise ValueError('z值必须为浮点数或者整数 !')
        self.__z = z

    # 获取x轴数据
    def x(self) -> float | int:
        return self.__x

    # 获取y轴数据
    def y(self) -> float | int:
        return self.__y

    # 获取z轴数据
    def z(self) -> float | int:
        return self.__z

# 距离信息类
class JDistance_def(JCoordinate_def):
    pass

# 方向信息类
class JOrientation_def(JCoordinate_def):
    pass

# 从Imu传感器获取的速度类
class JImu_Velocity_def(JCoordinate_def):
    pass

# 从Imu传感器获取的角速度类
class JImu_Angular_Velocity_def(JCoordinate_def):
    pass

# 从Imu传感器获取的加速度类
class JImu_Acceleration_def(JCoordinate_def):
    pass

# 从Imu传感器获取的加速度类
class JImu_Angular_Acceleration_def(JCoordinate_def):
    pass

# Apriltag信息类
class JApril_Tag_Info_def():
    def __init__(self):
        self.__distance = JDistance_def()
        self.__orientation = JOrientation_def()

    # 赋值距离参数
    def set_distance(self, distance_list: list | tuple) -> None:
        # 如果输入不是元组或是列表，亦或元组/列表中的元素不是浮点数或整数，亦或其元素个数不为3，即非（x,y,z），则触发报错
        if not isinstance(distance_list, (tuple, list)) or \
                not all([isinstance(i, (float, int)) for i in distance_list]) or \
                len(distance_list) != 3:
            raise ValueError(
                '在 JApril_Tag_def 中, 方法 set_distance 的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__distance.set_x(distance_list[0])
        self.__distance.set_y(distance_list[1])
        self.__distance.set_z(distance_list[2])

    # 赋值方向参数
    def set_orientation(self, orientation_list: list | tuple) -> None:
        # 如果输入不是元组或是列表，亦或元组/列表中的元素不是浮点数或整数，则触发报错
        if not isinstance(orientation_list, (tuple, list)) or \
                not all([isinstance(i, (float, int)) for i in orientation_list]) or \
                len(orientation_list) != 3:
            raise ValueError(
                '在 JApril_Tag_def 中, 方法 set_orientation 的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__orientation.set_x(orientation_list[0])
        self.__orientation.set_y(orientation_list[1])
        self.__orientation.set_z(orientation_list[2])

    # 获取距离信息
    def distance(self) -> JDistance_def:
        return self.__distance

    # 获取方向信息
    def orientation(self) -> JOrientation_def:
        return self.__orientation

# Imu信息类


class JImu_Info_def():
    def __init__(self) -> None:
        self.__imu_velocity = JImu_Velocity_def()
        self.__imu_angular_velocity = JImu_Angular_Velocity_def()
        self.__imu_acceleration = JImu_Acceleration_def()
        self.__imu_angular_acceleration = JImu_Angular_Acceleration_def()

    # 赋值速度参数
    def set_velocity(self, velocity_list: float | int) -> None:
        if not isinstance(velocity_list, (tuple, list)) or \
                not all([isinstance(i, (float, int)) for i in velocity_list]) or \
                len(velocity_list) != 3:
            raise ValueError(
                '在 JImu_Info_def 中, 方法 set_velocity 的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__imu_velocity.set_x(velocity_list[0])
        self.__imu_velocity.set_y(velocity_list[1])
        self.__imu_velocity.set_z(velocity_list[2])

    # 赋值角速度参数
    def set_angular_velocity(self, angular_velocity_list: float | int) -> None:
        if not isinstance(angular_velocity_list, (tuple, list)) or \
                not all([isinstance(i, (float, int)) for i in angular_velocity_list]) or \
                len(angular_velocity_list) != 3:
            raise ValueError(
                '在 JImu_Info_def 中, 方法 set_angular_velocity 的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__imu_angular_velocity.set_x(angular_velocity_list[0])
        self.__imu_angular_velocity.set_y(angular_velocity_list[1])
        self.__imu_angular_velocity.set_z(angular_velocity_list[2])

    # 赋值加速度参数
    def set_acceleration(self, acceleration_list: float | int) -> None:
        if not isinstance(acceleration_list, (tuple, list)) or \
                not all([isinstance(i, (float, int)) for i in acceleration_list]) or \
                len(acceleration_list) != 3:
            raise ValueError(
                '在 JImu_Info_def 中, 方法 set_acceleration 的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__imu_acceleration.set_x(acceleration_list[0])
        self.__imu_acceleration.set_y(acceleration_list[1])
        self.__imu_acceleration.set_z(acceleration_list[2])

    # 赋值角加速度参数
    def set_angular_acceleration(self, angular_acceleration_list: float | int) -> None:
        if not isinstance(angular_acceleration_list, (tuple, list)) or \
                not all([isinstance(i, (float, int)) for i in angular_acceleration_list]) or \
                len(angular_acceleration_list) != 3:
            raise ValueError(
                '在 JImu_Info_def 中, 方法 set_angular_acceleration 的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__imu_angular_acceleration.set_x(angular_acceleration_list[0])
        self.__imu_angular_acceleration.set_y(angular_acceleration_list[1])
        self.__imu_angular_acceleration.set_z(angular_acceleration_list[2])

    # 获取速度
    def velocity(self) -> JImu_Velocity_def:
        return self.__imu_velocity

    # 获取角速度
    def angular_velocity(self) -> JImu_Angular_Velocity_def:
        return self.__imu_angular_velocity

    # 获取加速度
    def acceleration(self) -> JImu_Acceleration_def:
        return self.__imu_acceleration

    # 获取角加速度
    def angular_acceleration(self) -> JImu_Angular_Acceleration_def:
        return self.__imu_angular_acceleration

# 位置信息类
class JLocation_def():
    def __init__(self) -> None:
        self.__left_list = []
        self.__right_list = []
        self.__front = JApril_Tag_Info_def()
        self.__imu = JImu_Info_def()

    # 赋值左侧位置信息
    def set_left_list(self, left_list: tuple | list) -> None:
        if not isinstance(left_list, (tuple, list)) or \
                not all([isinstance(i, JApril_Tag_Info_def) for i in left_list]):
            raise ValueError(
                '在 JLocation_def 中, 方法 set_left_list 的参数必须为元组或列表, 且其中元素必须为 JApril_Tag_Info_def !')
        self.__left_list = left_list

    # 赋值右侧位置信息
    def set_right_list(self, right_list: tuple | list) -> None:
        if not isinstance(right_list, (tuple, list)) or \
                not all([isinstance(i, JApril_Tag_Info_def) for i in right_list]):
            raise ValueError(
                '在 JLocation_def 中, 方法 set_right_list 的参数必须为元组或列表, 且其中元素必须为 JApril_Tag_Info_def !')
        self.__right_list = right_list

    # 赋值前方位置信息
    def set_front(self, front: JApril_Tag_Info_def) -> None:
        if not isinstance(front, JApril_Tag_Info_def):
            raise ValueError(
                '在 JLocation_def 中, 方法 set_front 的参数必须为 JApril_Tag_Info_def !')
        self.__front = front

    # 赋值Imu信息
    def set_imu(self, imu: JImu_Info_def) -> None:
        if not isinstance(imu, JImu_Info_def):
            raise ValueError(
                '在 JLocation_def 中, 方法 set_imu 的参数必须为 JApril_Tag_Info_def !')
        self.__imu = imu

    # 获取左侧位置信息
    def left_list(self, index:int|None = None) -> list:
        if index or index == 0:
            return self.__left_list[index]
        else:
            return self.__left_list

    # 获取右侧信息
    def right_list(self, index:int|None = None) -> list:
        if index or index == 0:
            return self.__right_list[index]
        else:
            return self.__right_list

    # 获取前方信息
    def front(self) -> JApril_Tag_Info_def:
        return self.__front

    def imu(self) -> JImu_Info_def:
        return self.__imu

a = JLocation_def()
b = JApril_Tag_Info_def()
c = JApril_Tag_Info_def()
b.set_distance([1, 1, 1])
c.set_distance([2, 2, 2])
a.set_left_list([b, c])
d = a.left_list(0)
e = a.left_list(1)
f =a.left_list()
g = a.left_list(0).distance().x()
print(d)
print(e)
print(f)
print(g)