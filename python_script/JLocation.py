
'''
该类是自定义的数据类，类主要关于小车的物理信息
'''

from types import UnionType
from typing import Any

# 坐标系基础类


class Coordinate_def():
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
class JDistance_def(Coordinate_def):
    pass

# 方向信息类


class JOrientation_def(Coordinate_def):
    pass


class JApril_Tag_def():
    def __init__(self):
        self.__distance = JDistance_def()
        self.__orientation = JOrientation_def()

    def set_distance(self, distance_list: list | tuple) -> None:
        # 如果输入不是元组或是列表，亦或元组/列表中的元素不是浮点数或整数，亦或其元素个数不为3，即非（x,y,z），则触发报错
        if not isinstance(distance_list, (tuple, list)) or \
                not all([isinstance(i, (float, int)) for i in distance_list]) or \
                len(distance_list) != 3:
            raise ValueError(
                '在JApril_Tag_def中, 方法“set_distance”的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__distance.set_x(distance_list[0])
        self.__distance.set_y(distance_list[1])
        self.__distance.set_z(distance_list[2])

    def set_orientation(self, orientation_list: list | tuple) -> None:
        # 如果输入不是元组或是列表，亦或元组/列表中的元素不是浮点数或整数，则触发报错
        if not isinstance(orientation_list, (tuple, list)) or not all([isinstance(i, (float, int)) for i in orientation_list]):
            raise ValueError(
                '在JApril_Tag_def中, 方法“set_orientation”的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__orientation.set_x(orientation_list[0])
        self.__orientation.set_y(orientation_list[1])
        self.__orientation.set_z(orientation_list[2])

    # 获取距离信息
    def distance(self) -> JDistance_def:
        return self.__distance

    # 获取方向信息
    def orientation(self) -> JOrientation_def:
        return self.__orientation


class Imu_Velocity(Coordinate_def):
    pass


class Imu_Acceleration(Coordinate_def):
    pass


class JLocation_def():
    def __init__(self) -> None:
        self.__left_list = []
        self.__right_list = []
        self.__front = JApril_Tag_def()
        self.__imu_velocity = Imu_Velocity()
        self.__imu_acceleration = Imu_Acceleration()
        self.__imu_acceleration.set_x(12)
        self.a = self.__imu_acceleration.x()
        print(self.a)

    def __eq__(self, value: object) -> bool:
        pass

    def __sizeof__(self) -> int:
        pass

    def __or__(self, value: Any) -> UnionType:
        pass

    def __delattr__(self, name: str) -> None:
        pass


am = JLocation_def()
