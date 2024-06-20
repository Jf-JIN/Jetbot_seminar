
'''
该类是自定义的数据类，主要关于小车的物理信息, 如距离, 速度, 加速度等
主要接口: 
JLocation:          用于存储Apriltag信息和IMU信息
    读取: 
        left_list   -> list[JApril_Tag_Info]        左侧Apriltag数据
        right_list  -> list[JApril_Tag_Info]        右侧Apriltag数据
        front       -> JApril_Tag_Info              正前Apriltag数据
        imu         -> JImu_Info                    IMU数据
    写入: 
        set_left_list(left_list: list | tuple)    写入左侧Apriltag数据
        set_right_list(right_list: list | tuple)  写入右侧Apriltag数据
        set_front(front: JApril_Tag_Info)         写入正前Apriltag数据
        set_imu(imu: JImu_Info)                   写入IMU数据
        copy_JLocation -> JLocation               深度复制JLocation

JFront: 
    读取:
        front     -> JApril_Tag_Info  正前方位置信息
        left      -> JApril_Tag_Info  前方最近左侧横版位置信息
        right     -> JApril_Tag_Info  前方最近右侧横版位置信息
    写入: 
        set_front(front: JApril_Tag_Info)   写入正前方位置信息
        set_left(left: JApril_Tag_Info)     写入前方最近左侧横版位置信息
        set_right(right: JApril_Tag_Info)   写入前方最近右侧横版位置信息

JImu_Info:          用于存储IMU信息
    读取:   
        orientation            -> JOrientation                  方向
        velocity               -> JImu_Velocity                 速度
        angular_velocity       -> JImu_Angular_Velocity         角速度
        acceleration           -> JImu_Acceleration             加速度
        angular_acceleration   -> JImu_Angular_Acceleration     角加速度
        magnetic_field         -> JImu_Magnetic_Field           磁场
    写入:  
        set_orientation(self, orientation_list: Union[list, tuple])         写入角度
        set_velocity(velocity_list: list | tuple)                           写入速度
        set_angular_velocity(angular_velocity_list: list | tuple)           写入角速度
        set_acceleration(acceleration_list: list | tuple)                   写入加速度
        set_angular_acceleration(angular_acceleration_list: list | tuple)   写入角加速度
        set_magnetic_field(magnetic_field_list: list | tuple)               写入磁场

JApril_Tag_Info:    用于存储Apriltag信息
    读取: 
        distance       -> JDistance     距离
        orientation    -> JOrientation  方向
        id             -> list          id列表
    写入: 
        set_distance(distance_list: list | tuple)           写入距离
        set_orientation(orientation_list: list | tuple)     写入方向
        set_id(id: list)                                    写入id列表

重要:  以上的所有信息的最终访问参数x,y,z, 必须使用方法x(), y(), z()

调用示例：
a = JLocation()                     实例化数据类
bb = JApril_Tag_Info()              实例化Apriltag类
cc = JApril_Tag_Info()              实例化Apriltag类
bb.set_distance([1, 1, 1])          写入数据
cc.set_distance([2, 2, 2])          写入数据
a.set_left_list([bb, cc])           写入数据
d = a.left_list[1].distance.x()     读取左侧列表第2个元素的距离中的x数值
e = a.left_list                     读取左侧列表
'''

from typing import Union
import copy

# 坐标系基础类


class JCoordinate():
    def __init__(self) -> None:
        self.__x = None
        self.__y = None
        self.__z = None

    # 向Coordinate的 x 属性赋值
    def set_x(self, x: Union[float, int]) -> None:
        # 如果输入不是浮点数或者整数，则触发报错
        if not isinstance(x, (float, int)):
            raise ValueError('x值必须为浮点数或者整数 !')
        self.__x = x

    # 向Coordinate的 y 属性赋值
    def set_y(self, y: Union[float, int]) -> None:
        if not isinstance(y, (float, int)):
            raise ValueError('y值必须为浮点数或者整数 !')
        self.__y = y

    # 向Coordinate的 z 属性赋值
    def set_z(self, z: Union[float, int]) -> None:
        if not isinstance(z, (float, int)):
            raise ValueError('z值必须为浮点数或者整数 !')
        self.__z = z

    # 获取x轴数据
    def x(self) -> Union[float, int]:
        return self.__x

    # 获取y轴数据
    def y(self) -> Union[float, int]:
        return self.__y

    # 获取z轴数据
    def z(self) -> Union[float, int]:
        return self.__z


# 距离信息类
class JDistance(JCoordinate):
    pass


# 方向信息类
class JOrientation(JCoordinate):
    pass


# 从Imu传感器获取的速度类
class JImu_Velocity(JCoordinate):
    pass


# 从Imu传感器获取的角速度类
class JImu_Angular_Velocity(JCoordinate):
    pass


# 从Imu传感器获取的加速度类
class JImu_Acceleration(JCoordinate):
    pass


# 从Imu传感器获取的加速度类
class JImu_Angular_Acceleration(JCoordinate):
    pass

# 从Imu传感器获取的加速度类


class JImu_Magnetic_Field(JCoordinate):
    pass


# Apriltag信息类
class JApril_Tag_Info():
    def __init__(self):
        self.__distance = JDistance()
        self.__orientation = JOrientation()
        self.__id = []

    # 获取距离信息
    @property
    def distance(self) -> JDistance:
        return self.__distance

    # 获取方向信息
    @property
    def orientation(self) -> JOrientation:
        return self.__orientation

    @property
    def id(self) -> list:
        return self.__id

    # 赋值距离参数
    def set_distance(self, distance_list: Union[list, tuple]) -> None:
        # 如果输入不是元组或是列表，亦或元组/列表中的元素不是浮点数或整数，亦或其元素个数不为3，即非(x,y,z)，则触发报错
        if not isinstance(distance_list, (list, tuple)) or \
                not all([isinstance(i, (float, int)) for i in distance_list]) or \
                len(distance_list) != 3:
            raise ValueError(
                '在 JApril_Tag 中, 方法 set_distance 的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__distance.set_x(distance_list[0])
        self.__distance.set_y(distance_list[1])
        self.__distance.set_z(distance_list[2])

    # 赋值方向参数
    def set_orientation(self, orientation_list: Union[list, tuple]) -> None:
        # 如果输入不是元组或是列表，亦或元组/列表中的元素不是浮点数或整数，则触发报错
        if not isinstance(orientation_list, (list, tuple)) or \
                not all([isinstance(i, (float, int)) for i in orientation_list]) or \
                len(orientation_list) != 3:
            raise ValueError(
                '在 JApril_Tag 中, 方法 set_orientation 的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__orientation.set_x(orientation_list[0])
        self.__orientation.set_y(orientation_list[1])
        self.__orientation.set_z(orientation_list[2])

    # 赋值id参数
    def set_id(self, id: list) -> None:
        # 如果输入不是列表，亦或列表中的元素不是整数，则触发报错
        if not isinstance(id, list):
            raise ValueError(
                '在 JApril_Tag 中, 方法 set_id 的参数必须为列表, 且列表内必须为整数')
        self.__id = id


# Imu信息类
class JImu_Info():
    def __init__(self) -> None:
        self.__orientation = JOrientation()
        self.__imu_velocity = JImu_Velocity()
        self.__imu_angular_velocity = JImu_Angular_Velocity()
        self.__imu_acceleration = JImu_Acceleration()
        self.__imu_angular_acceleration = JImu_Angular_Acceleration()
        self.__magnetic_field = JImu_Magnetic_Field()

    # 获取方向
    @property
    def orientation(self) -> JOrientation:
        return self.__orientation

    # 获取速度
    @property
    def velocity(self) -> JImu_Velocity:
        return self.__imu_velocity

    # 获取角速度
    @property
    def angular_velocity(self) -> JImu_Angular_Velocity:
        return self.__imu_angular_velocity

    # 获取加速度
    @property
    def acceleration(self) -> JImu_Acceleration:
        return self.__imu_acceleration

    # 获取角加速度
    @property
    def angular_acceleration(self) -> JImu_Angular_Acceleration:
        return self.__imu_angular_acceleration

    # 获取磁场
    @property
    def magnetic_field(self) -> JImu_Magnetic_Field:
        return self.__magnetic_field

    # 赋值方向
    def set_orientation(self, orientation_list: Union[list, tuple]):
        if not isinstance(orientation_list, (list, tuple)) or \
                not all([isinstance(i, (float, int)) for i in orientation_list]) or \
                len(orientation_list) != 3:
            raise ValueError(
                '在 JImu_Info 中, 方法 set_orientation 的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__orientation.set_x(orientation_list[0])
        self.__orientation.set_y(orientation_list[1])
        self.__orientation.set_z(orientation_list[2])

    # 赋值速度参数

    def set_velocity(self, velocity_list: Union[list, tuple]) -> None:
        if not isinstance(velocity_list, (list, tuple)) or \
                not all([isinstance(i, (float, int)) for i in velocity_list]) or \
                len(velocity_list) != 3:
            raise ValueError(
                '在 JImu_Info 中, 方法 set_velocity 的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__imu_velocity.set_x(velocity_list[0])
        self.__imu_velocity.set_y(velocity_list[1])
        self.__imu_velocity.set_z(velocity_list[2])

    # 赋值角速度参数
    def set_angular_velocity(self, angular_velocity_list: Union[list, tuple]) -> None:
        if not isinstance(angular_velocity_list, (list, tuple)) or \
                not all([isinstance(i, (float, int)) for i in angular_velocity_list]) or \
                len(angular_velocity_list) != 3:
            raise ValueError(
                '在 JImu_Info 中, 方法 set_angular_velocity 的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__imu_angular_velocity.set_x(angular_velocity_list[0])
        self.__imu_angular_velocity.set_y(angular_velocity_list[1])
        self.__imu_angular_velocity.set_z(angular_velocity_list[2])

    # 赋值加速度参数
    def set_acceleration(self, acceleration_list: Union[list, tuple]) -> None:
        if not isinstance(acceleration_list, (list, tuple)) or \
                not all([isinstance(i, (float, int)) for i in acceleration_list]) or \
                len(acceleration_list) != 3:
            raise ValueError(
                '在 JImu_Info 中, 方法 set_acceleration 的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__imu_acceleration.set_x(acceleration_list[0])
        self.__imu_acceleration.set_y(acceleration_list[1])
        self.__imu_acceleration.set_z(acceleration_list[2])

    # 赋值角加速度参数
    def set_angular_acceleration(self, angular_acceleration_list: Union[list, tuple]) -> None:
        if not isinstance(angular_acceleration_list, (list, tuple)) or \
                not all([isinstance(i, (float, int)) for i in angular_acceleration_list]) or \
                len(angular_acceleration_list) != 3:
            raise ValueError(
                '在 JImu_Info 中, 方法 set_angular_acceleration 的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__imu_angular_acceleration.set_x(angular_acceleration_list[0])
        self.__imu_angular_acceleration.set_y(angular_acceleration_list[1])
        self.__imu_angular_acceleration.set_z(angular_acceleration_list[2])

    # 赋值磁场参数
    def set_magnetic_field(self, magnetic_field_list: Union[list, tuple]) -> None:
        if not isinstance(magnetic_field_list, (list, tuple)) or \
                not all([isinstance(i, (float, int)) for i in magnetic_field_list]) or \
                len(magnetic_field_list) != 3:
            raise ValueError(
                '在 JImu_Info 中, 方法 set_magnetic_field 的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__magnetic_field.set_x(magnetic_field_list[0])
        self.__magnetic_field.set_y(magnetic_field_list[1])
        self.__magnetic_field.set_z(magnetic_field_list[2])

# front类


class JFront():
    def __init__(self) -> None:
        self.__front = JApril_Tag_Info()
        self.__left = JApril_Tag_Info()
        self.__right = JApril_Tag_Info()

    # 获取正前方位置信息
    @property
    def front(self) -> JApril_Tag_Info:
        return self.__front

    # 获取前方最近左侧横版位置信息
    @property
    def left(self) -> JApril_Tag_Info:
        return self.__left

    # 获取前方最近右侧横版位置信息
    @property
    def right(self) -> JApril_Tag_Info:
        return self.__right

    # 赋值前方位置信息
    def set_front(self, front: JApril_Tag_Info) -> None:
        self.__front = front

    # 赋值前方最近左侧横版位置信息
    def set_left(self, left: JApril_Tag_Info) -> None:
        self.__left = left

    # 赋值前方最近右侧横版位置信息
    def set_right(self, right: JApril_Tag_Info) -> None:
        self.__right = right

# 位置信息类


class JLocation():
    def __init__(self) -> None:
        self.__left_list = []
        self.__right_list = []
        self.__front = JFront()
        self.__imu = JImu_Info()

    # 获取左侧位置信息
    @property
    def left_list(self) -> list[JApril_Tag_Info]:
        return self.__left_list

    # 获取右侧信息
    @property
    def right_list(self) -> list[JApril_Tag_Info]:
        return self.__right_list

    # 获取前方信息
    @property
    def front(self) -> JFront:
        return self.__front

    # 获取IMU信息
    @property
    def imu(self) -> JImu_Info:
        return self.__imu

    # 赋值左侧位置信息
    def set_left_list(self, left_list: Union[list, tuple]) -> None:
        if not isinstance(left_list, (list, tuple)) or \
                not all([isinstance(i, JApril_Tag_Info) for i in left_list]):
            raise ValueError(
                '在 JLocation 中, 方法 set_left_list 的参数必须为元组或列表, 且其中元素必须为 JApril_Tag_Info !')
        self.__left_list = left_list

    # 赋值右侧位置信息
    def set_right_list(self, right_list: Union[list, tuple]) -> None:
        if not isinstance(right_list, (list, tuple)) or \
                not all([isinstance(i, JApril_Tag_Info) for i in right_list]):
            raise ValueError(
                '在 JLocation 中, 方法 set_right_list 的参数必须为元组或列表, 且其中元素必须为 JApril_Tag_Info !')
        self.__right_list = right_list

    # 赋值前方位置信息
    def set_front(self, front: JFront) -> None:
        if not isinstance(front, JFront):
            raise ValueError(
                '在 JLocation 中, 方法 set_front 的参数必须为 JFront !')
        self.__front = front

    # 赋值Imu信息
    def set_imu(self, imu: JImu_Info) -> None:
        if not isinstance(imu, JImu_Info):
            raise ValueError(
                '在 JLocation 中, 方法 set_imu 的参数必须为 JImu_Info !')
        self.__imu = imu

    # 深复制JLocation
    def copy_JLocation(self) -> "JLocation":
        return copy.deepcopy(self)
        # location = JLocation()
        # location.front.front.set_distance([self.front.front.distance.x(), self.front.front.distance.y(), self.front.front.distance.z()])
        # location.front.front.set_orientation([self.front.front.orientation.x(), self.front.front.orientation.y(), self.front.front.orientation.z()])
        # location.front.left.set_distance([self.front.left.distance.x(), self.front.left.distance.y(), self.front.left.distance.z()])
        # location.front.left.set_orientation([self.front.left.orientation.x(), self.front.left.orientation.y(), self.front.left.orientation.z()])
        # location.front.right.set_distance([self.front.right.distance.x(), self.front.right.distance.y(), self.front.right.distance.z()])
        # location.front.right.set_orientation([self.front.right.orientation.x(), self.front.right.orientation.y(), self.front.right.orientation.z()])
        # location.left_list = [None, None]
        # location.right_list = [None, None]
        # if self.left_list[0]:
        #     location.left_list[0] = JApril_Tag_Info()
        #     location.left_list[0].set_distance([self.left_list[0].distance.x(), self.left_list[0].distance.y(), self.left_list[0].distance.z()])
        #     location.left_list[0].set_orientation([self.left_list[0].orientation.x(), self.left_list[0].orientation.y(), self.left_list[0].orientation.z()])
        #     location.left_list[0].set_id(self.left_list[0].id)
        # if self.left_list[1]:
        #     location.left_list[1] = JApril_Tag_Info()
        #     location.left_list[1].set_distance([self.left_list[1].distance.x(), self.left_list[1].distance.y(), self.left_list[1].distance.z()])
        #     location.left_list[1].set_orientation([self.left_list[1].orientation.x(), self.left_list[1].orientation.y(), self.left_list[1].orientation.z()])
        #     location.left_list[1].set_id(self.left_list[1].id)
        # if self.right_list[0]:
        #     location.right_list[0] = JApril_Tag_Info()
        #     location.right_list[0].set_distance([self.right_list[0].distance.x(), self.right_list[0].distance.y(), self.right_list[0].distance.z()])
        #     location.right_list[0].set_orientation([self.right_list[0].orientation.x(), self.right_list[0].orientation.y(), self.right_list[0].orientation.z()])
        #     location.right_list[0].set_id(self.right_list[0].id)
        # if self.right_list[1]:
        #     location.right_list[1] = JApril_Tag_Info()
        #     location.right_list[1].set_distance([self.right_list[1].distance.x(), self.right_list[1].distance.y(), self.right_list[1].distance.z()])
        #     location.right_list[1].set_orientation([self.right_list[1].orientation.x(), self.right_list[1].orientation.y(), self.right_list[1].orientation.z()])
        #     location.right_list[1].set_id(self.right_list[1].id)
        # location.imu.orientation.set_x(self.imu.orientation.x())
        # location.imu.orientation.set_y(self.imu.orientation.y())
        # location.imu.orientation.set_z(self.imu.orientation.z())
        # location.imu.velocity.set_x(self.imu.velocity.x())
        # location.imu.velocity.set_y(self.imu.velocity.y())
        # location.imu.velocity.set_z(self.imu.velocity.z())
        # location.imu.angular_velocity.set_x(self.imu.angular_velocity.x())
        # location.imu.angular_velocity.set_y(self.imu.angular_velocity.y())
        # location.imu.angular_velocity.set_z(self.imu.angular_velocity.z())
        # location.imu.acceleration.set_x(self.imu.acceleration.x())
        # location.imu.acceleration.set_y(self.imu.acceleration.y())
        # location.imu.acceleration.set_z(self.imu.acceleration.z())
        # location.imu.angular_acceleration.set_x(self.imu.angular_acceleration.x())
        # location.imu.angular_acceleration.set_y(self.imu.angular_acceleration.y())
        # location.imu.angular_acceleration.set_z(self.imu.angular_acceleration.z())
        # location.imu.magnetic_field.set_x(self.imu.magnetic_field.x())
        # location.imu.magnetic_field.set_y(self.imu.magnetic_field.y())
        # location.imu.magnetic_field.set_z(self.imu.magnetic_field.z())
        # return location
