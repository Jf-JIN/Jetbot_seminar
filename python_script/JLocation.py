
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

JImu_Info:          用于存储IMU信息
    读取:   
        velocity               -> JImu_Velocity                 速度
        angular_velocity       -> JImu_Angular_Velocity         角速度
        acceleration           -> JImu_Acceleration             加速度
        angular_acceleration   -> JImu_Angular_Acceleration     角加速度
        magnetic_field         -> JImu_Magnetic_Field           磁场
    写入:  
        set_velocity(velocity_list: list | tuple)                           写入速度
        set_angular_velocity(angular_velocity_list: list | tuple)           写入角速度
        set_acceleration(acceleration_list: list | tuple)                   写入加速度
        set_angular_acceleration(angular_acceleration_list: list | tuple)   写入角加速度
        set_magnetic_field(magnetic_field_list: list | tuple)               写入磁场

JApril_Tag_Info:    用于存储Apriltag信息
    读取: 
        distance       -> JDistance     距离
        orientation    -> JOrientation  方向
        id             -> int           id
    写入: 
        set_distance(distance_list: list | tuple)           写入距离
        set_orientation(orientation_list: list | tuple)     写入方向3
        set_id(id: int)                                     写入id

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


# 坐标系基础类
class JCoordinate():
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
    def set_y(self, y: float | int) -> None:
        if not isinstance(y, (float, int)):
            raise ValueError('y值必须为浮点数或者整数 !')
        self.__y = y

    # 向Coordinate的 z 属性赋值
    def set_z(self, z: float | int) -> None:
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
        self.__id = None

    # 获取距离信息
    @property
    def distance(self) -> JDistance:
        return self.__distance

    # 获取方向信息
    @property
    def orientation(self) -> JOrientation:
        return self.__orientation

    @property
    def id(self) -> int:
        return self.__id

    # 赋值距离参数
    def set_distance(self, distance_list: list | tuple) -> None:
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
    def set_orientation(self, orientation_list: list | tuple) -> None:
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
    def set_id(self, id: int) -> None:
        # 如果输入不是元组或是列表，亦或元组/列表中的元素不是浮点数或整数，亦或其元素个数不为3，即非(x,y,z)，则触发报错
        if not isinstance(id, int):
            raise ValueError(
                '在 JApril_Tag 中, 方法 set_id 的参数必须为整数')
        self.__id = id


# Imu信息类
class JImu_Info():
    def __init__(self) -> None:
        self.__imu_velocity = JImu_Velocity()
        self.__imu_angular_velocity = JImu_Angular_Velocity()
        self.__imu_acceleration = JImu_Acceleration()
        self.__imu_angular_acceleration = JImu_Angular_Acceleration()
        self.__magnetic_field = JImu_Magnetic_Field()

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

    # 赋值速度参数
    def set_velocity(self, velocity_list: list | tuple) -> None:
        if not isinstance(velocity_list, (list, tuple)) or \
                not all([isinstance(i, (float, int)) for i in velocity_list]) or \
                len(velocity_list) != 3:
            raise ValueError(
                '在 JImu_Info 中, 方法 set_velocity 的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__imu_velocity.set_x(velocity_list[0])
        self.__imu_velocity.set_y(velocity_list[1])
        self.__imu_velocity.set_z(velocity_list[2])

    # 赋值角速度参数
    def set_angular_velocity(self, angular_velocity_list: list | tuple) -> None:
        if not isinstance(angular_velocity_list, (list, tuple)) or \
                not all([isinstance(i, (float, int)) for i in angular_velocity_list]) or \
                len(angular_velocity_list) != 3:
            raise ValueError(
                '在 JImu_Info 中, 方法 set_angular_velocity 的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__imu_angular_velocity.set_x(angular_velocity_list[0])
        self.__imu_angular_velocity.set_y(angular_velocity_list[1])
        self.__imu_angular_velocity.set_z(angular_velocity_list[2])

    # 赋值加速度参数
    def set_acceleration(self, acceleration_list: list | tuple) -> None:
        if not isinstance(acceleration_list, (list, tuple)) or \
                not all([isinstance(i, (float, int)) for i in acceleration_list]) or \
                len(acceleration_list) != 3:
            raise ValueError(
                '在 JImu_Info 中, 方法 set_acceleration 的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__imu_acceleration.set_x(acceleration_list[0])
        self.__imu_acceleration.set_y(acceleration_list[1])
        self.__imu_acceleration.set_z(acceleration_list[2])

    # 赋值角加速度参数
    def set_angular_acceleration(self, angular_acceleration_list: list | tuple) -> None:
        if not isinstance(angular_acceleration_list, (list, tuple)) or \
                not all([isinstance(i, (float, int)) for i in angular_acceleration_list]) or \
                len(angular_acceleration_list) != 3:
            raise ValueError(
                '在 JImu_Info 中, 方法 set_angular_acceleration 的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__imu_angular_acceleration.set_x(angular_acceleration_list[0])
        self.__imu_angular_acceleration.set_y(angular_acceleration_list[1])
        self.__imu_angular_acceleration.set_z(angular_acceleration_list[2])

    # 赋值磁场参数
    def set_magnetic_field(self, magnetic_field_list: list | tuple) -> None:
        if not isinstance(magnetic_field_list, (list, tuple)) or \
                not all([isinstance(i, (float, int)) for i in magnetic_field_list]) or \
                len(magnetic_field_list) != 3:
            raise ValueError(
                '在 JImu_Info 中, 方法 set_magnetic_field 的参数必须为元组或列表, 且其中元素必须为浮点数或整数, 且长度必须为3, 即(x, y, z) !')
        self.__magnetic_field.set_x(magnetic_field_list[0])
        self.__magnetic_field.set_y(magnetic_field_list[1])
        self.__magnetic_field.set_z(magnetic_field_list[2])


# 位置信息类
class JLocation():
    def __init__(self) -> None:
        self.__left_list = []
        self.__right_list = []
        self.__front = JApril_Tag_Info()
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
    def front(self) -> JApril_Tag_Info:
        return self.__front

    # 获取IMU信息
    @property
    def imu(self) -> JImu_Info:
        return self.__imu

    # 赋值左侧位置信息
    def set_left_list(self, left_list: list | tuple) -> None:
        if not isinstance(left_list, (list, tuple)) or \
                not all([isinstance(i, JApril_Tag_Info) for i in left_list]):
            raise ValueError(
                '在 JLocation 中, 方法 set_left_list 的参数必须为元组或列表, 且其中元素必须为 JApril_Tag_Info !')
        self.__left_list = left_list

    # 赋值右侧位置信息
    def set_right_list(self, right_list: list | tuple) -> None:
        if not isinstance(right_list, (list, tuple)) or \
                not all([isinstance(i, JApril_Tag_Info) for i in right_list]):
            raise ValueError(
                '在 JLocation 中, 方法 set_right_list 的参数必须为元组或列表, 且其中元素必须为 JApril_Tag_Info !')
        self.__right_list = right_list

    # 赋值前方位置信息
    def set_front(self, front: JApril_Tag_Info) -> None:
        if not isinstance(front, JApril_Tag_Info):
            raise ValueError(
                '在 JLocation 中, 方法 set_front 的参数必须为 JApril_Tag_Info !')
        self.__front = front

    # 赋值Imu信息
    def set_imu(self, imu: JImu_Info) -> None:
        if not isinstance(imu, JImu_Info):
            raise ValueError(
                '在 JLocation 中, 方法 set_imu 的参数必须为 JApril_Tag_Info !')
        self.__imu = imu
