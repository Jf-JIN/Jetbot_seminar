'''
接口：
JWall:
    读取：
        main_0          -> JApril_Code          正面大码, 向上向左
        main_1          -> JApril_Code          正面小码, 向上向左
        sub_0           -> JApril_Code          背面大码, 向下向右
        sub_1           -> JApril_Code          背面小码, 向下向右
        orientation     ->'H' 或 'V'            墙的方向, 'H' 为水平, 即平行于Y轴 'V'为垂直, 即平行于X轴
        middle          -> JApriltag_Position   墙的中心点
        id_list         -> list                 墙的所有码的 id 列表
        width           -> list                 墙的宽度
        height          -> list                 墙的高度
        thickness       -> list                 墙的厚度
        big_tag_size    -> list                 墙的大码大小
        small_tag_size  -> list                 墙的小码大小
        topleft         -> tuple                左上顶点
        topright        -> tuple                右上顶点
        bottomleft      -> tuple                左下顶点
        bottomright     -> tuple                右下顶点
        leftside        -> float                左边
        rightside       -> float                右边
        topside         -> float                上边
        bottomside      -> float                下边
        identifier      ->int|str               标识符, 用于算法区分墙与路径, 墙中心的本体
        sub_indentifier ->int|str               标识符, 用于算法区分墙与路径, 墙的延伸, 包括连接
JWalls:
    写入：
        build_JWall_list_from_yaml (yaml_data: yaml) -> list[JWall]
        该方法为从yaml文件中读取所有的Code码信息, 将Code码分类, 并打包为JWall类,
        将信息记录在JWall中, 返回一个含有所有JWall类实体的列表
'''

from JAlgorithm_JWall_Const import *
from JLocation import *
import yaml

# 基础类 Apriltag 的位置 注意 x y z 值只能使用方法调用


class JApriltag_Position():
    def __init__(self) -> None:
        self.__x = None
        self.__y = None
        self.__z = None

    def x(self) -> float:
        return self.__x

    def y(self) -> float:
        return self.__y

    def z(self) -> float:
        return self.__z

    def set_x(self, x: float) -> None:
        self.__x = x

    def set_y(self, y: float) -> None:
        self.__y = y

    def set_z(self, z: float) -> None:
        self.__z = z

# 基础类 Apriltag 的方向 注意 qw qx qy qz 值只能使用方法调用


class JApriltag_Orientation():
    def __init__(self) -> None:
        self.__qw = None
        self.__qx = None
        self.__qy = None
        self.__qz = None

    def qw(self) -> float:
        return self.__qw

    def qx(self) -> float:
        return self.__qx

    def qy(self) -> float:
        return self.__qy

    def qz(self) -> float:
        return self.__qz

    def set_qw(self, qw: float) -> None:
        self.__qw = qw

    def set_qx(self, qx: float) -> None:
        self.__qx = qx

    def set_qy(self, qy: float) -> None:
        self.__qy = qy

    def set_qz(self, qz: float) -> None:
        self.__qz = qz

# JApril_Code 为基类 JApriltag_Position 与 JApriltag_Orientation 的扩展类


class JApril_Code():
    def __init__(self) -> None:
        self.__id = None
        self.__pos = JApriltag_Position()
        self.__ori = JApriltag_Orientation()

    @property
    def id(self) -> int:
        return self.__id

    @property
    def pos(self) -> JApriltag_Position:
        return self.__pos

    @property
    def ori(self) -> JApriltag_Orientation:
        return self.__ori

    def set_id(self, id: int) -> None:
        self.__id = id

    def set_pos(self, xyz_list: list[float]) -> None:
        self.pos.set_x(xyz_list[0])
        self.pos.set_y(xyz_list[1])
        self.pos.set_z(xyz_list[2])

    def set_ori(self, wxyz_list: list[float]) -> None:
        self.__ori.set_qw(wxyz_list[0])
        self.__ori.set_qx(wxyz_list[1])
        self.__ori.set_qy(wxyz_list[2])
        self.__ori.set_qz(wxyz_list[3])

# 主要调取类 JWall 内含4个 Apriltag 码, 是墙的类


class JWall():
    def __init__(self):
        self.__main_0 = JApril_Code()
        self.__main_1 = JApril_Code()
        self.__sub_0 = JApril_Code()
        self.__sub_1 = JApril_Code()
        self.__orientation: str = ''
        self.__middle = JApriltag_Position()
        self.__id_list: list = []
        self.__width: float = WALL_WIDTH  # m
        self.__height: float = WALL_HEIGHT  # m
        self.__thickness: float = WALL_THICKNESS  # m
        self.__big_tag_size: float = WALL_BIG_TAG_SIZE  # m
        self.__small_tag_size: float = WALL_SMALL_TAG_SIZE  # m
        self.__topleft: list = None
        self.__topright: list = None
        self.__topleft: list = None
        self.__bottomright: list = None
        self.__leftside: float = None  # y 轴的值
        self.__rightside: float = None  # y 轴的值
        self.__topside: float = None  # x 轴的值
        self.__bottomside: float = None  # x 轴的值
        self.__identifier = 0  # 标识符, 用于算法区分墙与路径, 墙中心的本体
        self.__sub_indentifier = 0  # 标识符, 用于算法区分墙与路径, 墙的延伸, 包括连接

    @property
    def main_0(self) -> JApril_Code:
        return self.__main_0

    @property
    def main_1(self) -> JApril_Code:
        return self.__main_1

    @property
    def sub_0(self) -> JApril_Code:
        return self.__sub_0

    @property
    def sub_1(self) -> JApril_Code:
        return self.__sub_1

    @property
    def orientation(self) -> str:
        return self.__orientation

    @property
    def middle(self) -> JApriltag_Position:
        return self.__middle

    @property
    def id_list(self) -> list:
        return self.__id_list

    @property
    def width(self) -> list:
        return self.__width

    @property
    def height(self) -> list:
        return self.__height

    @property
    def thickness(self) -> list:
        return self.__thickness

    @property
    def big_tag_size(self) -> list:
        return self.__big_tag_size

    @property
    def small_tag_size(self) -> list:
        return self.__small_tag_size

    @property
    def topleft(self) -> tuple:
        return self.__topleft

    @property
    def topright(self) -> tuple:
        return self.__topright

    @property
    def bottomleft(self) -> tuple:
        return self.__bottomleft

    @property
    def bottomright(self) -> tuple:
        return self.__bottomright

    @property
    def leftside(self) -> float:
        return self.__leftside

    @property
    def rightside(self) -> float:
        return self.__rightside

    @property
    def topside(self) -> float:
        return self.__topside

    @property
    def bottomside(self) -> float:
        return self.__bottomside

    @property
    def identifier(self) -> int | str:
        return self.__identifier

    @property
    def sub_identifier(self) -> int | str:
        return self.__sub_indentifier

    def set_main_0(self, code_dict) -> None:
        if code_dict:
            self.main_0.set_id(code_dict['id'])
            if 'x' in code_dict and 'y' in code_dict and 'z' in code_dict:
                self.main_0.set_pos(
                    [code_dict['x'], code_dict['y'], code_dict['z']])
            if 'qw' in code_dict and 'qx' in code_dict and 'qy' in code_dict and 'qz' in code_dict:
                self.main_0.set_ori(
                    [code_dict['qw'], code_dict['qx'], code_dict['qy'], code_dict['qz']])

    def set_main_1(self, code_dict) -> None:
        if code_dict:
            self.main_1.set_id(code_dict['id'])
            if 'x' in code_dict and 'y' in code_dict and 'z' in code_dict:
                self.main_1.set_pos(
                    [code_dict['x'], code_dict['y'], code_dict['z']])
            if 'qw' in code_dict and 'qx' in code_dict and 'qy' in code_dict and 'qz' in code_dict:
                self.main_1.set_ori(
                    [code_dict['qw'], code_dict['qx'], code_dict['qy'], code_dict['qz']])

    def set_sub_0(self, code_dict) -> None:
        if code_dict:
            self.sub_0.set_id(code_dict['id'])
            if 'x' in code_dict and 'y' in code_dict and 'z' in code_dict:
                self.sub_0.set_pos(
                    [code_dict['x'], code_dict['y'], code_dict['z']])
            if 'qw' in code_dict and 'qx' in code_dict and 'qy' in code_dict and 'qz' in code_dict:
                self.sub_0.set_ori(
                    [code_dict['qw'], code_dict['qx'], code_dict['qy'], code_dict['qz']])

    def set_sub_1(self, code_dict) -> None:
        if code_dict:
            self.sub_1.set_id(code_dict['id'])
            if 'x' in code_dict and 'y' in code_dict and 'z' in code_dict:
                self.sub_1.set_pos(
                    [code_dict['x'], code_dict['y'], code_dict['z']])
            if 'qw' in code_dict and 'qx' in code_dict and 'qy' in code_dict and 'qz' in code_dict:
                self.sub_1.set_ori(
                    [code_dict['qw'], code_dict['qx'], code_dict['qy'], code_dict['qz']])

    def set_middle(self):  # 板子本身的方向，而非板子的法向向量方向
        if self.main_0.ori.qx():
            if self.main_0.ori.qx() == 0.5:
                self.__orientation = 'H'
            else:
                self.__orientation = 'V'
        # 如果存在4个 Apriltag 码：
        if (self.main_0.pos.x() or self.main_0.pos.x() == 0) and (self.sub_0.pos.x() or self.sub_0.pos.x() == 0):
            # print(self.main_0.pos.y() , self.sub_0.pos.y())
            self.middle.set_x((self.main_0.pos.x() + self.sub_0.pos.x()) / 2)
            self.middle.set_y((self.main_0.pos.y() + self.sub_0.pos.y()) / 2)
            self.middle.set_z((self.main_0.pos.z() + self.sub_0.pos.z()) / 2)
        # 如果只有 主Apriltag码：
        elif self.main_0.pos.x() or self.main_0.pos.x() == 0:
            if self.__orientation == 'H':
                self.middle.set_x(self.main_0.pos.x()+0.0015)
                self.middle.set_y(self.main_0.pos.y())
            else:
                self.middle.set_x(self.main_0.pos.x())
                self.middle.set_y(self.main_0.pos.y()+0.0015)
            self.middle.set_z(self.main_0.pos.z())
        # 如果只有 副Apriltag码:
        elif self.sub_0.pos.x() or self.sub_0.pos.x() == 0:
            if self.__orientation == 'H':
                self.middle.set_x(self.sub_0.pos.x()+0.0015)
                self.middle.set_y(self.sub_0.pos.y())
            else:
                self.middle.set_x(self.sub_0.pos.x())
                self.middle.set_y(self.sub_0.pos.y()+0.0015)
            self.middle.set_z(self.sub_0.pos.z())
        else:
            print(f'当前墙壁无中心点, 未得到坐标数据{self.main_0.id}')

    def set_id_list(self, id0, id1, id2, id3):
        self.__id_list = [id0, id1, id2, id3]

    def set_edge(self):
        if not self.middle.x() or not self.__middle.y():
            return
        if self.orientation == 'H':
            self.__leftside = self.__middle.y() - self.__width / 2
            self.__rightside = self.__middle.y() + self.__width / 2
            self.__topside = self.middle.x() - self.__thickness / 2
            self.__bottomside = self.middle.x() + self.__thickness / 2
        else:
            self.__leftside = self.__middle.y() - self.__thickness / 2
            self.__rightside = self.__middle.y() + self.__thickness / 2
            self.__topside = self.middle.x() - self.__width / 2
            self.__bottomside = self.middle.x() + self.__width / 2

    def set_vertex(self):
        if not self.__leftside or not self.__topside or not self.__rightside or not self.__bottomside:
            return
        self.__topleft = (self.__leftside, self.__topside)
        self.__topright = (self.__rightside, self.__topside)
        self.__bottomleft = (self.__leftside, self.__bottomside)
        self.__bottomright = (self.__rightside, self.__bottomside)

    # 主要入口
    def set_wall(self, code_list_dict):
        min_x = code_list_dict[0]['x']
        min_y = code_list_dict[0]['y']
        max_x = 0
        max_y = 0
        main = None
        sub = None
        for i in code_list_dict:
            x = i['x']
            y = i['y']
            if x > max_x:
                max_x = x
            if x <= min_x:
                min_x = x
            if y > max_y:
                max_y = y
            if y <= min_y:
                min_y = y
        for i in code_list_dict:
            x = i['x']
            y = i['y']
            if x == min_x or y == min_y:
                if i['size'] == 0.084:
                    self.set_main_0(i)
                elif i['size'] == 0.0168:
                    self.set_main_1(i)
            elif x == max_x or y == max_y:
                if i['size'] == 0.084:
                    self.set_sub_0(i)
                elif i['size'] == 0.0168:
                    self.set_sub_1(i)
            else:
                print(f'方法 set_wall 输入错误, 请检查code_list_dict: {code_list_dict}\n 当前为: {i}\n min_x: {min_x}\tmin_y: {min_y}\tmax_x: {max_x}\tmax_y: {max_y}')
                return
        self.set_id_list(self.main_0.id, self.main_1.id, self.sub_0.id, self.sub_1.id)
        self.update_wall_calculated_data()

    # 有问题需要检查！！！！！！！！！不能使用id进行定义
    def set_wall_from_apriltag(self, apriltag: JApril_Tag_Info):
        id_list = apriltag.id
        for i in id_list:
            if i % 4 == 0:
                self.set_main_0({'id': i})
            elif i % 4 == 1:
                self.set_main_1({'id': i})
            elif i % 4 == 2:
                self.set_sub_0({'id': i})
            elif i % 4 == 3:
                self.set_sub_1({'id': i})
        self.set_id_list(self.main_0.id, self.main_1.id, self.sub_0.id, self.sub_1.id)
        self.update_wall_calculated_data()

    def update_wall_calculated_data(self):
        self.set_middle()
        self.set_edge()
        self.set_vertex()

        # 用于读取yaml文件, 并返回含有所有墙的列表


class JWalls():
    def __init__(self) -> None:
        self.JWall_list: list = []
        self.range_Maze_x: int = []
        self.range_Maze_y: int = []
        self.max_middle_x: float = None
        self.max_middle_y: float = None

    def build_JWall_list_from_yaml(self, yaml_data: yaml) -> list[JWall]:
        list_dict = yaml_data['tag_bundles'][0]['layout']
        code_list_dict = self._code_sort(list_dict)
        wall_list = self._build_in_JWall_from_code_list(code_list_dict)
        self.JWall_list = wall_list
        self.range_Maze_x = int((self.max_middle_x-(WALL_WIDTH_HALF + WALL_THICKNESS)) / (WALL_WIDTH + WALL_THICKNESS) + 1)  # 最大中点位置 = (0.25 + 0.003) * (n - 1) + (0.125 + 0.003)
        self.range_Maze_y = int((self.max_middle_y-(WALL_WIDTH_HALF + WALL_THICKNESS)) / (WALL_WIDTH + WALL_THICKNESS) + 1)
        return wall_list

    def _build_in_JWall_from_code_list(self, code_list_dict: list) -> list[JWall]:
        temp_JWalls_list = []
        for i in code_list_dict:  # i 是个列表, 里面是四个 code 的字典
            temp_wall = JWall()
            temp_wall.set_wall(i)
            self.set_update_max_middle_x_y(temp_wall)
            temp_JWalls_list.append(temp_wall)
        return temp_JWalls_list

    # 设置最大墙中点
    def set_update_max_middle_x_y(self, wall: JWall) -> None:
        if self.max_middle_x:
            if wall.middle.x() > self.max_middle_x:
                self.max_middle_x = wall.middle.x()
        else:
            self.max_middle_x = wall.middle.x()
        if self.max_middle_y:
            if wall.middle.y() > self.max_middle_y:
                self.max_middle_y = wall.middle.y()
        else:
            self.max_middle_y = wall.middle.y()

    # Apriltag分类，将一张板上的4个码放在一个列表中
    def _code_sort(self, ori_list: list[dict]) -> list[list[dict]]:
        temp_JWall_list = []
        for item_dict in ori_list[:]:
            code_list = []
            id_list = self._compare_with_base(item_dict['id'])
            new_list = []
            for item_dict_2 in ori_list:
                if item_dict_2['id'] in id_list:
                    code_list.append(item_dict_2)
                else:
                    new_list.append(item_dict_2)
            ori_list = new_list
            if code_list:
                temp_JWall_list.append(code_list)
        return temp_JWall_list

    # 对比标准板库
    def _compare_with_base(self, input_id: int) -> list:
        for _, value in APRILCODE_BASE.items():
            if input_id in value:
                return value
        return []

    # 删除列表元素
    def _delete_in_list(self, id_input: int, list: list) -> None:
        for index, item in enumerate(list):
            if item['id'] == id_input:
                del list[index]
                break
