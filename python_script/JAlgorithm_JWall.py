from JAlgorithm_JWall_Const import *
import yaml


class JApriltag_Position():
    def __init__(self) -> None:
        self.__x__ = None
        self.__y__ = None
        self.__z__ = None

    @property
    def x(self) -> float:
        return self.__x__

    @property
    def y(self) -> float:
        return self.__y__

    @property
    def z(self) -> float:
        return self.__z__

    def set_x(self, x: float) -> None:
        self.__x__ = x

    def set_y(self, y: float) -> None:
        self.__y__ = y

    def set_z(self, z: float) -> None:
        self.__z__ = z


class JApriltag_Orientation():
    def __init__(self) -> None:
        self.__qw__ = None
        self.__qx__ = None
        self.__qy__ = None
        self.__qz__ = None

    @property
    def qw(self) -> float:
        return self.__qw__

    @property
    def qx(self) -> float:
        return self.__qx__

    @property
    def qy(self) -> float:
        return self.__qy__

    @property
    def qz(self) -> float:
        return self.__qz__

    def set_qw(self, qw: float) -> None:
        self.__qw__ = qw

    def set_qx(self, qx: float) -> None:
        self.__qx__ = qx

    def set_qy(self, qy: float) -> None:
        self.__qy__ = qy

    def set_qz(self, qz: float) -> None:
        self.__qz__ = qz


class JApril_Code():
    def __init__(self) -> None:
        self.__id__ = None
        self.__pos__ = JApriltag_Position()
        self.__ori__ = JApriltag_Orientation()

    @property
    def id(self) -> int:
        return self.__id__

    @property
    def pos(self) -> JApriltag_Position:
        return self.__pos__

    @property
    def ori(self) -> JApriltag_Orientation:
        return self.__ori__

    def set_id(self, id: int) -> None:
        self.__id__ = id

    def set_pos(self, xyz_list: list[float]) -> None:
        self.pos.set_x(xyz_list[0])
        self.pos.set_y(xyz_list[1])
        self.pos.set_z(xyz_list[2])

    def set_ori(self, wxyz_list: list[float]) -> None:
        self.__ori__.set_qw(wxyz_list[0])
        self.__ori__.set_qx(wxyz_list[1])
        self.__ori__.set_qy(wxyz_list[2])
        self.__ori__.set_qz(wxyz_list[3])


class JWall():
    def __init__(self):
        self.__main_0__ = JApril_Code()
        self.__main_1__ = JApril_Code()
        self.__sub_0__ = JApril_Code()
        self.__sub_1__ = JApril_Code()
        self.__orientation__: str = ''
        self.__middle__ = JApriltag_Position()
        self.__id_list__: list = []

    @property
    def main_0(self) -> JApril_Code:
        return self.__main_0__

    @property
    def main_1(self) -> JApril_Code:
        return self.__main_1__

    @property
    def sub_0(self) -> JApril_Code:
        return self.__sub_0__

    @property
    def sub_1(self) -> JApril_Code:
        return self.__sub_1__

    @property
    def middle(self) -> JApriltag_Position:
        return self.__middle__

    @property
    def id_list(self) -> list:
        return self.__id_list__

    @property
    def orientation(self) -> str:
        return self.__orientation__

    def set_main_0(self, code_dict) -> None:
        if code_dict:
            self.main_0.set_id(code_dict['id'])
            self.main_0.set_pos(
                [code_dict['x'], code_dict['y'], code_dict['z']])
            self.main_0.set_ori(
                [code_dict['qw'], code_dict['qx'], code_dict['qy'], code_dict['qz']])

    def set_main_1(self, code_dict) -> None:
        if code_dict:
            self.main_1.set_id(code_dict['id'])
            self.main_1.set_pos(
                [code_dict['x'], code_dict['y'], code_dict['z']])
            self.main_1.set_ori(
                [code_dict['qw'], code_dict['qx'], code_dict['qy'], code_dict['qz']])

    def set_sub_0(self, code_dict) -> None:
        if code_dict:
            self.sub_0.set_id(code_dict['id'])
            self.sub_0.set_pos(
                [code_dict['x'], code_dict['y'], code_dict['z']])
            self.sub_0.set_ori(
                [code_dict['qw'], code_dict['qx'], code_dict['qy'], code_dict['qz']])

    def set_sub_1(self, code_dict) -> None:
        if code_dict:
            self.sub_1.set_id(code_dict['id'])
            self.sub_1.set_pos(
                [code_dict['x'], code_dict['y'], code_dict['z']])
            self.sub_1.set_ori(
                [code_dict['qw'], code_dict['qx'], code_dict['qy'], code_dict['qz']])

    def set_middle(self):
        if self.main_0.ori.qx == 0.5:
            self.__orientation__ = 'H'
        else:
            self.__orientation__ = 'V'
        if self.main_0.pos.x and self.sub_0.pos.x:
            self.middle.set_x((self.main_0.pos.x + self.sub_0.pos.x) / 2)
            self.middle.set_y((self.main_0.pos.y + self.sub_0.pos.y) / 2)
            self.middle.set_z((self.main_0.pos.z + self.sub_0.pos.z) / 2)
        elif self.main_0.pos.x:
            self.middle.set_x(self.main_0.pos.x)
            self.middle.set_y(self.main_0.pos.y)
            self.middle.set_z(self.main_0.pos.z)
        elif self.sub_0.pos.x:
            self.middle.set_x(self.sub_0.pos.x)
            self.middle.set_y(self.sub_0.pos.y)
            self.middle.set_z(self.sub_0.pos.z)
        else:
            raise ValueError('没有中心点，两侧都没有码')

    def set_id_list(self, id0, id1, id2, id3):
        self.__id_list__ = [id0, id1, id2, id3]

    def set_wall(self, code_list_dict):
        for i in code_list_dict:
            if i['id'] % 4 == 0:
                self.set_main_0(i)
            elif i['id'] % 4 == 1:
                self.set_main_1(i)
            elif i['id'] % 4 == 2:
                self.set_sub_0(i)
            elif i['id'] % 4 == 3:
                self.set_sub_1(i)
            else:
                print(i)
                raise ValueError('方法 set_wall 输入错误，请检查code_list_dict')
        self.set_id_list(self.main_0.id, self.main_1.id,
                         self.sub_0.id, self.sub_1.id)
        self.set_middle()


class JWalls():
    def __init__(self) -> None:
        pass

    def build_JWall_list_from_yaml(self, yaml_data: yaml) -> list[JWall]:
        list_dict = yaml_data['tag_bundles'][0]['layout']
        code_list_dict = self._code_sort(list_dict)
        wall_list = self._build_in_JWall_from_code_list(code_list_dict)
        return wall_list

    def _build_in_JWall_from_code_list(self, code_list_dict: list) -> list[JWall]:
        temp_JWalls_list = []
        for i in code_list_dict:  # i 是个列表，里面是四个 code 的字典
            temp_wall = JWall()
            temp_wall.set_wall(i)
            temp_JWalls_list.append(temp_wall)
        return temp_JWalls_list

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

    def _compare_with_base(self, input_id: int) -> list:
        for _, value in APRILCODE_BASE.items():
            if input_id in value:
                return value
        return []

    def _delete_in_list(self, id_input: int, list: list) -> None:
        for index, item in enumerate(list):
            if item['id'] == id_input:
                del list[index]
                break
