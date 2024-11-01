# 所有的单位都是米， 角度为度

JB_DISTANCE_FROM_CAMERA_TO_CENTER = 0.0596      # 相机到旋转中心的距离
JB_DISTANCE_FROM_CAMERA_TO_FRONT = 0.0168       # 相机到小车前侧的距离
JB_DISTANCE_BETWEEN_DOUBLE_WHEEL_OUTSIDE = 0.132    # 两个轮子外侧距离
JB_WHEEL_WIDTH = 0.015                          # 轮子宽度
JB_WHEEL_DIAMETER = 0.0645                      # 轮子直径
JB_WHEEL_RADIUS = JB_WHEEL_DIAMETER / 2     # 轮子半径
JB_WHEEL_CENTER_DISTANCE_TO_FRONT_SIDE = 0.045  # 旋转中心到前侧距离
JB_WHEEL_CENTER_TO_CENTER = (JB_DISTANCE_BETWEEN_DOUBLE_WHEEL_OUTSIDE - JB_WHEEL_WIDTH) / 2  # 单边轮子中线到中心的距离
JB_BODY_WIDTH = 0.0954                          # 小车宽度
JB_BODY_LENGTH = 0.134                          # 小车长度
MOTOR_MAX_SPEED_RATE = 0.5     # 电机最大转动速率
MOTOR_MIN_START = 0.15
MOTOR_MAX_STOP = 0.1


PATH_TURN_DISTANCE = 0.125                      # 转弯提前停止距离
PATH_TURN_SPEED = 0.18
PATH_DISTANCE_BEFORE_TURN = 0
PATH_SEARCH_APRILTAG_SPEED = 0.2
PATH_SEARCH_APRILTAG_TURN_TIME = 0.2
PATH_SEARCH_APRILTAG_DEGREES = 10
PATH_DEFAULT_MIN_DISTANCE = 0.02               # 距离墙壁最小距离
PATH_TOLERANCE_DISTANCE = 0.01                 # pid 允许距离误差
PATH_TOLERANCE_DEGREES = 2.86                     # pid 允许角度误差
PATH_CALIBRATION_TOLERANCE_DISTANCE_NEAR = 0.005    # 矫正允许距离误差-近
PATH_CALIBRATION_TOLERANCE_DISTANCE_FAR = 0.010     # 矫正允许距离误差-远
PATH_CALIBRATION_TOLERANCE_DEGREES = 1                # 矫正允许角度误差

WALL_WIDTH = 0.250                                  # 墙壁宽度
WALL_WIDTH_HALF = WALL_WIDTH / 2                # 墙壁半宽
WALL_HEIGHT = 0.170                                 # 墙壁高度
WALL_THICKNESS = 0.003                              # 墙壁厚度
WALL_THICKNESS_HALF = WALL_THICKNESS / 2     # 墙壁半厚
WALL_BIG_TAG_SIZE = 0.140                           # 大码大小
WALL_SMALL_TAG_SIZE = 0.028                         # 小码大小

# 距离pid参数
PID_DISTANCE_DEFAULT_KP = 1
PID_DISTANCE_DEFAULT_KI = 0.1
PID_DISTANCE_DEFAULT_KD = 0
PID_INTEGRAL_MAX = 2

MOTOR_LEFT_CALIB = 0
MOTOR_RIGHT_CALIB = 0

CUBE_WIDTH = 0.028
CUBE_TO_CENTER = CUBE_WIDTH / 2

DIGITS = 6
DIGITS_S = 2

DRIVER_VOLTAGE = 12.0
MAX_VOLTAGE = 6.0
SCALE_RATIO = MAX_VOLTAGE / DRIVER_VOLTAGE

# 墙壁Apriltag码库
APRILCODE_BASE = {
    # 最小值可用 West/Nord
    0: [0, 1, 22, 23],
    4: [4, 5, 18, 19],
    8: [8, 9, 14, 15],
    24: [24, 25, 46, 47],
    28: [28, 29, 42, 43],
    32: [32, 33, 38, 39],
    96: [96, 97, 98, 99],
    100: [100, 101, 102, 103],
    104: [104, 105, 106, 107],
    108: [108, 109, 110, 111],
    112: [112, 113, 114, 115],
    116: [116, 117, 118, 119],
    120: [120, 121, 122, 123],
    124: [124, 125, 126, 127],
    128: [128, 129, 130, 131],
    132: [132, 133, 134, 135],
    136: [136, 137, 138, 139],
    140: [140, 141, 142, 143],
    144: [144, 145, 146, 147],
    148: [148, 149, 150, 151],
    152: [152, 153, 154, 155],
    156: [156, 157, 158, 159],
    160: [160, 161, 162, 163],
    164: [164, 165, 166, 167],
    168: [168, 169, 170, 171],
    172: [172, 173, 174, 175],
    176: [176, 177, 178, 179],
    180: [180, 181, 182, 183],
    184: [184, 185, 186, 187],
    188: [188, 189, 190, 191],
    # 不推荐使用，外墙（单面墙）
    48: [48, 49, None, None],
    56: [56, 57, None, None],
    64: [64, 65, None, None],
    68: [68, 69, None, None],
    72: [72, 73, None, None],
    76: [76, 77, None, None],
    80: [80, 81, None, None],
    84: [84, 85, None, None],
    88: [88, 89, None, None],
    92: [92, 93, None, None],
    192: [192, 193, None, None],
    196: [196, 197, None, None],
    200: [200, 201, None, None],
    204: [204, 205, None, None],
    208: [208, 209, None, None],
    212: [212, 213, None, None],
    216: [216, 217, None, None],
    220: [220, 221, None, None],
    224: [224, 225, None, None],
    228: [228, 229, None, None],
    232: [232, 233, None, None],
    236: [236, 237, None, None],
    240: [240, 241, None, None],
    244: [244, 245, None, None],
    248: [248, 249, None, None],
    252: [252, 253, None, None],
    256: [256, 257, None, None],
    260: [260, 261, None, None],
    264: [264, 265, None, None],
    268: [268, 269, None, None],
    272: [272, 273, None, None],
    276: [276, 277, None, None],
    280: [280, 281, None, None],
    284: [284, 285, None, None],
    288: [288, 289, None, None],
    292: [292, 293, None, None],
    296: [296, 297, None, None],
    300: [300, 301, None, None],
    304: [304, 305, None, None],
    308: [308, 309, None, None],
    312: [312, 313, None, None],
    316: [316, 317, None, None],
    320: [320, 321, None, None],
    324: [324, 325, None, None],
    328: [328, 329, None, None],
    332: [332, 333, None, None],
    # 最小值为背板， Ost/Sued
    2: [2, 3, 20, 21],
    6: [6, 7, 16, 17],
    10: [10, 11, 12, 13],
    26: [26, 27, 44, 45],
    30: [30, 31, 40, 41],
    34: [34, 35, 36, 37],
    52: [52, 53, None, None],
    60: [60, 61, None, None]
}
