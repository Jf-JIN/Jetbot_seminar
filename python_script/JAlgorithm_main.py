'''
这个文件主要是用于写路径算法的，包括计算路径及规划，建立地图，向中央处理单元发送指令建议
'''

class JAlgorithm():
    def __init__(self) -> None:
        pass

    def id_classification(self, id):
        if id % 4 == 0:
            print('W/N')
        elif id % 4 == 2:
            print('O/S')
        else:
            print('false id')