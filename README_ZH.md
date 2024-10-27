# Jetbot_seminar

- [简述](https://github.com/Jf-JIN/Jetbot_seminar/blob/main/README_ZH.md#%E7%AE%80%E8%BF%B0)
- [任务说明](https://github.com/Jf-JIN/Jetbot_seminar/blob/main/README_ZH.md#%E4%BB%BB%E5%8A%A1%E8%AF%B4%E6%98%8E)
- [项目内容](https://github.com/Jf-JIN/Jetbot_seminar/blob/main/README_ZH.md#%E9%A1%B9%E7%9B%AE%E5%86%85%E5%AE%B9)
- [项目中仍存在的问题](https://github.com/Jf-JIN/Jetbot_seminar/blob/main/README_ZH.md#%E9%A1%B9%E7%9B%AE%E4%B8%AD%E4%BB%8D%E5%AD%98%E5%9C%A8%E7%9A%84%E9%97%AE%E9%A2%98)
## 简述
本项目主要是设计算法、控制，使小车jetbot可以按照根据要求，自行路径规划、移动、探索并构建地图。
##### 项目原说明文档：
-  [Project Seminar 'Robotics and Computational Intelligence' 2024 presented by RIS | Technical University of Darmstadt](https://github.com/NikHoh/jetbot_maze)
-  [Craft a maze from lasercut MDF walls and generate the corresponding AprilTag Bundle YAML](https://github.com/NikHoh/apriltag-maze)

##### 项目参考：
- [jetbot](https://github.com/NVIDIA-AI-IOT/jetbot)
- [jetbot_ros](https://github.com/dusty-nv/jetbot_ros)
- [apriltag](https://github.com/AprilRobotics/apriltag)
- [apriltag-imgs](https://github.com/AprilRobotics/apriltag-imgs)
- [Adafruit_CircuitPython_MotorKit](https://github.com/adafruit/Adafruit_CircuitPython_MotorKit)

## 任务说明
本Seminar主要是有三个任务：

* 在确定的迷宫中，指定 jetbot 的起点与终点，jetbot 自动规划路径并可以自行到指定位置
* 在不确定的迷宫中，jetbot 自行探索迷宫并构建迷宫的yaml文件
* 在不确定的迷宫中，jetbot 自行探索，直至找到指定物体(Apriltag编号大于900)，并返回起点
<div style="display:inline-block;">  <img src="https://github.com/Jf-JIN/Jetbot_seminar/blob/main/image/jetbot2.gif" alt="gif1" height = "200">  <img src="https://github.com/Jf-JIN/Jetbot_seminar/blob/main/image/jetbot3.gif" alt="gif2" height = "200"></div>

## 项目内容
本项目主要分为：客户端设计(前端UI + 部分后端算法)，服务器设计(基于socket通讯)， 小车系统(如下细分)
小车系统主要分为四个模块：中央处理，数据采集，电机驱动控制，路径算法。

#### 客户端设计
  客户端主要是识别yaml文件，并构建可计算的地图类，与小车的服务器进行通讯，实时显示小车的相机、电机、路径等数据，并可以将设置的pid参数实时传递给小车(用于调试)，改变小车的运动状态
  <div style="display:inline-block;">  <img src="https://github.com/Jf-JIN/Jetbot_seminar/blob/main/image/jetbot1.gif" alt="jetbot1" height = "200"> </div>
  其中构建地图类是将yaml文件进行分析，通过每面墙的中心点和朝向建立不同的墙类，然后将其放置于一个地图(管理器)类中，其内分为两种矩阵，一种是对象矩阵，含有实际墙类的矩阵，用于提取墙的信息，另一种是抽象矩阵，含有0和1的矩阵，用于A* 算法计算路径
  <div style="display:inline-block;">  <img src="https://github.com/Jf-JIN/Jetbot_seminar/blob/main/image/matrix2.png" alt="matrix2" height = "200">  <img src="https://github.com/Jf-JIN/Jetbot_seminar/blob/main/image/matrix3.png" alt="matrix3" height = "200"></div>

#### 电机驱动
主要是电机控制，PID或者其他控制算法，过程姿态矫正。其将先接收到数据采集发来的数据包进行定位，然后根据中央处理给出的具体算法控制小车准确移动到指定为止

#### 数据采集
主要是相机识别二维码，识别两侧前方的Apritag，处理IMU数据，并将数据进行封装打包，数据使用自定义的数据类 JLocation。其数据将不断发送给中央处理模块和电机驱动模块。

#### 中央处理
主要是将路径算法的结果传递给电机驱动执行，并将数据采集的数据实时通过socket传递给客户端用于显示，与此同时接收客户端发来的指令包

#### 路径算法
	主要是计算路径及规划，建立地图，向中央处理单元发送指令建议
	路径算法主要采用两种：A* 与 DFS*(深度优先算法)* ，
<div style="display:inline-block;">  <img src="https://github.com/Jf-JIN/Jetbot_seminar/blob/main/image/A+Algo.png" alt="A*Algo" height = "200"> </div>
* A*算法 
用于任务1和任务3，其中对于路径扩展，我们考虑了两种，即4方向扩展与8方向扩展
<div style="display:inline-block;">  <img src="https://github.com/Jf-JIN/Jetbot_seminar/blob/main/image/erweitung4.png" alt="erweitung4" height = "200">  <img src="https://github.com/Jf-JIN/Jetbot_seminar/blob/main/image/erweitung8.png" alt="erweitung8" height = "200"></div>
对于确定的迷宫，其算法结果如下（本项目中采用曼哈顿距离计算）：
<div style="display:inline-block;">  <img src="https://github.com/Jf-JIN/Jetbot_seminar/blob/main/image//4-AStar.png" alt="4-AStar" height = "200">  <img src="https://github.com/Jf-JIN/Jetbot_seminar/blob/main/image/8-AStar.png" alt="8-AStar" height = "200"></div>
通过初步计算，我们获得了一个含有诸多路径点的路线规划，此时我们需要将路径点(节点)相互连接成含有起点、终点、终点动作的路径动作，如下图
<div style="display:inline-block;">  <img src="https://github.com/Jf-JIN/Jetbot_seminar/blob/main/image/nodes.png" alt="nodes" height = "200">  <img src="https://github.com/Jf-JIN/Jetbot_seminar/blob/main/image/behandelt.png" alt="behandelt" height = "200"></div>
之后小车便可以根据路径动作进行移动。

<div style="display:inline-block;">  <img src="https://github.com/Jf-JIN/Jetbot_seminar/blob/main/image/DFS-Algo.png" alt="DFS-Algo" height = "200"> </div>
* DFS 算法 
主要用于第2个和第3个任务，在不知地图的情况下对各个岔路(节点)进行有序的访问。
在这个任务中，有两个问题需要注意：
自由墙问题：会造成小车持续访问相同的节点，所以确定节点是否访问过很重要，否则小车将进入无限循环
无墙问题：由于小车仅能稳定准确识别前方一格的Apriltag，所以当前方没有Apriltag时，则认为是岔路，会新增节点进行扩展。于是便出现了与自有墙同样的重复节点问题
<div style="display:inline-block;">  <img src="https://github.com/Jf-JIN/Jetbot_seminar/blob/main/image/freiWand.png" alt="freiWand" height = "200">  <img src="https://github.com/Jf-JIN/Jetbot_seminar/blob/main/image/keinWand.png" alt="keinWand" height = "200"></div>

* BFS算法由于时间原因，未能写完与DFS进行对比

## 项目中仍存在的问题
1. 对前方 Apriltag 码 的识别角度的限定，在任务1中会出现非垂直定位，而是斜向定位，会导致撞墙丢码
2. 任务3中，当前采用的是倒车回溯上一节点，可优化为A* 算法计算当前位置与上一节点的路径，从而以最短距离到达上一节点

