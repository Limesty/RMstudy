# 概述
* 本文件中包含了一个命名空间和两个类：Map和Mapping，
* 命名空间用于定义特殊变量并防止命名冲突
* Map用来描述地图，Mapping负责描述运动状态和集成可能用到的函数。
***
# 类功能介绍
## Map
### protected部分
* 这一部分包含了地图中不同的地点集合，地图的边界等信息。
* 定义为protected便于继承类直接访问，且开发对可访问性要求不高
### public部分
* 这部分包含了判断函数，用于给定一个点坐标，判断
该点属于什么区域。

## Mapping
### 私有继承Map
* Mapping 类应当'has a' 地图信息，因此定义为Map的私有继承类，
可以访问Map的信息。
### private部分
* 用于描述机器人运动的变量，例如速度大小，运动方向，当前位置等等。
### public部分
* 定义机器人的运动策略的函数，描述机器人应当怎样运动。
* 属于路径规划的部分。
* 计划先用A*算法求出参考路径
* 再用frenet动态轨迹规划优化？ https://blog.csdn.net/m0_55205668/article/details/124884393
* 先看懂算法，具体实现cpp可能需要一段时间
### 友元函数部分
* 用于描述机器人碰到不同区域的反应。
* 由于可能需要调用其他类的私有信息，因此定义为友元函数。
***
#12.3更新
* 确定了路径规划的大概思路
* 采用局部路径规划算法
* 初始扫描全局栅格化，机器人体积做卷积核滤波，确定可行格点
* 在行进过程中确定下一步的速度与方向，作为函数输出
* 找到了teb_local_planner-melodic-devel包，包含了所需的函数和实例作为启发