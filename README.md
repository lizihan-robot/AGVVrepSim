1.在vrep中File->open sence ->选择agv12.ttt

2.python planner.py


文件介绍：

agv12.ttt ： vrep的项目文件，需要先在vrep中加载

vrep_map.py： 地图从vrep中读取的plane mark矩阵，模拟现实中的二维码矩阵

agv.py ： 小车的对象，实例化的时候传入需要建立的小车数量

pid_control.py： 小车在运动的时候进行 位置式pid控制

planner.py 启动入口，进行简单仿真规划


遗留问题：

目前发现在使用多线程对多台小车进行控制的时候小车会出现乱跑,
当小车数量太多的时候也会出现乱跑

说是乱跑但看起来更像通讯问题 导致pid调整不及时
