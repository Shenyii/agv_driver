该程序在ROS indigo成功运行。
ROS节点：drivernode
功能：ROS与单片机STM32F407串口通信，并且利用电机编码器和MPU6050实现里程计定位，并向单片机发送控制电机的速度命令。
订阅Topic：“cmd_vel”
发布Topic：里程计

ROS节点：path_tracker
功能：订阅规划器发布的路径，并通过PID跟踪这条路径。
订阅Topic：“/move_base/NavfnROS/plan”、“/destination_pose”
发布Topic：“/cmd_vel”、“whether_arrived_destination”

其它ROS节点为测试程序。