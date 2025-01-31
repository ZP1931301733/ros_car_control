# Ros_car_control Packages Instructions

此软件包是 AgRobot 小组项目的一部分

此包结合IMU数据用于实现四轮四转式机器人运动过程中的位姿误差矫正 (具体实现方式为通过IMU测量的机器人姿态角和控制话题的姿态设定角作闭环控制)

此包依赖于：
- [IMU Package](https://github.com/hmxf/ros_imu_wit)
- [机器人底盘 Package](https://github.com/hmxf/ros_ht_msg)

主要文件说明：
- `car_controller.yaml` 控制器的参数配置文件。
- `car_control.py` 矫正功能实现的主要程序文件。

