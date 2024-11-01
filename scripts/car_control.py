import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from ros_ht_msg.msg import ht_control
import math
from icecream import ic

# 初始化全局变量
control_linear_x = 0
control_angle = 0
imu_angle = 0
err_last1 = 0
err_last2 = 0
output_last = 0


class car_control:

    def __init__(self):
        self.pub_control = rospy.Publisher("/HT_Control", ht_control, queue_size=100)
        self.sub_control = rospy.Subscriber("/cmd_vel", Twist, self.control_update,queue_size=10)
        self.sub_imu = rospy.Subscriber("/wit/imu",Imu,self.imu_update,queue_size=10)

    def control_update(self,Twist):
        # 更新控制命令
        global control_linear_x ,control_angle
        control_linear_x = Twist.linear.x
        control_angle = Twist.angular.z

    def imu_update(self,Imu):
        # 更新imu反馈位姿，并计算得到yaw（偏航）角
        global imu_angle
        orientation_x = Imu.orientation.x
        orientation_y = Imu.orientation.y
        orientation_z = Imu.orientation.z
        orientation_w = Imu.orientation.w
        imu_angle = math.atan2(2 * (orientation_w * orientation_z + orientation_x * orientation_y), 1 - 2 * (orientation_y ** 2 + orientation_z ** 2))

    def pid_controller(self,angle_set,angle_back,Kp,Ki,Kd,max_output):
        # 增量式PID算法
        global err_last1 ,err_last2 ,output_last
        err = angle_set - angle_back
        increoutput = Kp*(err - err_last1) + Ki*err + Kd*(err - 2*err_last1 + err_last2)
        output = output_last + increoutput

        err_last2 = err_last1
        err_last1 = err
        if output > max_output:                 #限制控制器输出赋值
            output = max_output
        elif output < (-max_output):
            output = (-max_output)
        output_last = output

        return output
    
    def pub_ht_control(self,linear_x,angle):
        # 控制器输出数据转换，线速度m转成mm，角度rad转成度
        control = ht_control()
        pub_linear_x = int(linear_x*1000)
        pub_angle = int(math.degrees(angle)*100)
        # 发布数据
        control.mode = 0
        control.x = pub_linear_x
        control.y = pub_angle
        control.z = 0
        ic(control)
        self.pub_control.publish(control)
    
if __name__ == '__main__':
    # 使用到的全局变量：control_linear_x  control_angle  imu_angle  output_last
    rospy.init_node("controller")

    car_control = car_control()
    # 初始化控制器参数
    Kp = rospy.get_param("pid_gain/Kp", 0.5)
    Ki = rospy.get_param("pid_gain/Ki", 0.05)
    Kd = rospy.get_param("pid_gain/Kd", 0.05)
    rate = rospy.get_param("controller_rate", 10)
    max_controller_output = rospy.get_param("max_controller_output", 0.7854)

    rate = rospy.Rate(rate)

    while not rospy.is_shutdown():
        # 运动控制
        while control_linear_x != 0:
            output_linear_x = control_linear_x
            # 使用PID算法
            if mod == "PID_true":
                set_angle = control_angle +imu_angle_init
                output_angle = car_control.pid_controller(set_angle,imu_angle,Kp,Ki,Kd,max_controller_output)
                if output_linear_x >=0:
                    car_control.pub_ht_control(output_linear_x,(-output_angle))           # 前进
                else:
                    car_control.pub_ht_control(output_linear_x,output_angle)              # 后退
            # 直接连接
            elif mod == "PID_false":
                output_angle = control_angle
                car_control.pub_ht_control(output_linear_x,(-output_angle))
            rate.sleep()
        # 静止时刷新记忆数据
        while control_linear_x == 0:
            output_last = 0
            imu_angle_init = imu_angle
            ic(output_last)
            ic(imu_angle_init)
            mod = rospy.get_param("mod", "PID_true")
            rate.sleep()

        
       




