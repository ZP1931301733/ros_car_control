import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from ros_ht_msg.msg import ht_control
import math
from icecream import ic


class car_control:

    def __init__(self):
        self.pub_control = rospy.Publisher("/HT_Control", ht_control, queue_size=100)
        self.sub_control = rospy.Subscriber("/cmd_vel", Twist, self.control_update,queue_size=10)
        self.sub_imu = rospy.Subscriber("/wit/imu",Imu,self.imu_update,queue_size=10)

        # 初始化变量
        self.control_linear_x = 0
        self.control_angle = 0
        self.imu_angle = 0
        self.imu_angle_init = 0

        self.err_last1 = 0
        self.err_last2 = 0
        self.output_last = 0


    def control_update(self,Twist):
        # 更新控制命令
        self.control_linear_x = Twist.linear.x
        self.control_angle = Twist.angular.z

    def imu_update(self,Imu):
        # 更新imu反馈位姿，并计算得到yaw（偏航）角
        orientation_x = Imu.orientation.x
        orientation_y = Imu.orientation.y
        orientation_z = Imu.orientation.z
        orientation_w = Imu.orientation.w
        self.imu_angle = math.atan2(2 * (orientation_w * orientation_z + orientation_x * orientation_y), 1 - 2 * (orientation_y ** 2 + orientation_z ** 2))

    def pid_controller(self,angle_set,angle_back,Kp,Ki,Kd,max_output):
        # 增量式PID算法
        err = angle_set - angle_back
        increoutput = Kp*(err - self.err_last1) + Ki*err + Kd*(err - 2*self.err_last1 + self.err_last2)
        output = self.output_last + increoutput

        self.err_last2 = self.err_last1
        self.err_last1 = err
        # 限制控制器输出幅值
        output = max(min(output, max_output), -max_output)
        self.output_last = output

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
    
    def move_ht_control(self):
        # 运动控制

        # 初始化控制器参数
        str_gKp = rospy.get_param("straight_pid_gain/gKp", 5)
        str_gKi = rospy.get_param("straight_pid_gain/gKi", 1)
        str_gKd = rospy.get_param("straight_pid_gain/gKd", 1)

        tur_Kp = rospy.get_param("turn_pid_gain/Kp", 2)
        tur_Ki = rospy.get_param("turn_pid_gain/Ki", 0.2)
        tur_Kd = rospy.get_param("turn_pid_gain/Kd", 0.2)

        mod = rospy.get_param("mod", "PID_true")
        rate = rospy.get_param("controller_rate", 10)
        max_controller_output = rospy.get_param("max_controller_output", 0.7854)
        tolerance = rospy.get_param("turn_goal_tolerance", 0.01)

        rate = rospy.Rate(rate)


        # 直行控制
        while self.control_linear_x != 0 and self.control_angle == 0:
            output_linear_x = self.control_linear_x
            # 根据线速度计算自适应PID参数
            str_Kp = str_gKp*abs(self.control_linear_x)
            str_Ki = str_gKi*abs(self.control_linear_x)
            str_Kd = str_gKd*abs(self.control_linear_x)
            ic(str_Kp)
            ic(str_Ki)
            ic(str_Kd)
            # 使用PID算法
            if mod == "PID_true":
                set_angle = self.control_angle + self.imu_angle_init
                output_angle = self.pid_controller(set_angle,self.imu_angle,str_Kp,str_Ki,str_Kd,max_controller_output)
                if output_linear_x >= 0:
                    self.pub_ht_control(output_linear_x,(-output_angle))           # 前进
                else:
                    self.pub_ht_control(output_linear_x,output_angle)              # 后退
            # 直接连接
            elif mod == "PID_false":
                output_angle = self.control_angle
                self.pub_ht_control(output_linear_x,(-output_angle))
            rate.sleep()
        
        # 转弯控制
        while self.control_linear_x != 0 and self.control_angle != 0:
            output_linear_x = self.control_linear_x
        
            ic(tur_Kp)
            ic(tur_Ki)
            ic(tur_Kd)
            # 使用PID算法
            if mod == "PID_true":
                set_angle = self.control_angle + self.imu_angle_init
                output_angle = self.pid_controller(set_angle,self.imu_angle,tur_Kp,tur_Ki,tur_Kd,max_controller_output)
                if abs(set_angle - self.imu_angle) <= tolerance:                               # 到达目标停止并刷新记忆
                    output_linear_x = 0
                    output_angle = 0
                    self.control_linear_x = 0
                    self.output_last = 0
                    self.imu_angle_init = self.imu_angle
                if output_linear_x >= 0:
                    self.pub_ht_control(output_linear_x,(-output_angle))           # 前进转弯
                else:
                    self.pub_ht_control(output_linear_x,output_angle)              # 后退转弯
            # 直接连接
            elif mod == "PID_false":
                output_angle = self.control_angle
                self.pub_ht_control(output_linear_x,(-output_angle))
            rate.sleep()

        # 静止时刷新记忆数据
        while self.control_linear_x == 0:
            self.output_last = 0
            self.imu_angle_init = self.imu_angle

            ic(self.output_last)
            ic(self.imu_angle_init)

            mod = rospy.get_param("mod", "PID_true")
            rate.sleep()
        
           
if __name__ == '__main__':
    rospy.init_node("controller")

    car_control = car_control()
    
    while not rospy.is_shutdown():
       
        car_control.move_ht_control()
