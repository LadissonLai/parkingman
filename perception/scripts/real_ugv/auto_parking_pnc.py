#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
# 导入你自定义的车位消息
from parking_space_msgs.msg import ParkingSpace

def normalize_angle(angle):
    """
    将角度归一化到 [-pi, pi] 之间
    """
    return math.atan2(math.sin(angle), math.cos(angle))

class AutoParkingController:
    def __init__(self):
        rospy.init_node('auto_parking_node', anonymous=True)

        # ================= 1. 加载参数 =================
        # 车辆与车位参数
        self.ugv_length = rospy.get_param("~ugv_length", 0.65)  # 自车长度 (米)
        self.ugv_width = rospy.get_param("~ugv_width", 0.6)    # 自车宽度 (米)
        
        # 控制限幅
        self.max_v = rospy.get_param("~max_v", 0.2)            # 最大线速度 (m/s)
        self.max_w = rospy.get_param("~max_w", 0.3)            # 最大角速度 (rad/s)
        
        # 里程计/位姿标志位：默认 False，表示优先使用 TF 进行多传感器融合定位
        self.use_odom = rospy.get_param("~use_odom", True)    
        self.odom_data = None                                  # 用于存储回调收到的里程计数据
        
        # PID 参数
        self.kp_v = rospy.get_param("~kp_v", 0.5)              # 线速度 P
        self.kp_w = rospy.get_param("~kp_w", 1.0)              # 角速度 P
        
        # 状态机容忍度
        self.xy_tolerance = rospy.get_param("~xy_tolerance", 0.05)   # 距离误差阈值 (米)
        self.yaw_tolerance = rospy.get_param("~yaw_tolerance", 0.05) # 角度误差阈值 (弧度, 约2.8度)

        # ================= 2. 初始化变量 =================
        # TF 监听器：用于获取自车在 camera_init 下的位姿
        self.tf_listener = tf.TransformListener()
        
        # 状态机状态: 0:IDLE, 1:TURN_TO_PREP, 2:GO_TO_PREP, 3:ALIGN_YAW, 4:REVERSE_IN, 5:FINISHED
        self.state = 0 
        
        # 目标与准备点
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0
        
        self.prep_x = 0.0
        self.prep_y = 0.0
        
        self.has_goal = False

        # ================= 3. ROS 订阅与发布 =================
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.path_pub = rospy.Publisher('/parking_path_vis', Marker, queue_size=1, latch=True)
        self.goal_sub = rospy.Subscriber('/parking_goal', ParkingSpace, self.goal_callback)
        
        # 启动时清空先前残留的路径与可视化
        rospy.sleep(0.5)  # 稍微等待publisher注册
        self.clear_path_marker()
        
        # 里程计订阅 (如果标志位为 True)
        if self.use_odom:
            self.odom_sub = rospy.Subscriber('/Odometry', Odometry, self.odom_callback)

        rospy.loginfo("[AutoParking] 自动泊车节点初始化完成，等待 /parking_goal 指令...")

    def odom_callback(self, msg):
        """
        接收小车里程计的回调
        """
        self.odom_data = msg

    def goal_callback(self, msg):
        """
        接收到目标车位时的回调函数
        """
        if self.has_goal:
            return  # 如果已经在泊车流程中，不再接收新目标，实现"盲停"防止跳变

        if msg.confidence < 0.8:  # 置信度过滤
            rospy.logwarn("[AutoParking] 收到车位目标，但置信度过低忽略。")
            return

        # 1. 提取车位中心坐标和方向
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        
        # 将四元数转为欧拉角提取 yaw
        orientation_q = msg.pose.orientation
        _, _, self.goal_yaw = tf.transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )

        # 2. 计算准备点 (Preparation Point)
        # 箭头指向外部，准备点沿箭头方向延伸 D
        # D = 半个车位宽度(沿箭头方向) + 半个车长 + 0.1米
        D = (msg.width / 2.0) + (self.ugv_length / 2.0) + 0.1
        
        self.prep_x = self.goal_x + D * math.cos(self.goal_yaw)
        self.prep_y = self.goal_y + D * math.sin(self.goal_yaw)

        self.has_goal = True
        self.state = 1
        rospy.loginfo("[AutoParking] 收到new泊车指令！")
        rospy.loginfo("目标车位: X={:.2f}, Y={:.2f}, Yaw={:.2f}".format(self.goal_x, self.goal_y, self.goal_yaw))
        rospy.loginfo("准备点位: X={:.2f}, Y={:.2f}".format(self.prep_x, self.prep_y))
        rospy.loginfo(">>>>> 进入状态 1: 转向准备点 (TURN_TO_PREP)")

        # 发布可视化路径
        self.publish_path_marker()

    def publish_path_marker(self):
        curr_x, curr_y, _ = self.get_current_pose()
        if curr_x is None:
            rospy.logwarn("[AutoParking] 获取当前位姿失败，可视化路径起点默认置零。")
            curr_x, curr_y = 0.0, 0.0
            
        marker = Marker()
        marker.header.frame_id = "camera_init"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "parking_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        # ADD 动作在同 ns+id 的情况下会自动覆盖之前的 Marker，从而实现新目标删除旧路径
        marker.action = Marker.ADD 
        
        # 线宽
        marker.scale.x = 0.05
        
        # 颜色: 紫色 (R=0.5, G=0.0, B=0.5)
        marker.color.r = 0.5
        marker.color.g = 0.0
        marker.color.b = 0.5
        marker.color.a = 1.0
        
        # 起点
        p_start = Point()
        p_start.x = curr_x
        p_start.y = curr_y
        p_start.z = 0.0
        marker.points.append(p_start)
        
        # 准备点
        p_prep = Point()
        p_prep.x = self.prep_x
        p_prep.y = self.prep_y
        p_prep.z = 0.0
        marker.points.append(p_prep)
        
        # 终点
        p_goal = Point()
        p_goal.x = self.goal_x
        p_goal.y = self.goal_y
        p_goal.z = 0.0
        marker.points.append(p_goal)
        
        self.path_pub.publish(marker)

    def clear_path_marker(self):
        """
        清空 RViz 里面的轨迹显示
        """
        marker = Marker()
        marker.header.frame_id = "camera_init"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "parking_path"
        marker.id = 0
        marker.action = Marker.DELETE
        self.path_pub.publish(marker)

    def get_current_pose(self):
        """
        根据 use_odom 标志位选择通过 TF 获取还是通过里程计 Odometry 获取位姿
        """
        if self.use_odom:
            # 方式1：通过 Odometry 话题获取 
            if self.odom_data is not None:
                curr_x = self.odom_data.pose.pose.position.x
                curr_y = self.odom_data.pose.pose.position.y
                orientation_q = self.odom_data.pose.pose.orientation
                _, _, curr_yaw = tf.transformations.euler_from_quaternion([
                    orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
                return curr_x, curr_y, curr_yaw
            return None, None, None
        else:
            # 方式2：通过 TF 获取小车在全局系 camera_init (或 map/odom) 下的坐标和偏航角 (默认优先级更高)
            try:
                # 监听全局坐标系到自车坐标系的变换
                (trans, rot) = self.tf_listener.lookupTransform('camera_init', 'body', rospy.Time(0))
                curr_x = trans[0]
                curr_y = trans[1]
                _, _, curr_yaw = tf.transformations.euler_from_quaternion(rot)
                return curr_x, curr_y, curr_yaw
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                return None, None, None

    def run(self):
        """
        主控制循环：运行状态机和 PID
        """
        rate = rospy.Rate(20) # 20Hz 控制频率
        cmd = Twist()

        while not rospy.is_shutdown():
            if self.state == 0 or not self.has_goal:
                rate.sleep()
                continue

            # 获取当前位姿
            curr_x, curr_y, curr_yaw = self.get_current_pose()
            if curr_x is None:
                rospy.logwarn_throttle(2.0, "[AutoParking] 无法获取 camera_init 到 body 的 TF 变换！")
                rate.sleep()
                continue

            # 默认速度为0
            v = 0.0
            w = 0.0

            # ---------------------------------------------------------
            # STATE 1: 原地转向准备点 (TURN_TO_PREP)
            # ---------------------------------------------------------
            if self.state == 1:
                target_yaw = math.atan2(self.prep_y - curr_y, self.prep_x - curr_x)
                yaw_error = normalize_angle(target_yaw - curr_yaw)
                
                if abs(yaw_error) < self.yaw_tolerance:
                    self.state = 2
                    rospy.loginfo(">>>>> 进入状态 2: 直行前往准备点 (GO_TO_PREP)")
                else:
                    w = self.kp_w * yaw_error
                    # 对角速度限幅
                    w = max(-self.max_w, min(self.max_w, w))

            # ---------------------------------------------------------
            # STATE 2: 直行前往准备点 (GO_TO_PREP)
            # ---------------------------------------------------------
            elif self.state == 2:
                dist_error = math.hypot(self.prep_x - curr_x, self.prep_y - curr_y)
                
                # 动态计算目标角度（边走边微调航向，防止走偏）
                target_yaw = math.atan2(self.prep_y - curr_y, self.prep_x - curr_x)
                yaw_error = normalize_angle(target_yaw - curr_yaw)

                if dist_error < self.xy_tolerance:
                    self.state = 3
                    rospy.loginfo(">>>>> 进入状态 3: 原地对齐车位航向 (ALIGN_YAW)")
                else:
                    v = self.kp_v * dist_error
                    v = max(-self.max_v, min(self.max_v, v))
                    # 距离很近时关闭角速度微调，防止震荡
                    w = self.kp_w * yaw_error if dist_error > 0.2 else 0.0
                    w = max(-self.max_w, min(self.max_w, w))

            # ---------------------------------------------------------
            # STATE 3: 原地对齐车位航向 (ALIGN_YAW)
            # ---------------------------------------------------------
            elif self.state == 3:
                # 倒车入库且箭头朝外，意味着倒车时车头必须朝外，即对齐 goal_yaw
                target_yaw = self.goal_yaw
                yaw_error = normalize_angle(target_yaw - curr_yaw)

                if abs(yaw_error) < self.yaw_tolerance:
                    self.state = 4
                    rospy.loginfo(">>>>> 进入状态 4: 直线倒车入库 (REVERSE_IN)")
                else:
                    w = self.kp_w * yaw_error
                    w = max(-self.max_w, min(self.max_w, w))

            # ---------------------------------------------------------
            # STATE 4: 直线倒车入库 (REVERSE_IN)
            # ---------------------------------------------------------
            elif self.state == 4:
                dist_error = math.hypot(self.goal_x - curr_x, self.goal_y - curr_y)
                
                # 倒车过程中，保持车身姿态平行于车位箭头
                yaw_error = normalize_angle(self.goal_yaw - curr_yaw)

                if dist_error < self.xy_tolerance:
                    self.state = 5
                    rospy.loginfo(">>>>> 进入状态 5: 泊车完成 (FINISHED)！")
                else:
                    # 倒车！线速度取负值
                    v = -1.0 * (self.kp_v * dist_error)
                    v = max(-self.max_v, min(self.max_v, v))  # 负的最大值限制其实在 max 函数处理了，注意边界
                    
                    # 为了安全，这里严格限制倒车速度区间
                    v = max(-self.max_v, min(-0.05, v)) # 设定个最低倒车速度 0.05，防止靠不近

                    # 微调航向：倒车时直接依据姿态误差给 w 补偿即可
                    w = self.kp_w * yaw_error
                    w = max(-self.max_w, min(self.max_w, w))

            # ---------------------------------------------------------
            # STATE 5: 泊车完成 (FINISHED)
            # ---------------------------------------------------------
            elif self.state == 5:
                v = 0.0
                w = 0.0
                # 这里可以加个重置机制或者发布泊车完成的话题
                self.has_goal = False 
                
                # 任务完成后清空可视化路径
                self.clear_path_marker()

            # 发布控制指令
            cmd.linear.x = v
            cmd.angular.z = w
            self.cmd_pub.publish(cmd)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = AutoParkingController()
        controller.run()
    except rospy.ROSInterruptException:
        pass