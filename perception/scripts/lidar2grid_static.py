#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf2_ros
import sensor_msgs.point_cloud2 as pc2
import pcl
# [优化] 移除 tf2_sensor_msgs，因为我们将直接在Numpy中进行坐标变换，效率更高
# import tf2_sensor_msgs 
import math
import tf.transformations as tft
import time

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, TransformStamped

class MapBuilder:
    def __init__(self):
        rospy.init_node('keyframed_static_mapper', anonymous=True)
        rospy.loginfo("启动'关键帧-仅障碍物'激光雷达栅格地图构建节点 (已优化)...")

        self.load_params()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.map_width_cells = int(self.map_width_m / self.map_resolution)
        self.map_height_cells = int(self.map_height_m / self.map_resolution)
        # self.log_odds_map = np.full((self.map_height_cells, self.map_width_cells), self.log_odds_prior_free, dtype=np.float32)
        self.log_odds_map = np.zeros((self.map_height_cells, self.map_width_cells), dtype=np.float32)

        self.last_keyframe_pose = None
        self.last_keyframe_time = rospy.Time(0)

        self.map_points_pub = rospy.Publisher('/lidar/points_for_map', PointCloud2, queue_size=1)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)

        self.lidar_sub = rospy.Subscriber('/carla/ego_vehicle/lidar', PointCloud2, self.lidar_callback, queue_size=1)
            
        rospy.loginfo("地图构建器初始化完成，等待激光雷达数据...")

    def load_params(self):
        """从ROS参数服务器加载所有必要的参数 (参数值未修改)"""
        rospy.loginfo("加载参数...")
        # 坐标系
        self.world_frame = rospy.get_param('~world_frame', 'parking_start_map')
        self.robot_base_frame = rospy.get_param('~robot_base_frame', 'ego_vehicle')

        # 点云处理参数
        self.lidar_max_range = rospy.get_param('~lidar_max_range', 50.0)
        self.lidar_max_range_threshold_factor = rospy.get_param('~lidar_max_range_threshold_factor', 0.99)
        self.ransac_dist_thresh = rospy.get_param('~ransac_distance_threshold', 0.15)
        self.slice_min_height_world = rospy.get_param('~obstacle_min_height_world', 0.3)
        self.slice_max_height_world = rospy.get_param('~obstacle_max_height_world', 1.0)
        
        # 地图参数
        self.map_resolution = rospy.get_param('~resolution', 0.1)
        self.map_width_m = rospy.get_param('~width', 100.0)
        self.map_height_m = rospy.get_param('~height', 100.0)
        self.map_origin_x = rospy.get_param('~origin_x', 0.0)
        self.map_origin_y = rospy.get_param('~origin_y', 0.0)

        # Log-odds更新参数
        self.log_odds_occupied = rospy.get_param('~log_odds_occupied', 0.9)
        self.log_odds_clamp_min = rospy.get_param('~log_odds_clamp_min', -5.0)
        self.log_odds_clamp_max = rospy.get_param('~log_odds_clamp_max', 5.0)
        self.log_odds_prior_free = rospy.get_param('~log_odds_prior_free', -0.5)

        # 关键帧参数
        self.keyframe_dist_thresh = rospy.get_param('~keyframe_dist_thresh', 0.5)
        self.keyframe_angle_thresh = rospy.get_param('~keyframe_angle_thresh', 10.0)
        self.keyframe_time_thresh = rospy.get_param('~keyframe_time_thresh', 2.0)

        rospy.loginfo("参数加载完毕。")

    def lidar_callback(self, msg: PointCloud2):
        """高频回调，仅用于关键帧决策。"""
        try:
            current_pose_stamped = self.get_current_pose(msg.header.stamp)
            if current_pose_stamped is None: 
                return

            if self.is_keyframe(current_pose_stamped):
                rospy.loginfo("\033[94m-- 新关键帧! 开始处理地图更新... --\033[0m")
                
                start_time = time.time()
                self.process_and_update_map(msg) # 不再需要传递current_pose
                end_time = time.time()
                rospy.loginfo(f"\033[94m关键帧处理耗时: {(end_time - start_time) * 1000:.2f} ms\033[0m")

                self.last_keyframe_pose = current_pose_stamped
                self.last_keyframe_time = msg.header.stamp

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"TF变换查找失败: {e}")
        except Exception as e:
            rospy.logerr(f"处理关键帧时发生严重错误: {e}")

    def process_and_update_map(self, msg):
        """
        [优化] 对点云处理和地图更新流程进行了全面优化。
        """
        # [优化 1] 直接将点云读入Numpy数组，避免创建中间list
        # 虽然read_points返回的是生成器，但为了后续的向量化操作，一次性转为数组是必要的。
        # 这里的开销主要在C扩展中，比Python循环快得多。
        all_points_local_np = np.array(list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))), dtype=np.float32)
        if all_points_local_np.shape[0] == 0: return

        # 使用向量化操作快速计算距离并筛选
        distances = np.linalg.norm(all_points_local_np, axis=1)
        obstacle_candidate_points_local = all_points_local_np[distances < (self.lidar_max_range * self.lidar_max_range_threshold_factor)]
        if obstacle_candidate_points_local.shape[0] == 0: 
            return

        # RANSAC地面分割 (这部分依赖PCL，流程不变)
        cloud_pcl = pcl.PointCloud(obstacle_candidate_points_local)
        seg = cloud_pcl.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(self.ransac_dist_thresh)
        inliers, _ = seg.segment()
        # 直接获取去地面后的Numpy数组，不再转回PCL对象
        if not inliers:
            obstacles_no_ground_np = obstacle_candidate_points_local
        else:
            obstacles_no_ground_np = np.delete(obstacle_candidate_points_local, inliers, axis=0)

        if obstacles_no_ground_np.shape[0] == 0: 
            return

        # 获取TF变换
        transform = self.tf_buffer.lookup_transform(
            self.world_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1)
        )
        
        # [优化 2] 使用Numpy进行坐标变换，避免ROS消息的序列化开销
        obstacles_world_np = self.transform_points_numpy(obstacles_no_ground_np, transform)
        
        # [优化 3] 使用Numpy向量化操作进行高度切片，取代Python for循环
        z_values = obstacles_world_np[:, 2]
        height_mask = (z_values > self.slice_min_height_world) & (z_values < self.slice_max_height_world)
        final_obstacle_points_world = obstacles_world_np[height_mask]
        
        # 更新栅格地图
        if final_obstacle_points_world.shape[0] > 0:
            map_points_pub_msg = pc2.create_cloud_xyz32(transform.header, final_obstacle_points_world)
            self.map_points_pub.publish(map_points_pub_msg)
            self.update_occupancy_grid(final_obstacle_points_world)
            self.publish_map()

    # [优化] 新增的辅助函数，用于高效的Numpy点云变换
    def transform_points_numpy(self, points_np, transform: TransformStamped):
        """使用矩阵运算将Numpy点数组从一个坐标系转换到另一个"""
        # 将geometry_msgs/Transform转换为4x4的齐次变换矩阵
        trans = transform.transform.translation
        rot = transform.transform.rotation
        transform_matrix = tft.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
        transform_matrix[0:3, 3] = [trans.x, trans.y, trans.z]
        
        # 将(N, 3)的点云数组转换为(N, 4)的齐次坐标
        points_hom = np.hstack((points_np, np.ones((points_np.shape[0], 1))))
        
        # 应用变换: (4x4) @ (4xN) -> (4xN), then transpose to (N,4)
        points_transformed_hom = (transform_matrix @ points_hom.T).T
        
        # 返回(N, 3)的非齐次坐标
        return points_transformed_hom[:, :3]

    def update_occupancy_grid(self, obstacle_points_world):
        """[优化 4] 使用Numpy向量化操作批量更新栅格地图"""
        # 批量将世界坐标转换为地图栅格坐标
        mx = ((obstacle_points_world[:, 0] - self.map_origin_x) / self.map_resolution).astype(np.int32)
        my = ((obstacle_points_world[:, 1] - self.map_origin_y) / self.map_resolution).astype(np.int32)

        # 创建一个掩码，过滤掉地图范围之外的点
        valid_mask = (mx >= 0) & (mx < self.map_width_cells) & \
                     (my >= 0) & (my < self.map_height_cells)
        
        mx_valid = mx[valid_mask]
        my_valid = my[valid_mask]

        # 使用有效的栅格坐标作为索引，一次性更新所有占据点
        # 注意：对于重复的索引，Numpy的+=操作可能只执行一次。
        # 使用np.add.at可以确保对重复索引的栅格进行累加。
        np.add.at(self.log_odds_map, (my_valid, mx_valid), self.log_odds_occupied)

        # 限制log-odds值的范围 (clip是高效的C实现)
        np.clip(self.log_odds_map, self.log_odds_clamp_min, self.log_odds_clamp_max, out=self.log_odds_map)

    # --- 以下函数保持不变 ---
    def get_current_pose(self, timestamp):
        """获取指定时间戳的机器人位姿"""
        try:
            transform = self.tf_buffer.lookup_transform(self.world_frame, self.robot_base_frame, timestamp, rospy.Duration(0.2))
            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position = transform.transform.translation
            pose.pose.orientation = transform.transform.rotation
            return pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"无法获取当前位姿: {e}")
            return None

    def is_keyframe(self, current_pose):
        """判断当前帧是否为关键帧"""
        if self.last_keyframe_pose is None:
            return True

        dx = current_pose.pose.position.x - self.last_keyframe_pose.pose.position.x
        dy = current_pose.pose.position.y - self.last_keyframe_pose.pose.position.y
        dist = math.sqrt(dx*dx + dy*dy)
        if dist > self.keyframe_dist_thresh:
            rospy.loginfo(f"关键帧原因: 位移阈值触发 ({dist:.2f}m > {self.keyframe_dist_thresh:.2f}m)")
            return True

        q_curr = current_pose.pose.orientation
        q_last = self.last_keyframe_pose.pose.orientation
        _, _, yaw_curr = tft.euler_from_quaternion([q_curr.x, q_curr.y, q_curr.z, q_curr.w])
        _, _, yaw_last = tft.euler_from_quaternion([q_last.x, q_last.y, q_last.z, q_last.w])
        angle_diff = abs(math.degrees(yaw_curr) - math.degrees(yaw_last))
        if angle_diff > 180: 
            angle_diff = 360 - angle_diff
        if angle_diff > self.keyframe_angle_thresh:
            rospy.loginfo(f"关键帧原因: 角度阈值触发 ({angle_diff:.2f}° > {self.keyframe_angle_thresh:.2f}°)")
            return True
            
        if (current_pose.header.stamp - self.last_keyframe_time).to_sec() > self.keyframe_time_thresh:
            rospy.loginfo("关键帧原因: 时间阈值触发")
            return True
            
        return False

    def publish_map(self):
        """将log-odds地图转换为OccupancyGrid消息并发布"""
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = self.world_frame
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.map_width_cells
        grid_msg.info.height = self.map_height_cells
        grid_msg.info.origin = Pose(Point(self.map_origin_x, self.map_origin_y, 0), Quaternion(0, 0, 0, 1))
        prob_map = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds_map))
        occupancy_data = (prob_map * 100).astype(np.int8)
        occupancy_data[self.log_odds_map == 0] = 10
        grid_msg.data = occupancy_data.flatten().tolist()
        self.map_pub.publish(grid_msg)
        rospy.loginfo_once("已成功发布第一张'仅障碍物'地图！")

    def world_to_map_coords(self, world_x, world_y):
        """将世界坐标转换为地图栅格坐标"""
        mx = int((world_x - self.map_origin_x) / self.map_resolution)
        my = int((world_y - self.map_origin_y) / self.map_resolution)
        if 0 <= mx < self.map_width_cells and 0 <= my < self.map_height_cells:
            return mx, my
        return None, None

if __name__ == '__main__':
    try:
        MapBuilder()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("地图构建节点已终止。")