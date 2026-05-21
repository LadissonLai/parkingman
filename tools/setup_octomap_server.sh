<launch>
  <node pkg="octomap_server" type="octomap_server_node" name="octomap_server">
    <!-- 栅格分辨率，比如 0.05米 -->
    <param name="resolution" value="0.05" />
    
    <!-- FAST_LIO2 的全局坐标系通常是 camera_init 或 map -->
    <param name="frame_id" type="string" value="camera_init" />
    
    <!-- 传感器最大有效范围 -->
    <param name="sensor_model/max_range" value="50.0" />
    
    <!-- 过滤掉地面（高度低于该值的点云不视为障碍物） -->
    <param name="occupancy_min_z" value="0.1" /> 
    <!-- 过滤掉过高的障碍物（比如天花板或树冠） -->
    <param name="occupancy_max_z" value="2.0" />
    
    <!-- 订阅 FAST_LIO2 发布的当前帧配准点云 -->
    <remap from="cloud_in" to="/cloud_registered" />
  </node>
</launch>