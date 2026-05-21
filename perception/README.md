# Perception 功能包

`perception` 是 LLMParking 系统中的核心感知模块。它主要负责基于俯视图（BEV）和激光雷达等多模态传感器信息，利用 YOLO 深度学习模型等技术实现对当前环境的感知：主要是检测周围的空闲停车位、静态车辆信息，并将局部感知结果转换到全局坐标系（如 `map` 统一坐标系）中，构建并维护一个全局停车位及车辆属性地图。这些信息随后会被封装发布给基于大语言模型（LLM）的决策中心。

---

## 核心功能

1. **BEV 空闲车位检测**：基于拼接的俯视图图像（BEV），使用预训练的 YOLO OBB 模型进行旋转目标检测，识别空闲停车位。
2. **局部到全局的建图（Map Building）**：利用TF树将检测到的局部车位坐标转换至全局地图架构中，剔除误检、合并重叠检测，形成稳定的全局车位地图。
3. **车辆检测与属性记录**：负责感知环境中的全景车辆分布状态和颜色/ID等属性。
4. **生成全局描述发送给大语言模型**：构建包含每个空位相对位置、朝向以及周围其他车辆信息的全局场景描述协议（如 `GlobalParkingSpaceDescription`, `GlobalVehicleDescription`）。
5. **激光点云构建静态占据栅格地图**（Lidar to OccupancyGrid）。

---

## 核心脚本介绍

主要脚本位于 `scripts/` 目录下：

- **`space_detect_and_tf_to_map.py`**
  - **功能**：最重要的感知入口节点。使用 YOLO 模型（如 `yolo_obb_best.pt`）订阅自车周围的 BEV 图像进行 2D 空闲停车位边界检测，然后联合相机的外参及 TF 树信息，将像素坐标解算为全局物理坐标下的 3D 车位参数，并发布。
- **`build_ps_map.py`**
  - **功能**：停车位聚合与建图节点。接收连续帧的单次检测结果，采用时序滤波及多帧位置聚类，更新并维护一个稳定可靠的“已确认车位地图”（Confirmed Spaces Map）。
- **`vehicle_detect_and_map.py`**
  - **功能**：全局车辆及其属性状态建图节点。对其他车辆的位置、ID、颜色等信息进行长期维护，并转换为面向 LLM 决策的可阅读格式。
- **`lidar2grid_static.py`**
  - **功能**：将激光雷达点云（Pointcloud2）实时压缩转换成 2D 占据栅格地图（Occupancy Grid Map）。

---

## 订阅与发布的话题 (Sub & Pub)

### 1. `space_detect_and_tf_to_map.py`
- **订阅的话题**：
  - `/bev/image_stitched_hard` (`sensor_msgs/Image`): 拼接好或 IPM（逆透视变换）处理后的鸟瞰图（BEV）。
- **发布的话题**：
  - `/parking_spaces_data` (自定义类型 `ParkingSpaceArray`): 输出检测到并完成坐标转换的车位信息流。
  - `/parking_space_detections_image` (`sensor_msgs/Image`): 带有 YOLO 画框的可视化检测结果图像。
  - `/parking_space_markers` (`visualization_msgs/MarkerArray`): 提供给 RViz 显示的 3D 车位框可视化。

### 2. `build_ps_map.py`
- **订阅的话题**：
  - `/parking_spaces_data` (自定义类型 `ParkingSpaceArray`): 接收单帧即时的车位数据验证。
  - `/perception/parking_space_map/reset` (`std_msgs/Bool`): 重置停车位地图的指令。
- **发布的话题**：
  - `/parking_map/confirmed_spaces_in_world` (自定义 `ParkingSpaceArray`): 经过时间检验（Tracking）并确认的全局可用停车位序列。
  - `/perception/global_parking_space_description` (自定义 `GlobalParkingSpaceDescription`): **【核心】** 提供给 LLM 决策模块的最顶层语义数据描述，包含带数字ID、朝向、大小等的文本化语义封装。
  - `/parking_map_markers` & `/perception/parking_space_id_text_markers` (`visualization_msgs/MarkerArray`): 在 RViz 中持久化的数字 ID 及框可视化。
  - `/parking_map_info_text` (`jsk_rviz_plugins/OverlayText`): Rviz 屏幕上的文本调试面板。

### 3. `vehicle_detect_and_map.py`
- **发布的话题**：
  - `/perception/global_vehicle_description` (自定义 `GlobalVehicleDescription`): 输出带车辆特征信息的语义流（位置/颜色）。
  - `/perception/global_vehicle_color_id_viz` (`visualization_msgs/MarkerArray`): 标记其它车辆可视化。

---

## 话题消息格式示例

虽然具体的消息在内部 message pkg（如 `llm_parking_msgs`）中定义，其核心结构推导如下：
- **`ParkingSpaceArray`**:
  - `header` (包含 frame_id 为 map/world_frame)
  - `spaces` (列表): 每一个空间包含如 `id`, `center_pose` (Pose), `length`, `width`, `category` 等字段。
- **`GlobalParkingSpaceDescription`** / **`GlobalVehicleDescription`**:
  - 用来将空间/车辆信息按照利于 LLM（Prompt形式）理解的方式组织打包，内部通常包括所有被检出对象的数组，或者直接附带结构化 JSON/String 文本串字段。

---

## 如何使用

1. **准备与编译**
   请确保您已经完成了 `llm_parking_msgs`（或相关自定消息包）等依赖的编译。
   ```bash
   cd ~/LLM_ws
   catkin_make
   source devel/setup.bash
   ```

2. **启动 BEV 检测及转换系统**
   你可以通过自带的 launch 文件将核心建图功能带起来。
   ```bash
   roslaunch perception ps_detection.launch world_frame:=parking_start_map
   ```
   **关于这个 launch 文件**：
    - 此文件会自动拉起 `space_detect_and_tf_to_map.py` 节点。
    - 并自动加载预存的检测权重模型参数 ：`model_path:=$(find perception)/weights/yolo_obb_best.pt`。

3. **启动地图聚合模块 (PS Map Building)**
   在一个新的终端启动车位管理机制：
   ```bash
   rosrun perception build_ps_map.py
   ```

4. **车辆检测映射等其他节点**
   需要车辆语义和静态占据栅格图支持时：
   ```bash
   rosrun perception vehicle_detect_and_map.py
   rosrun perception lidar2grid_static.py
   ```

5. **可视化**
   系统运行时，打开 `RViz`，配置添加：
   - 图像视图： `/parking_space_detections_image`
   - `MarkerArray` 视图： `/parking_map_markers` 和 `/parking_space_markers` 以直观排查建图效果。
