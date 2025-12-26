# 单个停车位的信息带时间戳 (在世界坐标系下)

std_msgs/Header header   # 包含时间戳和坐标系ID (e.g., "parking_start_map")

uint32 id               # 在一帧内的唯一ID
float32 confidence      # 检测的置信度

geometry_msgs/Pose pose # 车位的位姿 (中心点位置+方向)

float32 width           # 车位的宽度 (米)
float32 height          # 车位的高度 (米)