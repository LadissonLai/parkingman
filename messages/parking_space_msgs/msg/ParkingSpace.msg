# 单个停车位的信息 (在世界坐标系下)

uint32 id               # 在一帧内的唯一ID
float32 confidence      # 检测的置信度

geometry_msgs/Pose pose # 车位的位姿 (中心点位置+方向)

float32 width           # 沿着箭头方向的宽度 (米)
float32 height          # 沿着箭头垂直方向的高度 (米)