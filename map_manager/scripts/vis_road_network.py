import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math
from map_model import RoadMap

# ================= 数据准备 (保持不变) =================
xml_data = """
<RoadNet>
    <nodes>
        <n id="1" p="0.0 0.0 0.0 0.0" />
        <n id="2" p="10.0 0.0 0.0 0.0" />
        <n id="3" p="15.0 0.0 0.0 -90.0" />
        <n id="4" p="15.0 -10.0 0.0 -90.0" />
        <n id="5" p="15.0 -20.0 0.0 -90.0" />
    </nodes>

    <ways>
        <w id="101" nodes="1 2" type="Entry" />
        <w id="102" nodes="2 3 4" type="RightTurn" />
        <w id="103" nodes="4 5" type="Exit" />
    </ways>

    <relations>
        <r from="101" to="102" />
        <r from="102" to="103" />
    </relations>
</RoadNet>
"""

def calculate_center_and_orientation(xs, ys):
    """
    计算折线的几何中心位置，以及该处的切线向量
    Returns: (center_x, center_y, dir_x, dir_y)
    """
    if len(xs) < 2:
        return xs[0], ys[0], 1.0, 0.0
    
    # 1. 计算总长
    dists = []
    total_len = 0
    for i in range(len(xs) - 1):
        d = math.hypot(xs[i+1] - xs[i], ys[i+1] - ys[i])
        dists.append(d)
        total_len += d
    
    # 2. 找到中点
    target_dist = total_len / 2.0
    current_dist = 0
    
    for i, d in enumerate(dists):
        if current_dist + d >= target_dist:
            # 中点在这段 (i) 上
            remain = target_dist - current_dist
            if d == 0: ratio = 0
            else: ratio = remain / d
            
            # 中心坐标
            cx = xs[i] + (xs[i+1] - xs[i]) * ratio
            cy = ys[i] + (ys[i+1] - ys[i]) * ratio
            
            # 该段的单位方向向量
            dx = xs[i+1] - xs[i]
            dy = ys[i+1] - ys[i]
            length = math.hypot(dx, dy)
            if length > 0:
                dx /= length
                dy /= length
            else:
                dx, dy = 1.0, 0.0
                
            return cx, cy, dx, dy
            
        current_dist += d
    
    return xs[-1], ys[-1], 1.0, 0.0

def plot_static_map_optimized(road_map):
    # 设置绘图风格
    fig, ax = plt.subplots(figsize=(10, 8), dpi=100)
    ax.set_aspect('equal')
    
    # 背景与网格
    ax.grid(True, linestyle=':', color='#999999', alpha=0.4)
    ax.set_facecolor('#fdfdfd') 
    for spine in ax.spines.values():
        spine.set_edgecolor('#dddddd')

    ax.set_title("Static Road Network Topology (With Node Detail)", fontsize=14, pad=20, fontweight='bold', color='#333333')
    ax.set_xlabel("X (m)", color='#666666')
    ax.set_ylabel("Y (m)", color='#666666')

    # 绘制每一条 Way
    for wid, way in road_map.ways.items():
        # 获取坐标点
        xs = [road_map.nodes[nid].x for nid in way.node_ids]
        ys = [road_map.nodes[nid].y for nid in way.node_ids]

        # 1. 绘制道路线 (深灰色)
        ax.plot(xs, ys, color='#444444', linewidth=2.5, alpha=0.8, solid_capstyle='round', zorder=2)
        
        # 2. 绘制节点 (现在可以清晰看到了)
        # 用空心圆圈表示 Node，既显眼又不遮挡线条
        ax.scatter(xs, ys, color='white', edgecolors='#666666', s=40, linewidths=1.5, zorder=3, label='Nodes')

        if len(xs) >= 2:
            # 3. 绘制末端方向箭头 (保持不变)
            last_x_start, last_y_start = xs[-2], ys[-2]
            last_x_end, last_y_end = xs[-1], ys[-1]
            arrow_x = last_x_start + (last_x_end - last_x_start) * 0.8
            arrow_y = last_y_start + (last_y_end - last_y_start) * 0.8
            dx = last_x_end - last_x_start
            dy = last_y_end - last_y_start
            
            arrow = patches.FancyArrow(
                arrow_x, arrow_y, dx*0.01, dy*0.01, 
                width=0, head_width=0.8, head_length=1.0, 
                color='#1f77b4', zorder=4
            )
            ax.add_patch(arrow)

        # 4. 绘制 Way ID (侧边偏移)
        # 获取中心点和方向向量
        cx, cy, dx, dy = calculate_center_and_orientation(xs, ys)
        
        # 计算法向量 (向左旋转90度: -y, x)
        # 如果需要向右偏移，可以改为 (y, -x)
        normal_x = -dy
        normal_y = dx
        
        # 设置偏移距离 (例如 1.5 米)
        offset_distance = 1.5
        label_x = cx + normal_x * offset_distance
        label_y = cy + normal_y * offset_distance
        
        # 绘制连接线 (可选，让归属更清晰，用极细的虚线)
        ax.plot([cx, label_x], [cy, label_y], color='#cccccc', linestyle=':', linewidth=0.8, zorder=1)

        # 绘制文本框
        ax.text(
            label_x, label_y, 
            str(wid), 
            fontsize=9, 
            fontweight='bold',
            color='#d62728', 
            ha='center', va='center',
            bbox=dict(
                boxstyle="round,pad=0.2", 
                fc="white", 
                ec="#e0e0e0", 
                alpha=0.9,
                linewidth=0.5
            ),
            zorder=5
        )

    # 稍微调整视口范围以容纳偏移的标签
    ax.autoscale_view()
    
    # 防止重复图例
    # ax.legend() # 静态图通常不需要图例，元素自解释
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    r_map = RoadMap()
    # r_map.load_from_string(xml_data)
    r_map.load_from_file("town04_parkinglot.xml")
    plot_static_map_optimized(r_map)