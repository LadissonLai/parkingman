import matplotlib.pyplot as plt
import numpy as np
import math
from map_model import RoadMap
from trajectory_engine import TrajectoryEngine

# 使用同样的路网数据
xml_data = """
<RoadNet>
    <nodes>
        <n id="1" p="0.0 0.0 0.0 0.0" />
        <n id="2" p="10.0 0.0 0.0 0.0" />
        <!-- 直角转弯控制点 -->
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

def plot_trajectory_check(road_map, trajectory, start_p, end_p):
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.set_title("Exact Trajectory Verification (Pruned)", fontsize=14)
    
    # 1. 绘制背景路网 (灰色虚线骨架)
    for wid, way in road_map.ways.items():
        xs = [road_map.nodes[nid].x for nid in way.node_ids]
        ys = [road_map.nodes[nid].y for nid in way.node_ids]
        ax.plot(xs, ys, 'k--.', alpha=0.2) 
        
        # 简单的 Way ID 标注
        mid_x = np.mean(xs)
        mid_y = np.mean(ys)
        ax.text(mid_x, mid_y, f"W{wid}", color='gray', fontsize=8, alpha=0.5)

    # 2. 绘制生成的轨迹 (红色实线)
    if trajectory:
        traj_xs = [p['x'] for p in trajectory]
        traj_ys = [p['y'] for p in trajectory]
        ax.plot(traj_xs, traj_ys, 'r-', linewidth=3, label='Trajectory', alpha=0.9)

        # 3. 绘制轨迹上的方向箭头 (每隔一段)
        step = max(1, len(trajectory) // 10) 
        for i in range(0, len(trajectory), step):
            p = trajectory[i]
            rad = np.radians(p['theta'])
            ax.arrow(p['x'], p['y'], math.cos(rad)*0.5, math.sin(rad)*0.5,
                     head_width=0.3, fc='purple', ec='purple', alpha=0.6)
    else:
        print("Error: No trajectory to plot.")

    # 4. 标记起点 (绿色) 和 终点 (红色)
    # 起点
    ax.plot(start_p['x'], start_p['y'], 'go', markersize=10, label='Start', zorder=10)
    s_rad = np.radians(start_p['theta'])
    ax.arrow(start_p['x'], start_p['y'], math.cos(s_rad)*2, math.sin(s_rad)*2, color='green', width=0.1)

    # 终点
    ax.plot(end_p['x'], end_p['y'], 'rx', markersize=10, markeredgewidth=3, label='End', zorder=10)
    e_rad = np.radians(end_p['theta'])
    ax.arrow(end_p['x'], end_p['y'], math.cos(e_rad)*2, math.sin(e_rad)*2, color='red', width=0.1)

    ax.legend()
    plt.tight_layout()
    plt.show()

def main():
    # === 定义测试任务 ===
    # 长距离测试
    start_pose = {'x': 286.52, 'y': 209.31, 'theta': 90.44}
    target_pose = {'x': 304.71, 'y': 201.38, 'theta': 88.81}

    r_map = RoadMap()
    r_map.load_from_file("static_map.xml")
    engine = TrajectoryEngine(r_map)

    # === 测试 Case ===
    print(">>> Test Case: Normal Navigation (Cross Ways)")

    # 调用新接口
    trajectory = engine.get_trajectory(start_pose, target_pose, step_size=0.1)
    
    if trajectory:
        print(f"Success! Generated {len(trajectory)} points.")
        print(f"Start Point: {trajectory[0]}")
        print(f"End Point:   {trajectory[-1]}")
    else:
        print("Failed to generate trajectory.")

    plot_trajectory_check(r_map, trajectory, start_pose, target_pose)

if __name__ == "__main__":
    main()