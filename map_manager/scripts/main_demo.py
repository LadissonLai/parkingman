#! /usr/bin/env python3
from map_model import RoadMap
from trajectory_engine import TrajectoryEngine

# XML 数据：模拟一个停车场入口进入并右转泊入的过程
# 注意：角度使用度数，右手系 (逆时针为正，东=0，北=90，南=-90)
xml_data = """
<RoadNet>
    <nodes>
        <n id="1" p="0.0 0.0 0.0 0.0" />
        <n id="2" p="10.0 0.0 0.0 0.0" />
        
        <!-- 转弯的中间控制点，故意设为直角顶点 -->
        <!-- 如果是 Hermite，车会硬生生开到 (15, 0) 再转90度 -->
        <!-- 如果是 B-Spline，车会从 (10,0) 开始提前内切，不经过 (15,0)，直接划出弧线去 (15,-5) -->
        <n id="3" p="15.0 0.0 0.0 -90.0" /> 
        
        <n id="4" p="15.0 -10.0 0.0 -90.0" />
    </nodes>

    <ways>
        <w id="101" nodes="1 2" type="Entry" />
        <w id="102" nodes="2 3 4" type="RightTurn" /> 
    </ways>

    <relations>
        <r from="101" to="102" />
    </relations>
</RoadNet>
"""

def main():
    # 1. 初始化
    print(">>> Map Loading...")
    road_map = RoadMap()
    road_map.load_from_string(xml_data)
    engine = TrajectoryEngine(road_map)

    # 2. 模拟任务
    start_pose = {'x': 0.5, 'y': 0.0, 'theta': 0.0} 
    target_pose = {'x': 15.0, 'y': -14.0, 'theta': -90.0}

    # 3. 规划
    start_way, _ = engine.find_nearest_location(start_pose['x'], start_pose['y'], start_pose['theta'])
    end_way, _ = engine.find_nearest_location(target_pose['x'], target_pose['y'], target_pose['theta'])
    
    print(f">>> Plan: From Way {start_way} to Way {end_way}")
    
    way_path = engine.search_topology_path(start_way, end_way)
    
    # 4. 生成平滑轨迹
    # step_size 越小，轨迹越平滑
    traj = engine.generate_dense_trajectory(way_path, step_size=0.5)
    
    print(f">>> Trajectory Generated: {len(traj)} points")

    # 5. 验证关键点的平滑性 (特别是转弯处)
    # 我们检查 Way 102 (Node 2 -> Node 3) 的中间点
    print("\n--- Checking Curve Smoothness (Turn) ---")
    
    # 找到大约在转弯中间的点 (x 在 10~15之间)
    for i, p in enumerate(traj):
        if 11.0 < p['x'] < 14.0:
            print(f"Point {i}: x={p['x']:.2f}, y={p['y']:.2f}, theta={p['theta']:.2f}°")
            # 预期：theta 应该从 0 逐渐变到 -90，而不是突变

    print("\n--- End Points ---")
    print(traj[-1])

if __name__ == "__main__":
    main()