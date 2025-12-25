import numpy as np
import math
import collections
from scipy.interpolate import splprep, splev
from map_model import RoadMap

class TrajectoryEngine:
    def __init__(self, road_map: RoadMap):
        self.map = road_map

    def normalize_angle_deg(self, angle):
        """ 将角度归一化到 [-180, 180] """
        return (angle + 180) % 360 - 180

    def get_trajectory(self, start_pose, target_pose, step_size=0.2):
        """
        [新增] 高层接口：直接计算从起点到终点的轨迹
        包含：定位 -> 拓扑搜索 -> 几何生成 -> 裁剪/线性插值
        """
        # 1. 定位 (使用较宽松的阈值以适应稀疏路网)
        s_way, _ = self.find_nearest_location(
            start_pose['x'], start_pose['y'], start_pose['theta'], 
            dist_threshold=10.0
        )
        e_way, _ = self.find_nearest_location(
            target_pose['x'], target_pose['y'], target_pose['theta'], 
            dist_threshold=10.0
        )

        if s_way is None or e_way is None:
            print(f"[Engine] Localization failed. StartWay: {s_way}, EndWay: {e_way}")
            return []

        # 2. 拓扑规划
        way_path = self.search_topology_path(s_way, e_way)
        if not way_path:
            print(f"[Engine] No topology path found between Way {s_way} and Way {e_way}")
            return []

        # 3. 生成稠密轨迹 (包含裁剪和线性插值逻辑)
        trajectory = self.generate_dense_trajectory(way_path, start_pose, target_pose, step_size)
        
        return trajectory

    def find_nearest_location(self, x, y, theta_deg, dist_threshold=10.0, angle_threshold=30.0):
        best_way_id = None
        best_node_idx = -1
        min_score = float('inf')
        target_pos = np.array([x, y])

        for wid, way in self.map.ways.items():
            for idx, nid in enumerate(way.node_ids):
                node = self.map.nodes[nid]
                dist = np.linalg.norm(np.array([node.x, node.y]) - target_pos)
                angle_diff = abs(self.normalize_angle_deg(node.theta_deg - theta_deg))

                if dist < dist_threshold and angle_diff < angle_threshold:
                    if dist < min_score:
                        min_score = dist
                        best_way_id = wid
                        best_node_idx = idx
        return best_way_id, best_node_idx

    def search_topology_path(self, start_way_id, end_way_id):
        if start_way_id == end_way_id:
            return [start_way_id]
        queue = collections.deque([[start_way_id]])
        visited = set([start_way_id])
        while queue:
            path = queue.popleft()
            node = path[-1]
            if node == end_way_id:
                return path
            for neighbor in self.map.adjacency.get(node, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    new_path = list(path)
                    new_path.append(neighbor)
                    queue.append(new_path)
        return None

    def _generate_linear_trajectory(self, start, end, step_size):
        dist = math.sqrt((end['x'] - start['x'])**2 + (end['y'] - start['y'])**2)
        if dist < 1e-3: return [end]
        num_steps = max(2, int(dist / step_size))
        traj = []
        start_z = start.get('z', 0.0)
        end_z = end.get('z', 0.0)
        for i in range(num_steps):
            t = i / (num_steps - 1)
            x = start['x'] + t * (end['x'] - start['x'])
            y = start['y'] + t * (end['y'] - start['y'])
            z = start_z + t * (end_z - start_z)
            diff = self.normalize_angle_deg(end['theta'] - start['theta'])
            theta = self.normalize_angle_deg(start['theta'] + t * diff)
            traj.append({'x': round(x, 3), 'y': round(y, 3), 'z': round(z, 3), 'theta': round(theta, 3)})
        return traj

    def _get_pruned_control_points(self, skeleton_nodes, start_pose, end_pose):
        if not skeleton_nodes: return []

        def find_closest_segment_idx(nodes, pose):
            p = np.array([pose['x'], pose['y']])
            min_d = float('inf')
            best_idx = 0
            for i in range(len(nodes) - 1):
                p1 = np.array([nodes[i].x, nodes[i].y])
                p2 = np.array([nodes[i+1].x, nodes[i+1].y])
                d = np.linalg.norm(p1 - p)
                if d < min_d:
                    min_d = d
                    best_idx = i
            if np.linalg.norm(np.array([nodes[-1].x, nodes[-1].y]) - p) < min_d:
                best_idx = len(nodes) - 1
            return best_idx

        if start_pose:
            idx = find_closest_segment_idx(skeleton_nodes, start_pose)
            skeleton_nodes = skeleton_nodes[idx+1:]
        
        if end_pose:
            idx = find_closest_segment_idx(skeleton_nodes, end_pose)
            skeleton_nodes = skeleton_nodes[:idx]

        final_points = []
        if start_pose:
            final_points.append([start_pose['x'], start_pose['y'], 0.0])
        for n in skeleton_nodes:
            final_points.append([n.x, n.y, n.z])
        if end_pose:
            final_points.append([end_pose['x'], end_pose['y'], 0.0])

        return final_points

    def generate_dense_trajectory(self, way_path, start_pose=None, end_pose=None, step_size=0.2):
        if not way_path: return []

        # 同 Way 线性插值优化
        if len(way_path) == 1 and start_pose is not None and end_pose is not None:
            return self._generate_linear_trajectory(start_pose, end_pose, step_size)

        # B-Spline 逻辑
        skeleton_nodes = []
        for i, wid in enumerate(way_path):
            way = self.map.ways[wid]
            nodes = [self.map.nodes[nid] for nid in way.node_ids]
            if i > 0 and skeleton_nodes and nodes[0].id == skeleton_nodes[-1].id:
                nodes = nodes[1:]
            skeleton_nodes.extend(nodes)

        if len(skeleton_nodes) < 2: return []

        raw_points_list = self._get_pruned_control_points(skeleton_nodes, start_pose, end_pose)
        
        if len(raw_points_list) < 2:
            if start_pose and end_pose:
                 return self._generate_linear_trajectory(start_pose, end_pose, step_size)
            else:
                return []

        points_np = np.array(raw_points_list).T
        x_pts, y_pts, z_pts = points_np[0], points_np[1], points_np[2]

        tan_len = 3.0
        
        if start_pose: s_rad = math.radians(start_pose['theta'])
        else: s_rad = 0 
        p_start_aux_x = x_pts[0] + math.cos(s_rad) * tan_len
        p_start_aux_y = y_pts[0] + math.sin(s_rad) * tan_len
        
        if end_pose: e_rad = math.radians(end_pose['theta'])
        else: e_rad = 0
        p_end_aux_x = x_pts[-1] - math.cos(e_rad) * tan_len
        p_end_aux_y = y_pts[-1] - math.sin(e_rad) * tan_len

        x_pts = np.insert(x_pts, 1, p_start_aux_x)
        y_pts = np.insert(y_pts, 1, p_start_aux_y)
        z_pts = np.insert(z_pts, 1, z_pts[0])

        x_pts = np.insert(x_pts, -1, p_end_aux_x)
        y_pts = np.insert(y_pts, -1, p_end_aux_y)
        z_pts = np.insert(z_pts, -1, z_pts[-1])

        try:
            clean_pts = np.array([x_pts, y_pts, z_pts])
            valid_cols = [0]
            for c in range(1, clean_pts.shape[1]):
                if np.linalg.norm(clean_pts[:2, c] - clean_pts[:2, c-1]) > 0.1:
                    valid_cols.append(c)
            clean_pts = clean_pts[:, valid_cols]

            if clean_pts.shape[1] <= 3: 
                k_order = clean_pts.shape[1] - 1
            else:
                k_order = 3

            tck, u = splprep(clean_pts, s=0.0, k=k_order) 
        except Exception:
            if start_pose and end_pose:
                return self._generate_linear_trajectory(start_pose, end_pose, step_size)
            return []

        total_dist = 0
        for k in range(clean_pts.shape[1]-1):
            total_dist += np.linalg.norm(clean_pts[:2, k+1] - clean_pts[:2, k])
        
        num_samples = max(2, int(total_dist / step_size))
        u_new = np.linspace(0, 1, num_samples)
        
        new_points = splev(u_new, tck)
        first_der = splev(u_new, tck, der=1)
        dx_dt, dy_dt = first_der[0], first_der[1]

        dense_traj = []
        for i in range(len(u_new)):
            yaw_deg = math.degrees(math.atan2(dy_dt[i], dx_dt[i]))
            dense_traj.append({
                'x': round(new_points[0][i], 3), 
                'y': round(new_points[1][i], 3), 
                'z': round(new_points[2][i], 3), 
                'theta': round(self.normalize_angle_deg(yaw_deg), 3)
            })
            
        return dense_traj