import json
import os

class DataFormatter:
    def __init__(self, config):
        self.config = config
        self.map_content = self._load_map(config['paths']['map_file'])

    def _load_map(self, path):
        """读取路网XML内容，压缩多余空白以节省Token"""
        try:
            abs_path = os.path.abspath(path)
            with open(abs_path, 'r', encoding='utf-8') as f:
                # 移除换行和多余空格，压缩XML结构
                lines = [line.strip() for line in f.readlines() if line.strip()]
                return "".join(lines)
        except Exception as e:
            print(f"[Error] Failed to load map XML: {e}")
            return "<RoadNet>Error</RoadNet>"

    def format_observation(self, vehicles_msg, spaces_msg):
        """
        [关键修复] 将感知消息转为文本描述，保留颜色和尺寸信息。
        格式示例:
        [Vehicles] ID:1, Color:Red, Pos:[280.1,190.2], Yaw:90.0, Dim:[4.5,1.8,1.5]; ...
        [Free_Spaces] ID:101, Pos:[285.5,180.1], Yaw:0.0, Size:[5.0,2.5]; ...
        """
        v_str_list = []
        s_str_list = []

        # 处理车辆
        if vehicles_msg and hasattr(vehicles_msg, 'vehicles'):
            for v in vehicles_msg.vehicles:
                # 保留颜色、坐标、航向角、以及长宽高(dimensions x/y/z)
                v_info = (
                    f"ID:{v.id}, Color:{v.color}, "
                    f"Pos:[{v.position.x:.2f},{v.position.y:.2f}], Yaw:{v.yaw:.2f}, "
                    f"Dim:[{v.dimensions.x:.2f},{v.dimensions.y:.2f},{v.dimensions.z:.2f}]"
                )
                v_str_list.append(v_info)

        # 处理车位
        if spaces_msg and hasattr(spaces_msg, 'spaces'):
            for s in spaces_msg.spaces:
                # 保留车位ID、中心坐标、航向角、以及长宽(length/width)
                s_info = (
                    f"ID:{s.id}, Pos:[{s.x:.2f},{s.y:.2f}], Yaw:{s.yaw_degrees:.2f}, "
                    f"Size:[{s.length:.2f},{s.width:.2f}]"
                )
                s_str_list.append(s_info)

        # 构建最终字符串
        v_final = "; ".join(v_str_list) if v_str_list else "None"
        s_final = "; ".join(s_str_list) if s_str_list else "None"

        return f"[Vehicles] {v_final}\n[Free_Spaces] {s_final}"

    def build_system_prompt(self):
        """构建优化后的System Prompt"""
        sys_conf = self.config['system']
        
        prompt = (
            f"Role: You are an autonomous parking agent driven by Large Language Models.\n"
            f"Task: Follow the Human Instruction to explore the parking lot and find a parking slot. "
            f"Make decisions based on the Global Map, Observation (pay attention to vehicle Color/Dimensions and Space Size), "
            f"Ego Status, and Trajectory History.\n\n"
            f"--- Context ---\n"
            f"Global Map (XML): {self.map_content}\n"
            f"Entrance: {sys_conf['entrance_coords']}\n"
            f"Exit: {sys_conf['exit_coords']}\n"
            f"Human Instruction: \"{sys_conf['instruction']}\"\n\n"
            f"--- Output Format ---\n"
            f"Output a valid JSON object describing the next action.\n"
            f"1. Exploring: {{\"next_waypoint\": [x, y, yaw], \"start_parking\": false, \"slot_id\": -1}}\n"
            f"2. Parking: {{\"next_waypoint\": [], \"start_parking\": true, \"slot_id\": <target_id>}}\n"
            f"Note: 'yaw' uses Degrees (-180 to 180)."
        )
        return prompt

    def build_user_input(self, observation, ego_status, history):
        """
        构建用户输入
        """
        # 历史轨迹保留2位小数
        hist_str = str([[round(p[0],2), round(p[1],2), round(p[2],2)] for p in history])
        
        content = (
            f"Observation:\n{observation}\n"
            f"Ego Status: x={ego_status['x']:.2f}, y={ego_status['y']:.2f}, yaw={ego_status['yaw']:.2f}\n"
            f"History: {hist_str}"
        )
        return content

    def build_assistant_output(self, next_ego, is_parking, slot_id):
        """构建模型输出标签"""
        output = {
            "next_waypoint": [round(next_ego['x'], 2), round(next_ego['y'], 2), round(next_ego['yaw'], 2)] if next_ego else [],
            "start_parking": is_parking,
            "slot_id": slot_id
        }
        return json.dumps(output)

    def save_to_jsonl(self, conversations, filename):
        """保存数据"""
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        data = {"messages": conversations}
        with open(filename, 'a', encoding='utf-8') as f:
            f.write(json.dumps(data, ensure_ascii=False) + "\n")
        print(f"[Success] Saved episode with {len(conversations)} turns to {filename}")