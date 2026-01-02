#!/usr/bin/env python3
import rospy
import yaml
import threading
import copy
import sys
import time
import os
import rospkg
import select
import tty
import termios
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

# 确保这里的 msg 导入路径正确
from llm_perception_msgs.msg import GlobalVehicleDescription, GlobalParkingSpaceDescription

from data_utils import DataFormatter

class DataCollectorNode:
    def __init__(self, config_path):
        rospy.init_node('parking_decision_dataset_collector', anonymous=True)
        
        # 1. 加载配置
        try:
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
        except FileNotFoundError:
            rospy.logerr(f"Config file not found: {config_path}")
            sys.exit(1)
        
        self.formatter = DataFormatter(self.config)
        
        # 2. 运行时状态变量
        self.current_odom = None
        self.current_vehicles = None
        self.current_spaces = None
        self.lock = threading.Lock()
        
        # 3. 数据Buffer
        self.episode_buffer = [] 
        
        # 4. 订阅话题
        rospy.Subscriber(self.config['topics']['ego_odom'], Odometry, self.odom_cb)
        rospy.Subscriber(self.config['topics']['objects'], GlobalVehicleDescription, self.veh_cb)
        rospy.Subscriber(self.config['topics']['spaces'], GlobalParkingSpaceDescription, self.space_cb)
        
        # 5. 重置感知地图
        self.reset_parking_map_pub = rospy.Publisher('/perception/parking_space_map/reset', Bool, queue_size=1, latch=True)
        self.reset_vehicle_map_pub = rospy.Publisher('/perception/vehicle_map/reset', Bool, queue_size=1, latch=True)
        
        # 等待连接建立（可选，但推荐）并发送重置信号
        rospy.sleep(1.0) 
        reset_msg = Bool()
        reset_msg.data = True
        self.reset_parking_map_pub.publish(reset_msg)
        self.reset_vehicle_map_pub.publish(reset_msg)
        rospy.loginfo("Sent RESET signal to perception maps.")

        print("\n=== Parking Data Collector Started ===")
        print(f"Loading Map: {self.config['paths']['map_file']}")
        print(" [SPACE]     : Record current waypoint")
        print(" [BACKSPACE] : Delete last waypoint")
        print(" [ENTER]     : Finish episode & Select Slot ID")
        print(f"[Instruction]: {self.formatter.config['system']['instruction']}")
        print("Waiting for recording...\n")

        # 启动键盘监听线程
        self.running = True
        self.input_thread = threading.Thread(target=self.keyboard_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def keyboard_loop(self):
        def is_data():
            return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            
            while self.running and not rospy.is_shutdown():
                if is_data():
                    c = sys.stdin.read(1)
                    if c == ' ':  # Space
                        self.record_frame()
                    elif c == '\x7f':  # Backspace
                        self.delete_last_frame()
                    elif c == '\n' or c == '\r': # Enter
                        # 恢复终端设置以便进行 input() 输入
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                        self.finish_episode()
                        # 重新设置为 cbreak 模式
                        tty.setcbreak(sys.stdin.fileno())
                else:
                    time.sleep(0.1)
                    
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def odom_cb(self, msg):
        with self.lock:
            # 提取位置
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            
            # 提取Yaw (转为角度制，与Prompt保持一致)
            orientation_q = msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            yaw_deg = yaw * 180.0 / 3.1415926
            
            self.current_odom = {
                "x": x, "y": y, "z": z, "yaw": yaw_deg
            }

    def veh_cb(self, msg):
        with self.lock:
            self.current_vehicles = msg

    def space_cb(self, msg):
        with self.lock:
            self.current_spaces = msg

    def record_frame(self):
        with self.lock:
            if self.current_odom is None:
                print("\r[Warn] No Odom data yet! Cannot record.", end="")
                return
            
            # 记录时刻、自车状态、和格式化后的观测字符串
            # 注意：Obs在这里转换成String，锁定当前这一刻的感知结果
            frame = {
                "timestamp": time.time(),
                "ego": copy.deepcopy(self.current_odom),
                "obs_str": self.formatter.format_observation(self.current_vehicles, self.current_spaces)
            }
        
        # 简单防抖：0.5秒内不重复记录
        if len(self.episode_buffer) > 0:
            last_t = self.episode_buffer[-1]['timestamp']
            if frame['timestamp'] - last_t < self.config['sampling'].get('min_interval', 0.5):
                return

        self.episode_buffer.append(frame)
        print(f" -> Recorded Frame {len(self.episode_buffer)}. Pos: [{frame['ego']['x']:.1f}, {frame['ego']['y']:.1f}]")

    def delete_last_frame(self):
        if len(self.episode_buffer) > 0:
            removed = self.episode_buffer.pop()
            print(f" <- Deleted Frame. Remaining: {len(self.episode_buffer)}")
        else:
            print("[Warn] Buffer is empty.")

    def finish_episode(self):
        if len(self.episode_buffer) < 1:
            print("[Error] Not enough frames (need at least 1). Drive more before pressing Enter.")
            return

        print("\n=== Finishing Episode ===")
        
        # 获取输入，简单的阻塞式交互
        valid_input = False
        slot_id = -1
        while not valid_input:
            try:
                raw_input = input(">>> Please enter target Slot ID (integer): ")
                slot_id = int(raw_input)
                valid_input = True
            except ValueError:
                print("Invalid integer. Try again.")
            except EOFError:
                return # 防止Ctrl+D退出报错

        # 核心逻辑：离线回溯生成数据
        self.process_and_save(slot_id)
        
        # 清空Buffer，重置状态
        self.episode_buffer = []
        print("=== Ready for next episode ===\n")

    def process_and_save(self, slot_id):
        """
        将 [Frame 0, Frame 1, ... Frame N] 转换为对话数据
        Logic: Input(Frame_i) -> Output(Ego_i+1)
        """
        messages = []
        
        # 1. System Prompt
        messages.append({
            "role": "system",
            "content": self.formatter.build_system_prompt()
        })

        # 2. 对话生成
        history_points = [] # 记录 [x, y, yaw]
        total_frames = len(self.episode_buffer)
        
        for i in range(total_frames):
            current_frame = self.episode_buffer[i]
            
            # Step A: 构建 User 提问 (Obs + Status + History)
            user_content = self.formatter.build_user_input(
                current_frame['obs_str'],
                current_frame['ego'],
                history_points
            )
            messages.append({"role": "user", "content": user_content})
            
            # Step B: 更新 History (供下一轮 User Input 使用)
            # 策略：History 包含直到当前时刻之前的路径。
            # 所以在生成 Assistant 回复前，先把当前点加入 History 列表
            history_points.append([
                round(current_frame['ego']['x'], 2),
                round(current_frame['ego']['y'], 2),
                round(current_frame['ego']['yaw'], 2)
            ])
            
            # Step C: 构建 Assistant 回复 (预测下一个点 或 泊车)
            if i < total_frames - 1:
                # 还有下一帧，目标是移动到下一帧
                next_frame = self.episode_buffer[i+1]
                assist_content = self.formatter.build_assistant_output(
                    next_ego=next_frame['ego'],
                    is_parking=False,
                    slot_id=-1
                )
            else:
                # 最后一帧，动作是“开始泊车”
                assist_content = self.formatter.build_assistant_output(
                    next_ego=None, 
                    is_parking=True, 
                    slot_id=slot_id
                )
            
            messages.append({"role": "assistant", "content": assist_content})

        # 3. 写入文件
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('dataset_collector')
        
        # 拼接完整路径
        save_path = os.path.join(pkg_path, self.config['paths']['output_file'])
        
        # 确保目标目录存在，如果不存在则创建
        save_dir = os.path.dirname(save_path)
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            
        rospy.loginfo(f"Saving dataset to: {save_path}")
        self.formatter.save_to_jsonl(messages, save_path)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    import os
    # 自动定位 config 文件 (假设结构为 scripts/ 和 config/)
    current_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(current_dir, '../config/dataset_config.yaml')
    
    node = DataCollectorNode(config_path)
    node.run()