import carla
import yaml
import argparse
import random
import logging
import sys
import os

# 配置日志
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)

# --- 1. 预定义天气与时间字典 ---

# 天气类型预设 (云量, 降雨, 积水, 雾)
WEATHER_PRESETS = {
    "clear":  {"cloudiness": 0.0,  "precipitation": 0.0,  "wetness": 0.0,  "fog_density": 0.0},
    "cloudy": {"cloudiness": 80.0, "precipitation": 0.0,  "wetness": 0.0,  "fog_density": 0.0},
    "rain":   {"cloudiness": 90.0, "precipitation": 60.0, "wetness": 70.0, "fog_density": 10.0},
    "fog":    {"cloudiness": 50.0, "precipitation": 0.0,  "wetness": 10.0, "fog_density": 80.0}
}

# 时间段预设 (太阳高度角)
TIME_PRESETS = {
    "morning": 30.0,   # 早上
    "noon":    90.0,   # 正午
    "evening": 10.0,   # 傍晚/黄昏
    "night":   -90.0   # 深夜
}

class ScenarioManager:
    def __init__(self, host, port, timeout=10.0):
        try:
            self.client = carla.Client(host, port)
            self.client.set_timeout(timeout)
            self.world = self.client.get_world()
            self.bp_library = self.world.get_blueprint_library()
            logger.info("Connected to CARLA Server.")
        except Exception as e:
            logger.error(f"Failed to connect to CARLA: {e}")
            sys.exit(1)

    def set_weather(self, weather_config):
        """
        根据 YAML 配置设置天气
        支持格式:
        weather:
          - type: "rain"
          - time: "morning"
        """
        if not weather_config:
            return

        # 解析配置 (兼容列表或字典格式)
        w_conf = {}
        if isinstance(weather_config, list):
            for item in weather_config:
                w_conf.update(item)
        elif isinstance(weather_config, dict):
            w_conf = weather_config

        # 获取配置键值，默认为 晴天 + 正午
        w_type_key = w_conf.get('type', 'clear').lower()
        w_time_key = w_conf.get('time', 'noon').lower() # 兼容 time 或 time_of_day
        if 'time_of_day' in w_conf: w_time_key = w_conf['time_of_day'].lower()

        # 获取参数
        params = WEATHER_PRESETS.get(w_type_key, WEATHER_PRESETS['clear'])
        sun_angle = TIME_PRESETS.get(w_time_key, TIME_PRESETS['noon'])

        # 应用设置
        weather = carla.WeatherParameters(
            cloudiness=params['cloudiness'],
            precipitation=params['precipitation'],
            precipitation_deposits=params['wetness'], # 积水通常和湿度挂钩
            wind_intensity=20.0,
            sun_altitude_angle=sun_angle,
            fog_density=params['fog_density'],
            wetness=params['wetness']
        )

        self.world.set_weather(weather)
        logger.info(f"Weather set to: [{w_type_key}] at [{w_time_key}]")

    def spawn_vehicles(self, layout_data, task_data):
        """
        layout_data: 包含所有停车位的物理坐标 (Base Config)
        task_data: 包含本次任务需要生成的车辆ID和类型 (Scenario Config)
        """
        # 1. 建立基础坐标索引 (ID -> Transform Data)
        # 假设 layout 文件中 key 是 'carla_parking_space_config' 或直接是列表
        layout_list = layout_data.get('carla_parking_space_config', [])
        if not layout_list:
            layout_list = layout_data.get('spawn_points', [])
            
        layout_map = {item['id']: item for item in layout_list if 'id' in item}

        # 2. 获取任务列表
        # 假设任务文件中 key 是 'scenario_layout'
        task_list = task_data.get('scenario_layout', [])
        
        count = 0
        
        # 3. 遍历任务列表进行生成
        for task_item in task_list:
            vh_id = task_item.get('id')
            should_spawn = task_item.get('spawn', False)

            # 如果任务配置说不生成，或者根本没在任务列表里(虽然循环进不来)，直接跳过
            if not should_spawn:
                continue
            
            # 检查是否有坐标信息
            if vh_id not in layout_map:
                logger.warning(f"ID {vh_id} requested in scenario but NOT found in layout config.")
                continue

            base_info = layout_map[vh_id]
            
            # 准备蓝图
            bp_name = task_item.get('blueprint', 'default')
            if bp_name == 'default':
                # 如果任务没指定具体车型，尝试用layout里的默认值，或者Fallback到Model3
                bp_name = base_info.get('blueprint', 'vehicle.tesla.model3')
            
            bp = None
            if bp_name.lower() == 'random':
                bp = random.choice(self.bp_library.filter('vehicle.*'))
            else:
                bp = self.bp_library.find(bp_name)
            
            if not bp:
                logger.error(f"Blueprint '{bp_name}' not found for ID {vh_id}")
                continue

            # 准备坐标
            loc = base_info['transform']['location']
            rot = base_info['transform']['rotation']
            transform = carla.Transform(
                carla.Location(x=loc['x'], y=loc['y'], z=loc['z'] + 0.2), # z稍微抬高一点防止卡地
                carla.Rotation(roll=rot['roll'], pitch=rot['pitch'], yaw=rot['yaw'])
            )

            # 生成
            try:
                actor = self.world.spawn_actor(bp, transform)
                actor.set_autopilot(False)
                count += 1
                logger.info(f"Spawned {bp.id} at {vh_id}")
            except RuntimeError as e:
                logger.error(f"Collision/Error at {vh_id}: {e}")

        logger.info(f"Scenario setup complete. Total vehicles spawned: {count}")

def load_yaml(path):
    if not os.path.exists(path):
        logger.error(f"Config file not found: {path}")
        sys.exit(1)
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def main():
    parser = argparse.ArgumentParser(description="Carla Scenario Manager")
    parser.add_argument('--host', default='127.0.0.1', help='Carla Host IP')
    parser.add_argument('--port', type=int, default=2000, help='Carla Port')
    
    # 两个配置文件参数
    parser.add_argument('--layout', required=True, help='Base layout YAML (Coordinates)')
    parser.add_argument('--scenario', required=True, help='Task scenario YAML (Weather & Selection)')
    
    args = parser.parse_args()

    # 加载文件
    layout_data = load_yaml(args.layout)
    scenario_data = load_yaml(args.scenario)

    # 初始化并运行
    manager = ScenarioManager(args.host, args.port)
    
    # 1. 设置天气
    if 'weather' in scenario_data:
        manager.set_weather(scenario_data['weather'])
    else:
        logger.warning("No weather config found in scenario yaml.")

    # 2. 生成车辆
    manager.spawn_vehicles(layout_data, scenario_data)

if __name__ == '__main__':
    main()