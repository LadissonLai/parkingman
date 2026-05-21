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
    "noon":    80.0,   # 正午
    "evening": 10.0,   # 傍晚/黄昏
    "night":   -90.0   # 深夜
}

# 随机生成时排除的车辆蓝图列表
EXCLUDED_VEHICLE_BLUEPRINTS = [
    # 添加你想要排除的车辆蓝图ID
    "vehicle.mitsubishi.fusorosa",    
    "vehicle.carlamotors.firetruck",
    "vehicle.micro.microlino",
    "vehicle.tesla.cybertruck",
    "vehicle.ford.ambulance",
    "vehicle.mercedes.sprinter",
    "vehicle.nissan.patrol_2021"
    # 你可以继续添加其他不想在随机场景中出现的车辆
]

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

    def get_random_vehicle_blueprint(self):
        """
        获取随机车辆蓝图，参考car_bp_print.py的逻辑
        返回随机选择的4轮车辆蓝图，排除指定的车辆类型
        """
        # 获取所有车辆蓝图
        vehicle_blueprints = self.bp_library.filter('vehicle.*')

        # 进一步过滤，只选择4轮车辆（排除摩托车和自行车）
        car_blueprints = [x for x in vehicle_blueprints if int(x.get_attribute('number_of_wheels')) == 4]

        if not car_blueprints:
            logger.warning("No 4-wheel vehicle blueprints found, falling back to all vehicles")
            car_blueprints = vehicle_blueprints

        # 排除指定的车辆蓝图
        filtered_blueprints = [bp for bp in car_blueprints if bp.id not in EXCLUDED_VEHICLE_BLUEPRINTS]

        if not filtered_blueprints:
            logger.warning(f"All available blueprints are in the excluded list {EXCLUDED_VEHICLE_BLUEPRINTS}, using all available blueprints")
            filtered_blueprints = car_blueprints

        # 随机选择一个
        return random.choice(filtered_blueprints)

    def clean_previous_actors(self):
        """
        Clean up previously spawned vehicles, preserving ego/hero vehicles.
        """
        logger.info("Starting cleanup of previous actors...")
        
        # 强制同步一次，确保获取到最新的 Actor 列表
        # 有时候刚连接上 Server，Actor 列表可能还没同步过来
        try:
            self.world.wait_for_tick(seconds=2.0)
        except RuntimeError:
            logger.warning("wait_for_tick timed out (is the server running?), proceeding...")

        actor_list = self.world.get_actors()
        vehicles = actor_list.filter('vehicle.*')
        
        logger.info(f"Total actors in world: {len(actor_list)}")
        logger.info(f"Found {len(vehicles)} vehicles to check for cleanup.")
        
        batch_destroy = []
        for actor in vehicles:
            role_name = actor.attributes.get('role_name')
            if role_name in ['ego_vehicle', 'hero']:
                logger.info(f"Preserving vehicle {actor.id} with role '{role_name}'")
                continue
            
            batch_destroy.append(carla.command.DestroyActor(actor))

        if batch_destroy:
            try:
                responses = self.client.apply_batch_sync(batch_destroy)
                failed_count = sum(1 for response in responses if response.has_error())
                if failed_count > 0:
                    logger.warning(f"Failed to destroy {failed_count} actors.")
                else:
                    logger.info(f"Successfully destroyed {len(batch_destroy)} vehicles.")
            except Exception as e:
                logger.error(f"Error during batch destruction: {e}")
        else:
            logger.info("No vehicles found to clean up.")

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

    def spawn_vehicles(self, layout_data, task_data=None, random_generation=False, full_layout_generation=False, ratio=0.5, output_config=None):
        """
        layout_data: 包含所有停车位的物理坐标 (Base Config)
        task_data: 包含本次任务需要生成的车辆ID和类型 (Scenario Config) - optional when random_generation=True or full_layout_generation=True
        random_generation: 是否启用随机生成模式
        full_layout_generation: 是否启用完整layout生成模式（生成所有车辆）
        ratio: 随机生成时车辆生成比例 (0.0-1.0)
        output_config: 生成配置文件时输出配置文件的路径
        """
        # 1. 建立基础坐标索引 (ID -> Transform Data)
        # 假设 layout 文件中 key 是 'carla_parking_space_config' 或直接是列表
        layout_list = layout_data.get('carla_parking_space_config', [])
        if not layout_list:
            layout_list = layout_data.get('spawn_points', [])
            
        layout_map = {item['id']: item for item in layout_list if 'id' in item}

        count = 0

        # 用于收集生成的车辆配置（用于输出配置文件）
        generated_config = {"carla_parking_space_config": []} if (random_generation or full_layout_generation) else None

        if full_layout_generation:
            # 完整layout模式：生成所有车辆
            if not layout_list:
                logger.warning("No layout data found for full layout generation")
                return

            logger.info(f"Full layout generation: generating all {len(layout_list)} vehicles")

            # 先创建完整的车辆配置列表（所有车辆spawn=true）
            for base_info in layout_list:
                # 准备蓝图（完整layout模式使用layout中定义的默认蓝图或随机蓝图）
                bp_name = base_info.get('blueprint', None)
                if bp_name and bp_name.lower() != 'random':
                    bp_id = bp_name
                else:
                    # 如果没有指定蓝图或指定为random，使用随机蓝图
                    bp = self.get_random_vehicle_blueprint()
                    bp_id = bp.id
                    
                vehicle_config = {
                    'id': base_info['id'],
                    'blueprint': bp_id,
                    'spawn': base_info["spawn"],
                    'transform': base_info['transform']
                }
                if base_info["spawn"]:      
                    generated_config["carla_parking_space_config"].append(vehicle_config)

            # 遍历所有车辆进行生成
            for base_info in layout_list:
                vh_id = base_info['id']

                # 准备蓝图（完整layout模式使用layout中定义的默认蓝图或随机蓝图）
                bp_name = base_info.get('blueprint', None)
                if bp_name and bp_name.lower() != 'random':
                    bp = self.bp_library.find(bp_name)
                else:
                    # 如果没有指定蓝图或指定为random，使用随机蓝图
                    bp = self.get_random_vehicle_blueprint()
                    bp_name = bp.id

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
                    if base_info["spawn"]:
                        actor = self.world.spawn_actor(bp, transform)
                        actor.set_autopilot(False)
                        count += 1
                        logger.info(f"Spawned {bp.id} at {vh_id}")

                except RuntimeError as e:
                    logger.error(f"Collision/Error at {vh_id}: {e}")

            # 输出生成的配置文件（仅在完整layout模式下）
            if output_config:
                try:
                    with open(output_config, 'w') as f:
                        yaml.dump(generated_config, f, default_flow_style=False, sort_keys=False)
                    logger.info(f"Generated config saved to: {output_config}")
                except Exception as e:
                    logger.error(f"Failed to save config file: {e}")

        elif random_generation:
            # 随机模式：从layout中随机选择车辆
            if not layout_list:
                logger.warning("No layout data found for random generation")
                return

            # 计算需要生成的车辆数量
            total_available = len(layout_list)
            num_to_spawn = max(1, int(total_available * ratio))  # 至少生成1辆

            # 随机选择车辆ID
            selected_items = random.sample(layout_list, min(num_to_spawn, total_available))

            logger.info(f"Random generation: selected {len(selected_items)} out of {total_available} vehicles (ratio: {ratio})")

            # 用于收集生成的车辆配置（用于输出配置文件）
            generated_config = {"carla_parking_space_config": []}

            # 创建车辆ID到选中状态的映射
            selected_ids = {item['id'] for item in selected_items}

            # 先创建完整的车辆配置列表（根据是否选中设置spawn状态）
            for base_info in layout_list:
                vh_id = base_info['id']
                is_selected = vh_id in selected_ids

                # 准备蓝图（随机模式对所有车辆都分配随机蓝图）
                bp = self.get_random_vehicle_blueprint()
                bp_name = bp.id

                vehicle_config = {
                    'id': vh_id,
                    'blueprint': bp_name,
                    'spawn': is_selected,  # 只生成选中的车辆
                    'transform': base_info['transform']
                }
                generated_config["carla_parking_space_config"].append(vehicle_config)

            # 遍历选中的车辆进行生成
            for base_info in selected_items:
                vh_id = base_info['id']  # 随机模式下直接从base_info获取ID

                # 准备蓝图（随机模式使用随机蓝图）
                bp = self.get_random_vehicle_blueprint()
                bp_name = bp.id

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

            # 输出生成的配置文件（仅在随机模式下）
            if output_config and random_generation:
                try:
                    with open(output_config, 'w') as f:
                        yaml.dump(generated_config, f, default_flow_style=False, sort_keys=False)
                    logger.info(f"Generated config saved to: {output_config}")
                except Exception as e:
                    logger.error(f"Failed to save config file: {e}")
        else:
            # 非随机模式：使用原有逻辑
            # 2. 获取任务列表
            # 假设任务文件中 key 是 'scenario_layout'
            task_list = task_data.get('scenario_layout', [])

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
    parser = argparse.ArgumentParser(description="Carla Scenario Manager with Safe Cleanup")
    parser.add_argument('--host', default='127.0.0.1', help='Carla Host IP')
    parser.add_argument('--port', type=int, default=2000, help='Carla Port')
    
    # 两个配置文件参数
    parser.add_argument('--layout', required=True, help='Base layout YAML (Coordinates)')
    parser.add_argument('--scenario', help='Task scenario YAML (Weather & Selection) - optional when using random generation')

    # 生成模式参数
    parser.add_argument('--random', action='store_true', help='Enable random vehicle generation from layout config')
    parser.add_argument('--full-layout', action='store_true', help='Generate all vehicles from layout config (no scenario file needed)')
    parser.add_argument('--random-ratio', type=float, default=0.5, help='Ratio of vehicles to spawn randomly (0.0-1.0, default: 0.5)')
    parser.add_argument('--output-config', help='Output path for generated config file (used with --random or --full-layout)')
    
    args = parser.parse_args()

    # 验证随机比例参数范围
    if not (0.0 <= args.random_ratio <= 1.0):
        logger.error("Random ratio must be between 0.0 and 1.0")
        sys.exit(1)

    # 加载文件
    layout_data = load_yaml(args.layout)

    scenario_data = None
    if args.random:
        # 随机模式：不需要scenario文件
        pass
    elif args.full_layout:
        # 完整layout模式：不需要scenario文件
        pass
    else:
        # 传统模式：需要scenario文件
        if not args.scenario:
            logger.error("Scenario file is required when not using --random or --full-layout")
            sys.exit(1)
        scenario_data = load_yaml(args.scenario)

    # 初始化并运行
    manager = ScenarioManager(args.host, args.port)

    # 0. Clean up previous actors (Added Safe Logic)
    manager.clean_previous_actors()

    # 1. 设置天气
    if args.random or args.full_layout:
        mode_name = "Random generation" if args.random else "Full layout generation"
        logger.info(f"{mode_name} enabled - using default weather (clear/morning)")
        # 随机模式和完整layout模式下使用默认晴天早上天气
        manager.set_weather({'type': 'clear', 'time': 'noon'})
    elif 'weather' in scenario_data:
        manager.set_weather(scenario_data['weather'])
    else:
        logger.warning("No weather config found in scenario yaml.")

    # 2. 生成车辆
    if args.random:
        manager.spawn_vehicles(layout_data, None, random_generation=True, ratio=args.random_ratio, output_config=args.output_config)
    elif args.full_layout:
        manager.spawn_vehicles(layout_data, None, full_layout_generation=True, ratio=1.0, output_config=args.output_config)
    else:
        manager.spawn_vehicles(layout_data, scenario_data, random_generation=False, ratio=args.random_ratio)

if __name__ == '__main__':
    main()

