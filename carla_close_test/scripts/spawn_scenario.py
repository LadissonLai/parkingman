#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import carla
import yaml
import argparse
import sys
import logging

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)

def spawn_from_yaml(host, port, yaml_path):
    try:
        client = carla.Client(host, port)
        client.set_timeout(10.0)
        world = client.get_world()
        bp_library = world.get_blueprint_library()
        logger.info("Connected to CARLA Server.")
    except Exception as e:
        logger.error(f"Failed to connect to CARLA: {e}")
        sys.exit(1)

    # 1. 清理现有场景中的动态车辆 (保留 ego_vehicle / hero)
    logger.info("Cleaning up previous actors...")
    try:
        world.wait_for_tick(seconds=2.0)
    except RuntimeError:
        pass

    actor_list = world.get_actors()
    vehicles = actor_list.filter('vehicle.*')
    
    batch_destroy = []
    for actor in vehicles:
        role_name = actor.attributes.get('role_name')
        if role_name not in ['ego_vehicle', 'hero']:
            batch_destroy.append(carla.command.DestroyActor(actor))

    if batch_destroy:
        client.apply_batch(batch_destroy)
        logger.info(f"Destroyed {len(batch_destroy)} previous vehicles.")
    else:
        logger.info("No previous vehicles to clear.")

    # 2. 读取 YAML
    try:
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
    except Exception as e:
        logger.error(f"Failed to load yaml: {e}")
        sys.exit(1)

    # 3. 设置天气
    weather_conf = data.get('weather', [{'type': 'clear'}, {'time': 'noon'}])
    weather_dict = {}
    for item in weather_conf:
        weather_dict.update(item)
    
    w_type = weather_dict.get('type', 'clear')
    weather_params = carla.WeatherParameters.ClearNoon
    if w_type == 'clear':
        weather_params = carla.WeatherParameters.ClearNoon
    elif w_type == 'rain':
        weather_params = carla.WeatherParameters.MidRainyNoon
    elif w_type == 'cloudy':
        weather_params = carla.WeatherParameters.CloudyNoon
        
    world.set_weather(weather_params)
    logger.info(f"Weather updated to {w_type}.")

    # 4. 根据 YAML 中的参数生成车辆
    parking_config = data.get('carla_parking_space_config', [])
    count = 0
    for cfg in parking_config:
        if not cfg.get('spawn', False):
            continue
        
        bp_name = cfg.get('blueprint', 'vehicle.tesla.model3')
        bp = bp_library.find(bp_name)
        if not bp:
            logger.warning(f"Blueprint {bp_name} not found.")
            continue
            
        loc = cfg['transform']['location']
        rot = cfg['transform']['rotation']
        
        transform = carla.Transform(
            carla.Location(x=loc['x'], y=loc['y'], z=loc['z'] + 0.2), # 稍微抬高防止卡在地面
            carla.Rotation(roll=rot['roll'], pitch=rot['pitch'], yaw=rot['yaw'])
        )
        
        try:
            actor = world.try_spawn_actor(bp, transform)
            if actor:
                actor.set_autopilot(False)
                count += 1
        except Exception as e:
            logger.error(f"Failed to spawn {bp_name} at id {cfg.get('id')}: {e}")

    logger.info(f"Spawned {count} vehicles successfully from {yaml_path}.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--yaml', required=True, help='Path to scenario_layout.yaml')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    
    # 兼容 ROS launch 会传入自带的额外参数
    args, unknown = parser.parse_known_args()
    
    spawn_from_yaml(args.host, args.port, args.yaml)
