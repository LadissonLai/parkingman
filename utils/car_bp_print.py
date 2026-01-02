import carla
import random

def get_random_vehicle_blueprint():
    # 1. 连接到客户端
    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)
    world = client.get_world()

    # 2. 获取蓝图库
    blueprint_library = world.get_blueprint_library()

    # 3. 获取所有车辆蓝图 (使用 'vehicle.*' 过滤)
    # 这会返回一个列表，包含轿车、卡车、摩托车、自行车等所有车型
    vehicle_blueprints = blueprint_library.filter('vehicle.*')

    # 4. (可选) 进一步过滤，例如只想要汽车（排除摩托车和自行车）
    # 通过判断轮子数量，通常汽车有4个轮子
    car_blueprints = [x for x in vehicle_blueprints if int(x.get_attribute('number_of_wheels')) == 4]

    for i, c in enumerate(car_blueprints, start=1):
        print(f"{i}: {c.id}")

# 执行
get_random_vehicle_blueprint()