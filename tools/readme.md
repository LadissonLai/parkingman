数据采集的配置和启动流程

### 启动carla
```
./tools/setup_carla_dev.sh
```

### 修改场景配置
修改src/LLMParking/dataset_collector/config/town04_base_layout.yaml文件，设置哪些车辆显示，哪些车辆不显示，以及设置他们的车辆类型。
```
./tools/setup_scenario_dev.sh
```

### 检测空闲车位
```
./tools/setup_vps_map_dev.sh
```

### 启动vla数据采集
```
./tools/setup_dataset_collector_vla.sh
```





