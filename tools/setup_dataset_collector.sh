#!/bin/bash

./tools/setup_carla_dev.sh
sleep 15
./tools/setup_scenario_dev.sh
sleep 1
./tools/setup_vehicle_map_dev.sh
sleep 1
./tools/setup_vps_map_dev.sh
sleep 3

source ../../devel/setup.bash
PROGRAM1="rosrun dataset_collector collector_node.py"
gnome-terminal \
  --window --title="parking decision data collector" --command="bash -c '$PROGRAM1; exec bash'"