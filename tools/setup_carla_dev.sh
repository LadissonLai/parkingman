#!/bin/bash

export CARLA_ROOT=/home/u20/codes/carla/CARLA_0.9.14
export CARLA_ROS_BRIDGE_WS=../../
export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla:$CARLA_ROOT/PythonAPI:$PYTHONPATH

# launch carla server and ros bridge
PROGRAM1="cd $CARLA_ROOT && pwd && ./CarlaUE4.sh"
PROGRAM2="sleep 10 && cd $CARLA_ROS_BRIDGE_WS && source devel/setup.bash && roslaunch carla_ad_demo parking.launch"
PROGRAM3="python3 utils/pub_park_map_frame.py"
gnome-terminal \
  --window --title="carla_server" --command="bash -c '$PROGRAM1; exec bash'" \
  --tab --title="carla ros bridge" --command="bash -c '$PROGRAM2; exec bash'" \
  --tab --title="pub parking start frame id" --command="bash -c '$PROGRAM3; exec bash'" \


