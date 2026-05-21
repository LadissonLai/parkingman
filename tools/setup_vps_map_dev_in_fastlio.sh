#!/bin/bash
# launch vacant parking space map
source ../../devel/setup.bash
PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
PROGRAM0="roslaunch fastlio2_carla carla_fastlio2.launch"
PROGRAM1="sleep 5 && rosrun perception bev_ipm_hard.py"
PROGRAM2="sleep 3 && roslaunch perception ps_detection.launch world_frame:=camera_init"
PROGRAM3="sleep 3 && rosrun perception build_ps_map.py _local_world_frame:=camera_init"
gnome-terminal \
  --window --title="sticher bev" --command="bash -c '$PROGRAM0; exec bash'" \
  --tab --title="vps detection" --command="bash -c '$PROGRAM1; exec bash'" \
  --tab --title="vps detection" --command="bash -c '$PROGRAM2; exec bash'" \
  --tab --title="build vps map" --command="bash -c '$PROGRAM3; exec bash'"