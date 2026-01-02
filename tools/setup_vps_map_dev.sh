#!/bin/bash
# launch vacant parking space map
source ../../devel/setup.bash
PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
PROGRAM1="rosrun perception bev_ipm_hard.py"
PROGRAM2="roslaunch perception ps_detection.launch"
PROGRAM3="rosrun perception build_ps_map.py"
gnome-terminal \
  --window --title="sticher bev" --command="bash -c '$PROGRAM1; exec bash'" \
  --tab --title="vps detection" --command="bash -c '$PROGRAM2; exec bash'" \
  --tab --title="build vps map" --command="bash -c '$PROGRAM3; exec bash'"