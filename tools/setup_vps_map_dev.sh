#!/bin/bash
# launch vacant parking space map
source ../../devel/setup.bash
PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
PROGRAM1="rosrun perception bev_ipm_hard.py"
PROGRAM2="roslaunch perception vps_detection.launch"
PROGRAM3="rosrun perception build_ps_map.py"
gnome-terminal \
  --window --title="vps map" --command="bash -c '$PROGRAM1; exec bash'" \
  --tab --command="bash -c '$PROGRAM2; exec bash'" \
  --tab --command="bash -c '$PROGRAM3; exec bash'"