#!/bin/bash
source ../../devel/setup.bash
PROGRAM1="roslaunch perception global_vehicle_desc_map.launch"
gnome-terminal \
  --window --title="global vehicle desc map" --command="bash -c '$PROGRAM1; exec bash'"