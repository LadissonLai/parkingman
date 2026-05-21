#!/bin/bash
# Launch GT odometry, static map builder, and BEV parking-space perception.
# Replaces setup_vps_map_dev_in_fastlio.sh (no FastLIO required).
#
# Opens a gnome-terminal window with 5 tabs:
#   Tab 0 - gt_odom      : GT odometry node  (map->camera_init TF + /Odometry_camera_init)
#   Tab 1 - static_map   : static LiDAR obstacle map  (/local_map in camera_init frame)
#   Tab 2 - bev_ipm      : BEV image stitcher
#   Tab 3 - ps_detection : parking space detector
#   Tab 4 - build_ps_map : parking space map accumulator
#
# Note: setup_closetest_main.sh already starts the /local_map -> /map relay for Hybrid A*.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

source /opt/ros/noetic/setup.bash
source "$WS_ROOT/devel/setup.bash"
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH

PROGRAM0="rosrun carla_close_test carla_gt_odom.py"
PROGRAM1="sleep 3 && rosrun carla_close_test carla_static_map.py"
PROGRAM2="sleep 5 && rosrun perception bev_ipm_hard.py"
PROGRAM3="sleep 3 && roslaunch perception ps_detection.launch world_frame:=camera_init"
PROGRAM4="sleep 3 && rosrun perception build_ps_map.py _local_world_frame:=camera_init"

gnome-terminal \
  --window --title="gt_odom"      --command="bash -c '$PROGRAM0; exec bash'" \
  --tab    --title="static_map"   --command="bash -c '$PROGRAM1; exec bash'" \
  --tab    --title="bev_ipm"      --command="bash -c '$PROGRAM2; exec bash'" \
  --tab    --title="ps_detection" --command="bash -c '$PROGRAM3; exec bash'" \
  --tab    --title="build_ps_map" --command="bash -c '$PROGRAM4; exec bash'"
