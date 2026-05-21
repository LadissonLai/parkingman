#!/bin/bash

TOWN_NAME=${1:-"town04"}

PROGRAM="source ../../devel/setup.bash && rosrun carla_close_test record_trajectory.py _town_name:=$TOWN_NAME"

gnome-terminal \
  --window --title="record_trajectory - $TOWN_NAME" --command="bash -c '$PROGRAM; exec bash'"
