#!/bin/bash
source ../../devel/setup.bash
PROGRAM1="rosrun dataset_collector collector_node.py"
gnome-terminal \
  --window --title="parking decision data collector" --command="bash -c '$PROGRAM1; exec bash'"