#!/bin/bash
source ../../devel/setup.bash
PROGRAM1="rosrun dataset_collector vla_dataset_collector_node.py"
gnome-terminal \
  --window --title="vla data collector" --command="bash -c '$PROGRAM1; exec bash'"