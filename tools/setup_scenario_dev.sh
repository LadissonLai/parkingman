#!/bin/bash

HOST="127.0.0.1"
PORT=2000
LAYOUT_YAML="dataset_collector/config/town04_parkinglot_layout.yaml"
SCENARIO_YAML="dataset_collector/config/low_density_clear_noon.yaml"
RANDOM_MODE="--random"
FULL_MODE="--full-layout"

# 1. random
PROGRAM="python3 ./utils/scenario_generation_cleanup.py \
    --host $HOST \
    --port $PORT \
    --layout $LAYOUT_YAML \
    --scenario $SCENARIO_YAML \
    --random \
    --random-ratio 0.8 \
    --output-config random_scenario.yaml"

# 2. full mode
# PROGRAM="python3 ./utils/scenario_generation_cleanup.py \
#     --host $HOST \
#     --port $PORT \
#     --layout $LAYOUT_YAML \
#     --scenario $SCENARIO_YAML \
#     $FULL_MODE \
#     --output-config full_scenario.yaml"

# 3. task scenario mode
# PROGRAM="python3 ./utils/scenario_generation_cleanup.py \
#     --host $HOST \
#     --port $PORT \
#     --layout $LAYOUT_YAML \
#     --scenario $SCENARIO_YAML 
#     --output-config random_scenario.yaml"

gnome-terminal \
  --window --title="scenario_generation" --command="bash -c '$PROGRAM; exec bash'"