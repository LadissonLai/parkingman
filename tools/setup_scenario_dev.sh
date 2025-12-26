#!/bin/bash

HOST="127.0.0.1"
PORT=2000
LAYOUT_YAML="./benchmark/scenarios/config/town04_parkinglot_layout.yaml"
SCENARIO_YAML="./benchmark/scenarios/config/low_density_clear_noon.yaml"

PROGRAM="python3 ./utils/scenario_generation.py \
    --host $HOST \
    --port $PORT \
    --layout $LAYOUT_YAML \
    --scenario $SCENARIO_YAML"

gnome-terminal \
  --window --title="scenario_generation" --command="bash -c '$PROGRAM; exec bash'"