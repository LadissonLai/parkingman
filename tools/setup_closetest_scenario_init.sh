#!/bin/bash
# Spawn scenario vehicles and initialise ego vehicle to task start position.
# Both steps read from the same TASK_DIR.
#
# Usage:
#   ./tools/setup_closetest_scenario_init.sh [TASK_DIR]
#
# Opens a gnome-terminal window with 2 tabs:
#   Tab 0 - spawn_scenario : spawn NPC vehicles from scenario_layout.yaml
#   Tab 1 - init_ego       : teleport ego to gt_trajectory.csv[0]  (sleep 8s first)

HOST="${CARLA_HOST:-127.0.0.1}"
PORT="${CARLA_PORT:-2000}"
TASK_DIR="${1:-/home/u20/codes/LLM_ws/src/LLMParking/carla_close_test/test_datasets/town04/2026-05-16-02-52}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

source /opt/ros/noetic/setup.bash
source "$WS_ROOT/devel/setup.bash"

YAML_PATH="$TASK_DIR/scenario_layout.yaml"

PROGRAM0="roslaunch carla_close_test spawn_scenario.launch \
    carla_host:=$HOST carla_port:=$PORT yaml_path:=$YAML_PATH"
PROGRAM1="sleep 8 && rosrun carla_close_test init_ego.py _task_dir:=$TASK_DIR"

gnome-terminal \
  --window --title="spawn_scenario" --command="bash -c '$PROGRAM0; exec bash'" \
  --tab    --title="init_ego"       --command="bash -c '$PROGRAM1; exec bash'"
