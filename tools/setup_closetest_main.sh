#!/bin/bash
# Run the closed-loop VLM parking navigation test for a single task.
# Opens a gnome-terminal window with 3 tabs:
#   Tab 0 - relay        : /local_map -> /map relay for Hybrid A*
#   Tab 1 - hybrid_a_star: Hybrid A* planner
#   Tab 2 - closetest    : main test node (blocking until task completes)
#
# Usage:
#   ./tools/setup_closetest_main.sh [TASK_DIR] [VLM_SERVER]
#
# Example (single task):
#   ./tools/setup_closetest_main.sh \
#     /home/u20/codes/LLM_ws/src/LLMParking/carla_close_test/test_datasets/town04/2026-05-15-10-49 \
#     http://localhost:9999

TASK_DIR="${1:-/home/u20/codes/LLM_ws/src/LLMParking/carla_close_test/test_datasets/town04/2026-05-16-02-52}"
VLM_SERVER="${2:-http://localhost:9999}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

source /opt/ros/noetic/setup.bash
source "$WS_ROOT/devel/setup.bash"

echo "[closetest_main] Task:       $TASK_DIR"
echo "[closetest_main] VLM server: $VLM_SERVER"

PROGRAM0="source /opt/ros/noetic/setup.bash && source $WS_ROOT/devel/setup.bash && rosrun topic_tools relay /local_map /map"
PROGRAM1="source /opt/ros/noetic/setup.bash && source $WS_ROOT/devel/setup.bash && roslaunch hybrid_a_star run_hybrid_a_star.launch"
PROGRAM2="source /opt/ros/noetic/setup.bash && source $WS_ROOT/devel/setup.bash && sleep 3 && rosrun carla_close_test closetest_main.py _task_dir:=$TASK_DIR _vlm_server:=$VLM_SERVER"

gnome-terminal \
  --window --title="relay"         --command="bash -c '$PROGRAM0; exec bash'" \
  --tab    --title="hybrid_a_star" --command="bash -c '$PROGRAM1; exec bash'" \
  --tab    --title="closetest"     --command="bash -c '$PROGRAM2; exec bash'"
