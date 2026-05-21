#!/bin/bash
# Batch closed-loop VLM parking navigation test.
# Loops over all task sub-folders inside TOWN_DIR, running the single-task
# test for each. Between tasks, restarts FastLIO + BEV detection so that the
# camera_init frame resets cleanly for every task.
#
# Prerequisites (already running before calling this script):
#   1. CARLA simulator + carla-ros-bridge  (setup_carla_dev.sh)
#
# Usage:
#   ./tools/setup_closetest_batch.sh [TOWN_DIR] [VLM_SERVER]
#
# Example:
#   ./tools/setup_closetest_batch.sh \
#     /home/u20/codes/LLM_ws/src/LLMParking/carla_close_test/test_datasets/town04 \
#     http://localhost:9999

TOWN_DIR="${1:-/home/u20/codes/LLM_ws/src/LLMParking/carla_close_test/test_datasets/town04}"
VLM_SERVER="${2:-http://localhost:9999}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
PKG_DIR="$SCRIPT_DIR/../carla_close_test"

source /opt/ros/noetic/setup.bash
source "$WS_ROOT/devel/setup.bash"

echo "======================================================="
echo " Batch Close-Loop Test"
echo " Town dir:   $TOWN_DIR"
echo " VLM server: $VLM_SERVER"
echo "======================================================="

TASK_COUNT=0
PASS_COUNT=0
FAIL_COUNT=0

for TASK_DIR in "$TOWN_DIR"/*/; do
    # Skip folders without a gt_trajectory.csv (incomplete datasets)
    [ -f "$TASK_DIR/gt_trajectory.csv" ] || continue

    TASK_NAME="$(basename "$TASK_DIR")"
    TASK_COUNT=$((TASK_COUNT + 1))
    echo ""
    echo "-------------------------------------------------------"
    echo " Task $TASK_COUNT: $TASK_NAME"
    echo "-------------------------------------------------------"

    # ── 1. Spawn scenario vehicles ─────────────────────────────────────────
    YAML="$TASK_DIR/scenario_layout.yaml"
    if [ -f "$YAML" ]; then
        echo "[batch] Spawning scenario from $YAML ..."
        roslaunch carla_close_test spawn_scenario.launch yaml_path:="$YAML" &
        SCENARIO_PID=$!
        sleep 8
    else
        echo "[batch] No scenario_layout.yaml found — skipping spawn."
        SCENARIO_PID=""
    fi

    # ── 2. Place ego at task start position FIRST ───────────────────────────
    # Must init ego before FastLIO starts: if ego teleports after FastLIO is
    # already running, the sudden LiDAR jump corrupts FastLIO's odometry.
    echo "[batch] Initialising ego position..."
    bash "$SCRIPT_DIR/setup_closetest_init_ego.sh" "$TASK_DIR"
    sleep 3

    # ── 3. (Re)start FastLIO + BEV detection (fresh camera_init per task) ──
    echo "[batch] Starting FastLIO + BEV detection..."
    PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH \
    bash "$SCRIPT_DIR/setup_vps_map_dev_in_fastlio.sh" &
    VPS_PID=$!
    sleep 12   # wait for FastLIO to initialise and publish /Odometry_fastlio

    # ── 4. Run single-task test (blocks until closetest_main.py exits) ──────
    echo "[batch] Running main test..."
    bash "$SCRIPT_DIR/setup_closetest_main.sh" "$TASK_DIR" "$VLM_SERVER"
    TASK_EXIT=$?

    if [ $TASK_EXIT -eq 0 ]; then
        PASS_COUNT=$((PASS_COUNT + 1))
        echo "[batch] Task $TASK_NAME COMPLETED (exit 0)."
    else
        FAIL_COUNT=$((FAIL_COUNT + 1))
        echo "[batch] Task $TASK_NAME FAILED (exit $TASK_EXIT)."
    fi

    # ── 5. Kill FastLIO / BEV / scenario processes before next task ─────────
    echo "[batch] Cleaning up processes for task $TASK_NAME ..."
    [ -n "$VPS_PID" ]      && kill $VPS_PID      2>/dev/null
    [ -n "$SCENARIO_PID" ] && kill $SCENARIO_PID 2>/dev/null
    # Kill any residual FastLIO / perception nodes by name
    pkill -f "fastlio_mapping"   2>/dev/null || true
    pkill -f "pcl_to_gridmap"    2>/dev/null || true
    pkill -f "bev_ipm_hard"      2>/dev/null || true
    pkill -f "ps_detection"      2>/dev/null || true
    pkill -f "build_ps_map"      2>/dev/null || true
    pkill -f "map_to_init_tf"    2>/dev/null || true
    sleep 3
done

# ── Aggregate all results ──────────────────────────────────────────────────────
echo ""
echo "======================================================="
echo " All $TASK_COUNT tasks complete."
echo " Passed: $PASS_COUNT  |  Failed/Aborted: $FAIL_COUNT"
echo "======================================================="
echo ""
echo "[batch] Aggregating results..."
python3 "$PKG_DIR/scripts/aggregate_results.py" \
    "$PKG_DIR/performance_result"

echo "[batch] Batch test finished."
