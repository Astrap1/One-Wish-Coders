#!/usr/bin/env bash
# Version 3 collision check: the scripted v3_collision_test world drives the
# vehicle bow-first into a wall; the ROS collision monitor must report it.
#
#   tools/vehicle_tests/run_collision_test.sh [results_dir]
#
# Needs ROS 2 Jazzy + Gazebo Harmonic and the workspace built with colcon
# (source autonomous_tidal_vehicle_ws/install/setup.bash first). Runs Gazebo
# headless with only the bridge and the collision monitor (no autonomy), so the
# scripted commands are the only ones the vehicle receives.
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
WS="$ROOT/autonomous_tidal_vehicle_ws"
OUT="${1:-$ROOT/results/v3_collision_test}"
mkdir -p "$OUT"
WORLDS=$(mktemp -d)
python3 "$WS/src/tidal_vehicle_simulation/scripts/gen_test_worlds_v3.py" --no-render-sensors \
  --out "$WORLDS" > /dev/null
export GZ_SIM_RESOURCE_PATH="$WS/src/tidal_vehicle_description/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$WS/install/tidal_vehicle_simulation/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
BRIDGE="$OUT/bridge.yaml"
sed "s/WORLD/v3_collision_test/g" "$WS/src/tidal_vehicle_bringup/config/ros_gz_bridge_v3.yaml" > "$BRIDGE"

setsid bash -c "gz sim -s -r '$WORLDS/v3_collision_test.sdf' > '$OUT/gz.log' 2>&1" &
GZ=$!
setsid ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$BRIDGE" \
  -p use_sim_time:=true > "$OUT/bridge.log" 2>&1 &
BR=$!
setsid ros2 run tidal_vehicle_simulation collision_monitor_node.py --ros-args \
  -p use_sim_time:=true > "$OUT/monitor.log" 2>&1 &
MON=$!
timeout 90 ros2 topic echo --once /vehicle/collision tidal_vehicle_interfaces/msg/Collision \
  > "$OUT/collision.txt" 2>&1
STATUS=$?
kill -INT -- -$MON -$BR -$GZ 2>/dev/null; sleep 3
kill -KILL -- -$MON -$BR -$GZ 2>/dev/null
rm -rf "$WORLDS"
if [ $STATUS -eq 0 ] && grep -q "side: front" "$OUT/collision.txt"; then
  echo "[collision_test] PASS: front hit reported"
else
  echo "[collision_test] FAIL (echo status $STATUS)"
fi
cat "$OUT/collision.txt"
grep -h "COLLISION" "$OUT/monitor.log" | head -5
