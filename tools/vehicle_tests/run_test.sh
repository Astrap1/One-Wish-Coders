#!/usr/bin/env bash
# Run one scripted test world headless and collect the AirCushion CSV logs.
#
#   tools/vehicle_tests/run_test.sh <world_name> <sim_seconds> [results_dir]
#   e.g. tools/vehicle_tests/run_test.sh mud_ab_test 22
#
# Needs gz-sim 8 (Harmonic) on PATH and the workspace built with colcon
# (install/tidal_vehicle_simulation/lib/libtidal_vehicle_plugins.so), or the
# plugin built elsewhere and given with PLUGIN_DIR=...  Results go to results/
# (git-ignored).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
WS="$ROOT/autonomous_tidal_vehicle_ws"
WORLD="$1"; SECS="$2"; OUT="${3:-$ROOT/results/$WORLD}"
WORLD_DIR="${WORLD_DIR:-$WS/src/tidal_vehicle_simulation/worlds/vehicle_tests}"
MODELS="${MODELS_DIR:-$WS/src/tidal_vehicle_description/models}"
PLUGIN_DIR="${PLUGIN_DIR:-$WS/install/tidal_vehicle_simulation/lib}"

export GZ_SIM_RESOURCE_PATH="$MODELS${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PLUGIN_DIR:$WS/build/tidal_vehicle_simulation${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"

rm -f /tmp/hover_*.csv
mkdir -p "$OUT"
ITER=$(python3 -c "print(int(float('$SECS') * 1000))")   # 1 ms steps
echo "[run_test] $WORLD for ${SECS}s sim time ($ITER iterations)"
# -s server only, -r run, -z max update rate (run faster than real time when possible)
gz sim -s -r -z 100000 --iterations "$ITER" "$WORLD_DIR/$WORLD.sdf" -v 3 > "$OUT/gz.log" 2>&1 || true
cp /tmp/hover_*.csv "$OUT/" 2>/dev/null || echo "[run_test] WARNING: no CSV logs produced"
grep -E "AirCushion\]|TerrainZones\]|ScriptedCommands\]|Error|error" "$OUT/gz.log" | head -40 || true
ls -la "$OUT"
