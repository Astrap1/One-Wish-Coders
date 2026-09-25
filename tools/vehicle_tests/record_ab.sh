#!/usr/bin/env bash
# Record the Checkpoint 2 mud A/B clip straight from Gazebo's cameras.
#
#   tools/vehicle_tests/record_ab.sh [render_engine]      # ogre2 on a GPU machine (default), ogre for software GL
#
# 1. generates mud_ab_test with two recording cameras (wide + close-up)
# 2. runs it headless for 22 s of sim time, with tools/gz_sink subscribed to
#    both cameras (Gazebo only renders a camera that has a subscriber)
# 3. overlays labels / live telemetry from the AirCushion logs and encodes
#    results/mud_ab_test/checkpoint2_mud_ab.mp4 (tools/make_ab_video.py)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
WS="$ROOT/autonomous_tidal_vehicle_ws"
ENGINE="${1:-ogre2}"
WORLDS=/tmp/hover_clip_worlds
OUT="$ROOT/results/mud_ab_test"
export GZ_SIM_RESOURCE_PATH="${MODELS_DIR:-$WS/src/tidal_vehicle_description/models}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PLUGIN_DIR:-$WS/install/tidal_vehicle_simulation/lib}:$WS/build/tidal_vehicle_simulation"

python3 "$WS/src/tidal_vehicle_simulation/scripts/gen_test_worlds.py" --no-render-sensors \
    --clip-cameras --render-engine "$ENGINE" --out "$WORLDS" > /dev/null
rm -rf /tmp/gz_clip /tmp/hover_*.csv
mkdir -p "$OUT"
SINK="$ROOT/tools/vehicle_tests/gz_sink/build/gz_sink"
"$SINK" /world/mud_ab_test/model/cam_wide/link/link/sensor/cam/image 2> /tmp/sink_wide.log & S1=$!
"$SINK" /world/mud_ab_test/model/cam_close/link/link/sensor/cam/image 2> /tmp/sink_close.log & S2=$!
gz sim -s -r --iterations 22000 "$WORLDS/mud_ab_test.sdf" -v 2 > "$OUT/gz_record.log" 2>&1 || true
kill -INT $S1 $S2 2>/dev/null || true; sleep 1
cat /tmp/sink_wide.log /tmp/sink_close.log
cp /tmp/hover_*.csv "$OUT/"
python3 "$ROOT/tools/vehicle_tests/make_ab_video.py" --frames /tmp/gz_clip --logs "$OUT" --out "$OUT/checkpoint2_mud_ab.mp4"
