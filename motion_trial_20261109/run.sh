#!/usr/bin/env bash
# Run with locally validated hardware evidence and the manufacturer's ROS environment.
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
: "${APRILTAG_CONTAINER:?Set the container name}"
: "${APRILTAG_CONTAINER_ROOT:?Set the repository mount inside the container}"
: "${APRILTAG_ROS_ENV:?Set the ROS setup script inside the container}"
: "${APRILTAG_CAMERA_SERIAL:?Set the chest camera USB serial}"
KIND="${1:?return or short}"
MODE="${2:?--check or --execute}"
SESSION="${3:?unique session name}"
case "$KIND" in
 return) CONTROLLER=return_to_origin.py; : "${APRILTAG_CAMERA_DEVICE:?Set the chest V4L2 device}";;
 short) CONTROLLER=short_correction.py;;
 *) exit 2;;
esac
[[ "$MODE" == --check || "$MODE" == --execute ]]
[[ "$SESSION" =~ ^[A-Za-z0-9_-]+$ ]]
PROFILE="config/local/profile_$SESSION.json"
settings="$(python3 - "$ROOT" "$PROFILE" "$SESSION" "$MODE" "$KIND" <<'PY'
import hashlib
import json
from pathlib import Path
import shutil
import sys

root, profile_path, session, mode, kind = sys.argv[1:]
root = Path(root)
profile = json.loads((root / profile_path).read_text(encoding="utf-8"))
if profile["session"] != session:
    raise ValueError("Session and profile disagree")
duration = profile["mission_duration_s"]
steps = profile["max_steps"]
interval = profile["frame_archive_interval_s"]
for value, low, high in ((duration, 150, 660), (steps, 60, 180), (interval, 15, 60)):
    if type(value) is not int or not low <= value <= high:
        raise ValueError("Invalid single-segment budget")
pin = profile["validation"]
manifest_bytes = (root / pin["path"]).read_bytes()
if hashlib.sha256(manifest_bytes).hexdigest() != pin["sha256"]:
    raise ValueError("Manifest digest mismatch")
manifest = json.loads(manifest_bytes)
for name, digest in (manifest["source_sha256"] | manifest["control_source_sha256"]).items():
    if hashlib.sha256((root / name).read_bytes()).hexdigest() != digest:
        raise ValueError(f"Source digest mismatch: {name}")
if shutil.disk_usage(root).free < (160_000_000 if mode == "--execute" else 12_000_000):
    raise RuntimeError("Insufficient space for trial logs")
roi = profile.get("roi") if kind == "return" else [0] * 4
if not isinstance(roi, list) or len(roi) != 4 or any(type(n) is not int for n in roi):
    raise ValueError("A return requires the camera ROI [x, y, width, height]")
print(duration + 90, duration + 75, interval, pin["path"], profile["command_topic"], *roi)
PY
)"
read -r SUPPORT_DURATION WATCHDOG_DURATION FRAME_INTERVAL MANIFEST TOPIC X Y W H <<< "$settings"

mkdir -p "$ROOT/runtime" "$ROOT/reports"
exec 9>"$ROOT/runtime/session.lock"
flock -w3 9
OUT="$ROOT/reports/$SESSION"
CROOT="$APRILTAG_CONTAINER_ROOT"
COUT="$CROOT/reports/$SESSION"
if [[ "$MODE" == --check ]]; then
 mkdir -p "$ROOT/reports/preflight"
 OUT="$ROOT/reports/preflight/$SESSION"
 COUT="$CROOT/reports/preflight/$SESSION"
fi
mkdir "$OUT"
mkdir "$OUT/runtime" "$OUT/vision"

CAP= WAIST= VISION_STOPPER=
stop_reader() {
 docker exec "$APRILTAG_CONTAINER" python3 -c '
import os, signal, sys
from pathlib import Path
path = Path(sys.argv[1])
if path.is_file():
    pid = int(path.read_text())
    command = Path(f"/proc/{pid}/cmdline")
    if command.is_file():
        args = command.read_bytes().split(b"\0")
        if any(arg.endswith(os.fsencode("/" + sys.argv[2])) for arg in args) and any(
            arg.startswith(os.fsencode(sys.argv[3] + "/")) for arg in args
        ):
            os.kill(pid, signal.SIGTERM)
' "$COUT/$1.pid" "$2" "$COUT" 2>/dev/null || true
}
cleanup() {
 if [[ -n "$VISION_STOPPER" ]]; then kill -TERM "$VISION_STOPPER" 2>/dev/null || true; fi
 stop_reader pose return_vision_worker.py
 for pid in "$CAP" "$WAIST"; do
  if [[ -n "$pid" ]]; then
   kill -TERM "$pid" 2>/dev/null || true
   wait "$pid" 2>/dev/null || true
  fi
 done
 # A watchdog from an interrupted execution must be able to finish its stop.
 if [[ ! -e "$OUT/execution.claim" || -f "$OUT/watchdog_result.json" ]]; then
  stop_reader watchdog tag_pixel_v4_single_continuous_watchdog.py
 fi
}
trap cleanup EXIT

python3 -u "$ROOT/motion_trial_20261109/waist_guard.py" --output "$OUT/runtime/waist_live.json" \
 --duration "$SUPPORT_DURATION" >"$OUT/waist_guard.log" 2>&1 9>&- &
WAIST=$!

if [[ "$KIND" == return ]]; then
 python3 -u "$ROOT/tools/capture_latest.py" --device "$APRILTAG_CAMERA_DEVICE" \
  --serial "$APRILTAG_CAMERA_SERIAL" --output-dir "$OUT/runtime" \
  --duration "$SUPPORT_DURATION" >"$OUT/capture.log" 2>&1 9>&- &
 CAP=$!
 for attempt in {1..40}; do
  [[ -f "$OUT/runtime/latest.npz" ]] && break
  kill -0 "$CAP"
  sleep .15
 done
 [[ -f "$OUT/runtime/latest.npz" ]]
 docker exec -e APRILTAG_ROS_ENV "$APRILTAG_CONTAINER" bash -c '
  source "$APRILTAG_ROS_ENV" || exit
  export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
  echo $$ >"$1/pose.pid"
  exec python3 -u "$2/tools/return_vision_worker.py" --latest-dir "$1/runtime" \
   --output-dir "$1/vision" --duration "$3" --frame-interval-s "$4" --manifest "$2/$5" \
   --serial "$6" --roi "$7" "$8" "$9" "${10}"
 ' _ "$COUT" "$CROOT" "$SUPPORT_DURATION" "$FRAME_INTERVAL" "$MANIFEST" \
  "$APRILTAG_CAMERA_SERIAL" "$X" "$Y" "$W" "$H" >"$OUT/pose.log" 2>&1 9>&- &
 if [[ "$MODE" == --execute ]]; then
  (
   while [[ ! -f "$OUT/runtime/stop_vision.request" ]]; do sleep .05; done
   stop_reader pose return_vision_worker.py
   kill -TERM "$CAP" 2>/dev/null || true
  ) 9>&- &
  VISION_STOPPER=$!
 fi
else
 sleep 2
fi
kill -0 "$WAIST"

if [[ "$MODE" == --execute ]]; then
 docker exec -e APRILTAG_ROS_ENV "$APRILTAG_CONTAINER" bash -c '
  source "$APRILTAG_ROS_ENV" || exit
  export PYTHONDONTWRITEBYTECODE=1
  echo $$ >"$1/watchdog.pid"
  exec python3 -u "$2/motion_trial_20261109/tag_pixel_v4_single_continuous_watchdog.py" \
   --session "$3" --output "$1/watchdog_result.json" --duration "$4" --command-topic "$5"
 ' _ "$COUT" "$CROOT" "$SESSION" "$WATCHDOG_DURATION" "$TOPIC" >"$OUT/watchdog.log" 2>&1 9>&- &
fi

docker exec -e APRILTAG_CAMERA_SERIAL -e APRILTAG_ROS_ENV "$APRILTAG_CONTAINER" bash -c '
 source "$APRILTAG_ROS_ENV" || exit
 export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
 exec python3 -u "$2/motion_trial_20261109/$5" "$3" --profile "$2/$4" --output-dir "$1"
' _ "$COUT" "$CROOT" "$MODE" "$PROFILE" "$CONTROLLER" 9>&-
