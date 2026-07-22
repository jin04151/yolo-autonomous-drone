#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/project-env.sh"

failures=0
check_command() {
  if command -v "$1" >/dev/null 2>&1; then
    printf '[ok] command: %s\n' "$1"
  else
    printf '[missing] command: %s\n' "$1" >&2
    failures=$((failures + 1))
  fi
}

check_path() {
  if [[ -e "$1" ]]; then
    printf '[ok] path: %s\n' "$1"
  else
    printf '[missing] path: %s\n' "$1" >&2
    failures=$((failures + 1))
  fi
}

check_command gz
check_command cmake
check_path "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py"
check_path "$ARDUPILOT_DIR/build/sitl/bin/arducopter"
check_path "$ARDUPILOT_GAZEBO_DIR/build/libArduPilotPlugin.so"
check_path "$YOLOV5_DIR/models/common.py"
check_path "$YOLO_VENV/bin/python"

python3 - "$REPO_DIR/sim/gazebo" <<'PY'
import pathlib
import sys
import xml.etree.ElementTree as ET

root = pathlib.Path(sys.argv[1])
files = sorted(root.rglob("*.sdf")) + sorted(root.rglob("model.config"))
for path in files:
    text = path.read_text(encoding="utf-8")
    if "<gz:" in text and "xmlns:gz=" not in text:
        text = text.replace(
            "<sdf ", '<sdf xmlns:gz="http://gazebosim.org/schema" ', 1
        )
    ET.fromstring(text)
    print(f"[ok] XML: {path}")
PY

# These standalone models do not require model:// include resolution.
gz sdf --check "$REPO_DIR/sim/gazebo/models/ensamb_with_standoffs/model.sdf"
gz sdf --check "$REPO_DIR/sim/gazebo/models/target_basket/model.sdf"
python3 -m unittest discover -s "$REPO_DIR/tests" -v

if [[ -x "$YOLO_VENV/bin/python" ]]; then
  "$YOLO_VENV/bin/python" - <<'PY'
import cv2
import torch
from gz.transport import Node

print(f"[ok] OpenCV {cv2.__version__}, GStreamer={cv2.getBuildInformation().find('GStreamer:                   YES') >= 0}")
print(f"[ok] Torch {torch.__version__}")
print(f"[info] Torch CUDA available={torch.cuda.is_available()}")
print("[ok] Gazebo Python transport import")
PY
fi

if [[ -f "$REPO_DIR/weights/best_v5.pt" ]]; then
  printf '[ok] weights: %s\n' "$REPO_DIR/weights/best_v5.pt"
else
  printf '[warning] weights missing: %s\n' "$REPO_DIR/weights/best_v5.pt" >&2
fi

if (( failures > 0 )); then
  echo "Validation failed with $failures missing requirement(s)." >&2
  exit 1
fi
echo "Static validation passed."
