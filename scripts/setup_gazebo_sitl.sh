#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
YOLOV5_DIR="${YOLOV5_DIR:-$HOME/yolov5}"
ARDUPILOT_VENV="${ARDUPILOT_VENV:-$HOME/venv-ardupilot}"
YOLO_VENV="${YOLO_VENV:-$HOME/venv-yolov5}"
UPDATE_EXISTING="${UPDATE_EXISTING:-0}"
INSTALL_CUDA_TORCH="${INSTALL_CUDA_TORCH:-auto}"

# shellcheck disable=SC1091
source "$REPO_DIR/config/versions.env"

if [[ ! -r /etc/os-release ]]; then
  echo "This installer requires Ubuntu under WSL2." >&2
  exit 1
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" ]]; then
  echo "Unsupported distribution: ${PRETTY_NAME:-unknown}. Ubuntu 24.04 is required." >&2
  exit 1
fi
if [[ "${VERSION_ID:-}" != "24.04" ]]; then
  echo "warning: verified on Ubuntu 24.04; detected ${PRETTY_NAME:-unknown}" >&2
fi

echo "[1/8] Installing Ubuntu and Gazebo dependencies"
sudo apt-get update
sudo apt-get install -y curl gnupg lsb-release ca-certificates
sudo install -d -m 0755 /usr/share/keyrings
if [[ ! -s /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]]; then
  sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
sudo apt-get update
sudo apt-get install -y \
  build-essential ccache cmake git ninja-build pkg-config \
  python3 python3-dev python3-pip python3-venv \
  joystick usbutils mesa-utils rapidjson-dev libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav gstreamer1.0-gl \
  gz-jetty libgz-sim10-dev

clone_or_update() {
  local url="$1"
  local directory="$2"
  local revision="$3"
  local recursive="${4:-0}"

  if [[ ! -d "$directory/.git" ]]; then
    echo "Cloning $url -> $directory"
    if [[ "$recursive" == "1" ]]; then
      git clone --recurse-submodules "$url" "$directory"
    else
      git clone "$url" "$directory"
    fi
  elif [[ "$UPDATE_EXISTING" == "1" ]]; then
    echo "Fetching updates in $directory"
    git -C "$directory" fetch --all --tags --prune
  else
    echo "Using existing checkout without modifying it: $directory"
    return
  fi

  git -C "$directory" checkout --detach "$revision"
  if [[ "$recursive" == "1" ]]; then
    git -C "$directory" submodule update --init --recursive
  fi
}

echo "[2/8] Preparing pinned upstream source trees"
clone_or_update https://github.com/ArduPilot/ardupilot.git \
  "$ARDUPILOT_DIR" "$ARDUPILOT_REV" 1
clone_or_update https://github.com/ArduPilot/ardupilot_gazebo.git \
  "$ARDUPILOT_GAZEBO_DIR" "$ARDUPILOT_GAZEBO_REV"
clone_or_update https://github.com/ultralytics/yolov5.git \
  "$YOLOV5_DIR" "$YOLOV5_REV"

echo "[3/8] Installing ArduPilot prerequisites"
(
  cd "$ARDUPILOT_DIR"
  Tools/environment_install/install-prereqs-ubuntu.sh -y
  git submodule update --init --recursive
)

if [[ ! -x "$ARDUPILOT_VENV/bin/python" ]]; then
  echo "ArduPilot virtual environment was not created at $ARDUPILOT_VENV" >&2
  echo "Set ARDUPILOT_VENV to the path created by install-prereqs-ubuntu.sh." >&2
  exit 1
fi

echo "[4/8] Building ArduCopter SITL"
(
  cd "$ARDUPILOT_DIR"
  PATH="$ARDUPILOT_VENV/bin:/usr/lib/ccache:$PATH" ./waf configure --board sitl
  PATH="$ARDUPILOT_VENV/bin:/usr/lib/ccache:$PATH" ./waf copter
)

echo "[5/8] Building ardupilot_gazebo"
GZ_VERSION=jetty cmake -S "$ARDUPILOT_GAZEBO_DIR" \
  -B "$ARDUPILOT_GAZEBO_DIR/build" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -G Ninja
GZ_VERSION=jetty cmake --build "$ARDUPILOT_GAZEBO_DIR/build" -j"$(nproc)"

echo "[6/8] Creating YOLO Python environment"
if [[ ! -x "$YOLO_VENV/bin/python" ]]; then
  python3 -m venv --system-site-packages "$YOLO_VENV"
fi
"$YOLO_VENV/bin/python" -m pip install --upgrade pip
if [[ "$INSTALL_CUDA_TORCH" == "auto" ]]; then
  if command -v nvidia-smi >/dev/null 2>&1; then
    INSTALL_CUDA_TORCH=1
  else
    INSTALL_CUDA_TORCH=0
  fi
fi
if [[ "$INSTALL_CUDA_TORCH" == "1" ]]; then
  "$YOLO_VENV/bin/python" -m pip install \
    torch==2.6.0 torchvision==0.21.0 \
    --index-url https://download.pytorch.org/whl/cu124
else
  "$YOLO_VENV/bin/python" -m pip install torch==2.6.0 torchvision==0.21.0
fi
"$YOLO_VENV/bin/python" -m pip install -r "$REPO_DIR/requirements-yolo.txt"

echo "[7/8] Installing project launchers"
find "$SCRIPT_DIR" -maxdepth 1 -type f -exec chmod +x {} +
BIN_DIR="${BIN_DIR:-$HOME/.local/bin}" "$SCRIPT_DIR/install_launchers.sh"
if ! grep -Fq '# yolo-autonomous-drone' "$HOME/.bashrc"; then
  {
    echo
    echo '# yolo-autonomous-drone'
    printf 'export PATH="%s:$PATH"\n' "$HOME/.local/bin"
    printf 'source "%s"\n' "$SCRIPT_DIR/project-env.sh"
  } >> "$HOME/.bashrc"
fi

echo "[8/8] Running static validation"
"$SCRIPT_DIR/validate_setup.sh"

echo
echo "Setup complete. Open a new Ubuntu terminal or run:"
echo "  source ~/.bashrc"
if [[ ! -f "$REPO_DIR/weights/best_v5.pt" ]]; then
  echo "Then place the approved checkpoint at:"
  echo "  $REPO_DIR/weights/best_v5.pt"
fi
