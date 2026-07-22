#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN_DIR="${BIN_DIR:-$HOME/.local/bin}"

mkdir -p "$BIN_DIR"
for launcher in \
  gazebo-my-drone \
  gazebo-my-drone-headless \
  sitl-my-drone \
  yolo-basket-gazebo \
  joystick-bridge \
  drone-takeoff; do
  ln -sfn "$SCRIPT_DIR/$launcher" "$BIN_DIR/$launcher"
done

echo "Launchers installed in $BIN_DIR"
case ":$PATH:" in
  *":$BIN_DIR:"*) ;;
  *) echo "Add this to your shell: export PATH=\"$BIN_DIR:\$PATH\"" ;;
esac
