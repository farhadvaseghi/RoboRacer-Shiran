#!/usr/bin/env bash
# setup_workspace.sh — Run once after cloning to configure the full simulation workspace.
#
# What this does:
#   1. Symlinks the vendored f1tenth_gym_ros and f1tenth_gym from deps/ into src/
#   2. Installs f110_gym via pip (required — colcon cannot build it)
#   3. Builds the workspace

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"   # roboracer_perception/
WS="$(cd "$SCRIPT_DIR/../.." && pwd)"          # roboracer_ws/
SRC="$WS/src"
DEPS="$SCRIPT_DIR/deps"

echo "=== RoboRacer workspace setup ==="
echo "Workspace : $WS"
echo "Deps      : $DEPS"
echo ""

# ── 1. Symlink vendored dependencies into src/ ───────────────────────────────

for dep in f1tenth_gym_ros f1tenth_gym; do
    target="$SRC/$dep"
    source="$DEPS/$dep"

    if [ -L "$target" ]; then
        echo "[1/3] $dep — symlink already exists, updating..."
        ln -sfn "$source" "$target"
    elif [ -d "$target" ]; then
        echo "[1/3] WARNING: $target exists as a real directory (not a symlink)."
        echo "      Remove it manually and re-run this script to use the vendored version:"
        echo "        rm -rf $target"
        echo "        bash setup_workspace.sh"
    else
        echo "[1/3] Symlinking $dep..."
        ln -sfn "$source" "$target"
    fi
done

# ── 2. Install f110_gym via pip ──────────────────────────────────────────────

echo "[2/3] Installing f110_gym via pip..."
pip install -e "$SRC/f1tenth_gym/" --quiet

# ── 3. Build workspace ───────────────────────────────────────────────────────

echo "[3/3] Building workspace..."
cd "$WS"
colcon build --symlink-install --packages-ignore f110_gym

echo ""
echo "=== Setup complete ==="
echo "Run in every terminal before using ROS:"
echo "  source $WS/install/setup.bash"
