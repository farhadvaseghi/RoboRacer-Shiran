#!/usr/bin/env bash
# setup_workspace.sh — Run once after cloning to configure the full simulation workspace.
#
# What this does:
#   1. Clones f1tenth_gym_ros and f1tenth_gym (f110_gym) if not already present
#   2. Copies patched versions of gym_bridge.py, gym_bridge_launch.py, and sim.yaml
#      from the patches/ directory — no hardcoded usernames, improved collision handling
#   3. Installs f110_gym via pip (required — colcon cannot build it)
#   4. Builds the workspace

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"   # roboracer_perception/
WS="$(cd "$SCRIPT_DIR/../.." && pwd)"          # roboracer_ws/
SRC="$WS/src"
PATCHES="$SCRIPT_DIR/patches"

echo "=== RoboRacer workspace setup ==="
echo "Workspace : $WS"
echo "Patches   : $PATCHES"
echo ""

# ── 1. Clone dependencies if missing ────────────────────────────────────────

if [ ! -d "$SRC/f1tenth_gym_ros" ]; then
    echo "[1/4] Cloning f1tenth_gym_ros..."
    git -C "$SRC" clone https://github.com/f1tenth/f1tenth_gym_ros.git
else
    echo "[1/4] f1tenth_gym_ros already present — skipping clone."
fi

if [ ! -d "$SRC/f1tenth_gym" ]; then
    echo "[1/4] Cloning f1tenth_gym..."
    git -C "$SRC" clone https://github.com/f1tenth/f1tenth_gym.git
else
    echo "[1/4] f1tenth_gym already present — skipping clone."
fi

# ── 2. Apply patches ─────────────────────────────────────────────────────────

echo "[2/4] Applying patches to f1tenth_gym_ros..."

cp "$PATCHES/sim.yaml"              "$SRC/f1tenth_gym_ros/config/sim.yaml"
cp "$PATCHES/gym_bridge_launch.py"  "$SRC/f1tenth_gym_ros/launch/gym_bridge_launch.py"
cp "$PATCHES/gym_bridge.py"         "$SRC/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py"

echo "  sim.yaml            -> f1tenth_gym_ros/config/"
echo "  gym_bridge_launch.py -> f1tenth_gym_ros/launch/"
echo "  gym_bridge.py       -> f1tenth_gym_ros/f1tenth_gym_ros/"

# ── 3. Install f110_gym via pip ──────────────────────────────────────────────

echo "[3/4] Installing f110_gym via pip..."
pip install -e "$SRC/f1tenth_gym/" --quiet

# ── 4. Build workspace ───────────────────────────────────────────────────────

echo "[4/4] Building workspace..."
cd "$WS"
colcon build --symlink-install --packages-ignore f110_gym

echo ""
echo "=== Setup complete ==="
echo "Run in every terminal before using ROS:"
echo "  source $WS/install/setup.bash"
