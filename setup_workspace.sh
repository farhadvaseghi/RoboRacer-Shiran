#!/usr/bin/env bash
# setup_workspace.sh — Run once after cloning to configure the full simulation workspace.
#
# What this does:
#   1. Regenerates the local map assets used by the custom scenarios
#   2. Symlinks the vendored f1tenth_gym_ros and f1tenth_gym from deps/ into src/
#   3. Copies extra scenario launch/config/rviz files into f1tenth_gym_ros
#   4. Installs f110_gym via pip (required — colcon cannot build it)
#   5. Sources ROS if available and builds the workspace

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"   # roboracer_perception/
WS="$(cd "$SCRIPT_DIR/../.." && pwd)"          # roboracer_ws/
SRC="$WS/src"
DEPS="$SCRIPT_DIR/deps"
PATCHES="$SCRIPT_DIR/patches"

echo "=== RoboRacer workspace setup ==="
echo "Workspace : $WS"
echo "Deps      : $DEPS"
echo ""

# ── 0. Regenerate local simulation maps ──────────────────────────────────────

echo "[0/4] Regenerating simulation maps..."
python3 "$SCRIPT_DIR/tools/generate_solid_oval_map.py"
python3 "$SCRIPT_DIR/tools/generate_solid_oval_obstacles_map.py"
python3 "$SCRIPT_DIR/roboracer_perception/tube_track/generate_tube_map.py"

# ── 1. Symlink vendored dependencies into src/ ───────────────────────────────

for dep in f1tenth_gym_ros f1tenth_gym; do
    target="$SRC/$dep"
    source="$DEPS/$dep"

    if [ -L "$target" ]; then
        echo "[1/4] $dep — symlink already exists, updating..."
        ln -sfn "$source" "$target"
    elif [ -d "$target" ]; then
        echo "[1/4] WARNING: $target exists as a real directory (not a symlink)."
        echo "      Remove it manually and re-run this script to use the vendored version:"
        echo "        rm -rf $target"
        echo "        bash setup_workspace.sh"
    else
        echo "[1/4] Symlinking $dep..."
        ln -sfn "$source" "$target"
    fi
done

# ── 2. Copy extra scenario files into f1tenth_gym_ros ───────────────────────

echo "[2/4] Copying extra scenario files into f1tenth_gym_ros..."

cp "$PATCHES/sim_moving_obstacle.yaml" \
   "$SRC/f1tenth_gym_ros/config/sim_moving_obstacle.yaml"
cp "$PATCHES/gym_bridge.rviz"       "$SRC/f1tenth_gym_ros/launch/gym_bridge.rviz"
cp "$PATCHES/gym_bridge_solid.launch.py" \
   "$SRC/f1tenth_gym_ros/launch/gym_bridge_solid.launch.py"
cp "$PATCHES/gym_bridge_solid_cylinder.launch.py" \
   "$SRC/f1tenth_gym_ros/launch/gym_bridge_solid_cylinder.launch.py"
cp "$PATCHES/gym_bridge_solid_obstacles.launch.py" \
   "$SRC/f1tenth_gym_ros/launch/gym_bridge_solid_obstacles.launch.py"
cp "$PATCHES/gym_bridge_solid_obstacles_cylinder.launch.py" \
   "$SRC/f1tenth_gym_ros/launch/gym_bridge_solid_obstacles_cylinder.launch.py"
cp "$PATCHES/gym_bridge_solid_moving_obstacle.launch.py" \
   "$SRC/f1tenth_gym_ros/launch/gym_bridge_solid_moving_obstacle.launch.py"
cp "$PATCHES/gym_bridge_solid_moving_obstacle_cylinder.launch.py" \
   "$SRC/f1tenth_gym_ros/launch/gym_bridge_solid_moving_obstacle_cylinder.launch.py"
cp "$PATCHES/gym_bridge_solid_static_moving_obstacles.launch.py" \
   "$SRC/f1tenth_gym_ros/launch/gym_bridge_solid_static_moving_obstacles.launch.py"
cp "$PATCHES/gym_bridge_solid_static_moving_obstacles_cylinder.launch.py" \
   "$SRC/f1tenth_gym_ros/launch/gym_bridge_solid_static_moving_obstacles_cylinder.launch.py"

echo "  sim_moving_obstacle.yaml -> f1tenth_gym_ros/config/"
echo "  gym_bridge.rviz     -> f1tenth_gym_ros/launch/"
echo "  gym_bridge_solid.launch.py -> f1tenth_gym_ros/launch/"
echo "  gym_bridge_solid_cylinder.launch.py -> f1tenth_gym_ros/launch/"
echo "  gym_bridge_solid_obstacles.launch.py -> f1tenth_gym_ros/launch/"
echo "  gym_bridge_solid_obstacles_cylinder.launch.py -> f1tenth_gym_ros/launch/"
echo "  gym_bridge_solid_moving_obstacle.launch.py -> f1tenth_gym_ros/launch/"
echo "  gym_bridge_solid_moving_obstacle_cylinder.launch.py -> f1tenth_gym_ros/launch/"
echo "  gym_bridge_solid_static_moving_obstacles.launch.py -> f1tenth_gym_ros/launch/"
echo "  gym_bridge_solid_static_moving_obstacles_cylinder.launch.py -> f1tenth_gym_ros/launch/"

# ── 3. Install Python dependencies ────────────────────────────────────────────

PIP_USER_FLAG=()
if [ -z "${VIRTUAL_ENV:-}" ]; then
    PIP_USER_FLAG=(--user)
fi

echo "[3/4] Installing Python dependencies via pip..."
# numba in f110_gym expects a newer coverage API than Ubuntu 22.04's
# system package provides; upgrading coverage here avoids a blank RViz window
# caused by the simulator bridge crashing before it publishes TF.
python3 -m pip install "${PIP_USER_FLAG[@]}" --upgrade "coverage>=7,<8" --quiet
python3 -m pip install "${PIP_USER_FLAG[@]}" -e "$SRC/f1tenth_gym/" --quiet

# ── 4. Build workspace ───────────────────────────────────────────────────────

ROS_SETUP=""
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    ROS_SETUP="/opt/ros/humble/setup.bash"
fi

if [ -n "$ROS_SETUP" ]; then
    echo "[4/4] Sourcing ROS environment: $ROS_SETUP"
    # shellcheck source=/dev/null
    source "$ROS_SETUP"
else
    echo "[4/4] No ROS setup file found under /opt/ros; build may fail."
fi

echo "[4/4] Building workspace..."
cd "$WS"
colcon build --symlink-install --packages-ignore f110_gym

echo ""
echo "=== Setup complete ==="
echo "Run in every terminal before using ROS:"
echo "  source $WS/install/setup.bash"
