#!/usr/bin/env bash
# setup_workspace.sh — Run once after cloning to configure the full simulation workspace.
#
# What this does:
#   1. Clones f1tenth_gym_ros and f1tenth_gym (f110_gym) if not already present
#   2. Applies the portable gym_bridge_launch.py and sim.yaml patches so the map
#      path is resolved from the installed package — no hardcoded usernames.
#   3. Installs f110_gym via pip (required — colcon cannot build it)
#   4. Builds the workspace

set -e

WS="$(cd "$(dirname "$0")/../.." && pwd)"   # roboracer_ws/
SRC="$WS/src"

echo "=== RoboRacer workspace setup ==="
echo "Workspace: $WS"

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

# ── 2. Patch sim.yaml — remove hardcoded map_path ───────────────────────────

SIM_YAML="$SRC/f1tenth_gym_ros/config/sim.yaml"
echo "[2/4] Patching sim.yaml..."
# Replace any map_path line (hardcoded or placeholder) with the override marker
python3 - <<'PYEOF'
import re, sys

path = "$SIM_YAML"
text = open(path).read()
# Replace map_path value with the override marker
text = re.sub(
    r"(map_path:\s*)'.+'",
    r"\1'OVERRIDDEN_BY_LAUNCH'",
    text
)
open(path, 'w').write(text)
print("  sim.yaml patched.")
PYEOF

# ── 3. Patch gym_bridge_launch.py — portable map path ───────────────────────

LAUNCH="$SRC/f1tenth_gym_ros/launch/gym_bridge_launch.py"
echo "[3/4] Patching gym_bridge_launch.py..."

python3 - <<'PYEOF'
import re

path = "$LAUNCH"
text = open(path).read()

# Only patch if not already patched
if 'roboracer_perception' not in text:
    # Insert map_path computation after config_dict lines, before bridge_node
    insert = (
        "\n    # Portable map path — resolved from installed package share.\n"
        "    map_path = os.path.join(\n"
        "        get_package_share_directory('roboracer_perception'),\n"
        "        'maps',\n"
        "        'oval_track'\n"
        "    )\n"
    )
    text = text.replace(
        "\n    bridge_node = Node(",
        insert + "\n    bridge_node = Node("
    )
    # Override map_path in bridge_node parameters
    text = re.sub(
        r"(parameters=\[config\])",
        "parameters=[config, {'map_path': map_path}]",
        text
    )
    # Override map_path in map_server_node
    text = re.sub(
        r"'yaml_filename': config_dict\['bridge'\]\['ros__parameters'\]\['map_path'\] \+ '\.yaml'",
        "'yaml_filename': map_path + '.yaml'",
        text
    )
    open(path, 'w').write(text)
    print("  gym_bridge_launch.py patched.")
else:
    print("  gym_bridge_launch.py already patched — skipping.")
PYEOF

# ── 4. Install f110_gym via pip and build workspace ─────────────────────────

echo "[4/4] Installing f110_gym via pip..."
pip install -e "$SRC/f1tenth_gym/" --quiet

echo "[4/4] Building workspace..."
cd "$WS"
colcon build --symlink-install --packages-ignore f110_gym

echo ""
echo "=== Setup complete ==="
echo "Run in every terminal before using ROS:"
echo "  source $WS/install/setup.bash"
