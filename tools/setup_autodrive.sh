#!/usr/bin/env bash
# setup_autodrive.sh — Start the AutoDRIVE RoboRacer Docker containers.
#
# STARTUP ORDER (critical):
#   1. Run this script with --devkit  → starts WebSocket server + ROS 2 bridge
#   2. Run this script with --sim     → starts Unity simulator (connects to devkit)
#   3. ros2 launch roboracer_perception autodrive_sim.launch.py
#
# Usage:
#   bash setup_autodrive.sh <TAG> --devkit    # Terminal 1 (start first)
#   bash setup_autodrive.sh <TAG> --sim       # Terminal 2 (start after devkit is ready)
#
# Available tags (https://hub.docker.com/r/autodriveecosystem/autodrive_roboracer_sim/tags):
#   2026-icra-explore    (default) latest
#   2026-icra-practice
#   2025-icra-explore
#
# Simulator modes (only with --sim):
#   --nographics   No rendering — fastest, recommended for WSL2 (default)
#   --headless     Xvfb virtual display
#   --gui          Unity window (requires working display)

set -e

TAG=${1:-2026-icra-explore}
SIM_IMAGE="autodriveecosystem/autodrive_roboracer_sim:${TAG}"
API_IMAGE="autodriveecosystem/autodrive_roboracer_api:${TAG}"

MODE="nographics"
CONTAINER="sim"
for arg in "$@"; do
    [[ "$arg" == "--devkit" ]]     && CONTAINER="devkit"
    [[ "$arg" == "--sim" ]]        && CONTAINER="sim"
    [[ "$arg" == "--gui" ]]        && MODE="gui"
    [[ "$arg" == "--headless" ]]   && MODE="headless"
    [[ "$arg" == "--nographics" ]] && MODE="nographics"
done

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║       AutoDRIVE RoboRacer — Docker Setup             ║"
echo "╚══════════════════════════════════════════════════════╝"

# GPU flag
GPU_FLAG=""
if command -v nvidia-smi &>/dev/null; then
    GPU_FLAG="--gpus all"
    echo "  GPU detected — using --gpus all"
else
    echo "  WARNING: nvidia-smi not found. Running on CPU."
fi

TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_PATCH="${TOOLS_DIR}/autodrive_bridge_patched.py"
BRIDGE_TARGET="/home/autodrive_devkit/install/autodrive_roboracer/lib/python3.10/site-packages/autodrive_roboracer/autodrive_bridge.py"

DOCKER_COMMON=(
    --rm -it
    --network=host
    --ipc=host
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    -v "${BRIDGE_PATCH}:${BRIDGE_TARGET}:ro"
    --env DISPLAY
    --privileged
    --entrypoint=""
)
[[ -n "$GPU_FLAG" ]] && DOCKER_COMMON+=($GPU_FLAG)

xhost local:root 2>/dev/null || true

if [[ "$CONTAINER" == "devkit" ]]; then
    # ── Step 1: Start devkit (WebSocket server must be up before sim) ────
    echo ""
    echo "  Starting DEVKIT container (ROS 2 bridge + WebSocket server)..."
    echo "  Wait for 'autodrive_bridge' to print its startup message,"
    echo "  then start the simulator in a second terminal:"
    echo "    bash setup_autodrive.sh ${TAG} --sim"
    echo ""
    docker run "${DOCKER_COMMON[@]}" --name autodrive_roboracer_api \
        "${API_IMAGE}" \
        /bin/bash -c "source /opt/ros/humble/setup.bash && \
                      source /home/autodrive_devkit/install/setup.bash && \
                      ros2 launch autodrive_roboracer bringup_headless.launch.py"

elif [[ "$CONTAINER" == "sim" ]]; then
    # ── Step 2: Start simulator (connects to devkit on port 4567) ────────
    echo ""
    case "$MODE" in
      gui)
        echo "  Starting SIMULATOR in GUI mode..."
        docker run "${DOCKER_COMMON[@]}" --name autodrive_roboracer_sim \
            "${SIM_IMAGE}" \
            /bin/bash -c "cd /home/autodrive_simulator && \
                          ./AutoDRIVE\ Simulator.x86_64"
        ;;
      headless)
        echo "  Starting SIMULATOR in HEADLESS mode (Xvfb)..."
        docker run "${DOCKER_COMMON[@]}" --name autodrive_roboracer_sim \
            "${SIM_IMAGE}" \
            /bin/bash -c "cd /home/autodrive_simulator && \
                          xvfb-run ./AutoDRIVE\ Simulator.x86_64 \
                          -ip 127.0.0.1 -port 4567"
        ;;
      nographics)
        echo "  Starting SIMULATOR in NO-GRAPHICS mode (recommended for WSL2)..."
        docker run "${DOCKER_COMMON[@]}" --name autodrive_roboracer_sim \
            "${SIM_IMAGE}" \
            /bin/bash -c "cd /home/autodrive_simulator && \
                          ./AutoDRIVE\ Simulator.x86_64 \
                          -batchmode -nographics -ip 127.0.0.1 -port 4567"
        ;;
    esac
else
    echo "ERROR: specify --devkit or --sim"
    echo "Usage: bash setup_autodrive.sh <TAG> --devkit | --sim [--nographics|--headless|--gui]"
    exit 1
fi
