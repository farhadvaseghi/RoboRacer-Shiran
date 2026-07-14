#!/usr/bin/env bash
# deploy.sh — copy the prepared real-car files from this PC to the car.
# Run from Git Bash on the PC:  bash deploy.sh
# (Passwordless SSH key is already installed, so no password prompts.)
set -e
CAR=roboracer@192.168.50.10
PCREPO="/c/Users/Student/Documents/Shiran-Hozuri/RoboRacer-Shiran"
PCRR="/c/Users/Student/Documents/Shiran-Hozuri/nav2-realcar-deploy/rr"
CARREPO="~/roboracer_ws/src/RoboRacer-Shiran"

echo "== copying Nav2 real-car config + launch into the car repo =="
scp "$PCREPO/roboracer_estimation/config/nav2_params_real.yaml" \
    "$CAR:$CARREPO/roboracer_estimation/config/nav2_params_real.yaml"
scp "$PCREPO/roboracer_estimation/launch/autonomous_real.launch.py" \
    "$CAR:$CARREPO/roboracer_estimation/launch/autonomous_real.launch.py"

echo "== copying helper / logging scripts + docs into ~/rr =="
ssh "$CAR" 'mkdir -p ~/rr'
scp "$PCRR/"rr_*.sh "$CAR:~/rr/"
scp "$PCRR/LOCALIZATION.md" "$CAR:~/rr/"
ssh "$CAR" 'chmod +x ~/rr/*.sh && echo "chmod ok"'

echo "== DONE. Next on the car: colcon build (see guide step 2) =="
