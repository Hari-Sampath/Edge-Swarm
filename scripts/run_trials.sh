#!/usr/bin/env bash
# Run fixed packet-loss trials with reproducible seed values.
set -euo pipefail
cd "$(dirname "$0")/.."
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source install/setup.bash
export ROS_HOME="$PWD/.ros" ROS_LOG_DIR="$PWD/log/ros"
mkdir -p results
for loss in 0.0 0.2; do
  for seed in $(seq 1 30); do
    AMR_RESULTS_DIR="results/seed_${seed}_loss_${loss}" ros2 launch amr_lab fleet.launch.py seed:="$seed" packet_loss:="$loss" headless:=true > "results/seed_${seed}_loss_${loss}.launch.log" 2>&1
  done
done
