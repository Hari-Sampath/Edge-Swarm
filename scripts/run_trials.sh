#!/usr/bin/env bash
# Append the 30 seeded trials for each configured packet-loss level.
set -eo pipefail
cd "$(dirname "$0")/.."
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source install/setup.bash
set -u
export ROS_HOME="$PWD/.ros" ROS_LOG_DIR="$PWD/log/ros"
mkdir -p results
loss_index=0
for loss in 0.0 0.2 0.4; do
  for seed in $(seq 1 30); do
    run="seed_${seed}_loss_${loss}"
    mkdir -p "results/$run"
    : > "results/$run/decisions.log"
    domain=$((100 + loss_index * 30 + seed))
    ROS_DOMAIN_ID="$domain" AMR_TRIALS_CSV="$PWD/results/trials.csv" \
    AMR_RESULTS_DIR="$PWD/results/$run" \
      ros2 launch amr_lab fleet.launch.py seed:="$seed" packet_loss:="$loss" \
      headless:=true > "results/$run.launch.log" 2>&1
  done
  loss_index=$((loss_index + 1))
done
