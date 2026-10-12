# AMR Lab

Deterministic, tick-driven four autonomous mobile robots (AMRs) in Robot Operating System 2 (ROS 2) Jazzy. It uses a simple American Standard Code for Information Interchange (ASCII) aisle map, SciPy Hungarian task rounds, decentralized space-time reservations, numbered choke resources, and an independently seeded lossy message bus. Gazebo is installed in the target VM but is not used in this iteration. The simulator only advances ground truth; each robot plans in its own ROS process from its own pose and bus messages. The task server owns assignments and publishes them on robot-specific topics.

## Build and run

```bash
cd ~/Documents/amr_lab
source /opt/ros/jazzy/setup.bash
export ROS_HOME="$PWD/.ros" ROS_LOG_DIR="$PWD/log/ros"
colcon build --symlink-install
source install/setup.bash
ros2 launch amr_lab fleet.launch.py seed:=1 packet_loss:=0.2 headless:=true
```

The launch exits after all tasks finish or `max_ticks`. It writes a measured CSV row to `results/trials.csv` and decisions to `results/run/decisions.log`. Set `AMR_RESULTS_DIR` and `AMR_TRIALS_CSV` to separate runs. Configure robot count, tick rate, task count, timeout, and map in `src/amr_lab/config/`.

## RViz2 and trials

In another sourced terminal run `rviz2 -d ~/Documents/amr_lab/src/amr_lab/rviz/amr_lab.rviz` while a run is active. RViz2 (Robot Visualization tool) markers show the grid, walls, choke points, robots, goals, and planned paths. Run seeds 1–30 at packet loss 0.0, 0.2 and 0.4 with `bash scripts/run_trials.sh`. Print grouped means, standard deviations, minima, maxima and timeout counts, and save `results/summary.png` with `python3 scripts/summarize.py`.

## Tests and algorithm plug-ins

Run `PYTHONPATH=src/amr_lab pytest -q src/amr_lab/test` after sourcing ROS and the workspace. To add an algorithm: (1) add a class in its allocator or conflict module with one documented public method, (2) add its name and class to `core/registry.py`, (3) select it using `allocator` or `conflict_resolver` in `config/params.yaml`.

## Recorded choices and limits

The default map is a 15 by 11 ASCII grid; tasks are reproducibly generated at startup. The simulator applies requested moves and measures ground truth, while each robot plans from its own pose and its last received bus view. The task server owns task assignment. A zero `task_arrival_rate` releases all seeded tasks on the first tick; a positive rate is a seeded Bernoulli chance per tick to release the next task, at most one per tick. After a robot waits for `deadlock_ticks`, recovery uses a deterministic rotation over every robot id, including idle robots. The selected robot routes toward an open junction while others hold; idle robots also plan during recovery so they can clear a blocked aisle. The planner still rejects occupied cells and stale peer data. Each per-receiver bus delivery, including a retransmission, increments `messages_sent`; each delivery discarded by the receiver filter increments `messages_dropped`. Packet drops are independent per receiving robot. The current map is connected; Gazebo physics and Navigation 2 (Nav2) are not part of this version. A run records `timed_out` and `stalled_ticks` so incomplete work is visible. Configuration parsing uses the deliberately small known YAML layout without adding a Python dependency. Measured collisions and deadlocks can be nonzero and must be read from the CSV; the program does not claim safety outcomes it did not measure.
