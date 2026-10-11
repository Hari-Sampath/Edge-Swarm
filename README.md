# AMR Lab

Deterministic, tick-driven four autonomous mobile robots (AMRs) in Robot Operating System 2 (ROS 2) Jazzy. It uses a simple American Standard Code for Information Interchange (ASCII) aisle map, SciPy Hungarian task rounds, decentralized space-time reservations, numbered choke resources, and an independently seeded lossy message bus. Gazebo is installed in the target VM but is not used in this iteration. Fixed tasks are generated from the run seed; `task_arrival_rate` is reserved and currently zero.

## Build and run

```bash
cd ~/Documents/amr_lab
source /opt/ros/jazzy/setup.bash
export ROS_HOME="$PWD/.ros" ROS_LOG_DIR="$PWD/log/ros"
colcon build --symlink-install
source install/setup.bash
ros2 launch amr_lab fleet.launch.py seed:=1 packet_loss:=0.2 headless:=true
```

The node exits after all tasks finish or `max_ticks`. It writes a measured CSV row to `results/trials.csv` and decisions to `results/run/decisions.log`. Set `AMR_RESULTS_DIR` and `AMR_TRIALS_CSV` to separate runs. Configure robot count, tick rate, task count, timeout, and map in `src/amr_lab/config/`.

## RViz2 and trials

In another sourced terminal run `rviz2 -d ~/Documents/amr_lab/src/amr_lab/rviz/amr_lab.rviz` while a run is active. RViz2 (Robot Visualization tool) markers show the grid, walls, choke points, robots, goals, and planned paths. Run 30 seeds at each configured loss value with `bash scripts/run_trials.sh`; plot the collected measured makespan values with `python3 plot_results.py`.

## Tests and algorithm plug-ins

Run `PYTHONPATH=src/amr_lab pytest -q src/amr_lab/test` after sourcing ROS and the workspace. To add an algorithm: (1) add a class in its allocator or conflict module with one documented public method, (2) add its name and class to `core/registry.py`, (3) select it using `allocator` or `conflict_resolver` in `config/params.yaml`.

## Recorded choices and limits

The default map is a 15 by 11 ASCII grid; tasks are reproducibly generated at startup. The simulator uses a ROS node timer and a central process to advance ground truth, while each robot plans with only its last received bus view. Robots park at open aisle junctions after delivery so they do not remain stopped in narrow passages. Packet drops are independent per receiving robot. The current map is connected, no Gazebo physics or Navigation 2 (Nav2) is involved, and task arrivals during a run are not implemented. Configuration parsing uses the deliberately small known YAML layout without adding a Python dependency. Measured collisions and deadlocks can be nonzero and must be read from the CSV; the program does not claim safety outcomes it did not measure.
