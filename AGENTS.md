# Project conventions

- Target Robot Operating System 2 (ROS 2) Jazzy on Ubuntu 24.04; keep all source and generated trial data in this workspace.
- Set `ROS_HOME` and `ROS_LOG_DIR` to workspace-local paths when running commands so ROS logs stay here too.
- `amr_lab/core` is pure Python and must not import `rclpy`. ROS wrappers stay in `nodes/`.
- Keep source files near 150 lines or fewer. Explain acronyms on first use in each file and use plain-English comments.
- Algorithm plug-ins expose one documented public method: `Allocator.assign(state)` or `ConflictResolver.plan(robot_state, peer_view)`. Register implementations only in `core/registry.py` and select by ROS parameter.
- Core messages are dataclasses serialized as JSON in `std_msgs/String` on `/bus/` topics. Keep packet-loss random streams seeded and per receiver.
- Robots act only on `/tick`. Decisions must explain who acted, what happened, and why; simulator-only ground truth counts vertex and swap collisions.
- Keep allocator costs based on A-star path distance. Resolve conflicts using heard space-time reservations, stale-peer waiting, numbered choke resources, ascending resource order, and lower-id deterministic tie breaks.
- Parameters belong in `config/params.yaml`; the ASCII warehouse belongs in `config/warehouse.yaml`. Never report invented or adjusted trial results.
- Tests cover A-star, Hungarian optimality, reservation conflicts, seeded loss, and launch termination. Build with `colcon build`; test with pytest in a sourced ROS environment.
- Do not add Gazebo, Navigation 2 (Nav2), machine learning, vision, web UI, Compute Unified Device Architecture (CUDA), or Python dependencies beyond NumPy, SciPy, and Matplotlib.
