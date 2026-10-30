# Project Assumptions Log

This document records engineering decisions and assumptions made during implementation, with reasoning as requested by the company assignment.

| ID | Topic | Decision / Assumption | Reason |
|---|---|---|---|
| A01 | Robot Model | **TurtleBot3 Burger** (`TURTLEBOT3_MODEL=burger`) | Burger is a differential-drive robot with a circular footprint ($r \approx 0.105\text{ m}$) and a 360° planar 2D LiDAR. Its symmetric footprint makes costmap inflation tuning and obstacle avoidance straightforward and robust in tight indoor environments like `turtlebot3_world`. |
| A02 | Simulation Stack | **Gazebo Classic 11.10.2** with ROS 2 Humble | Matches the host environment (Ubuntu 24.04 with ROS 2 Humble). `ros-humble-gazebo-ros-pkgs` interfaces directly with Gazebo Classic 11. |
| A03 | Workspace Layout | Workspace root: `nav2_goal_task/` inside project repository | Isolates custom package source code (`src/`) and compilation outputs (`build/`, `install/`, `log/`) while keeping git version control at the repository root. |
| A04 | Middleware & Simulator Config | Loopback multicast enabled & `GAZEBO_PLUGIN_PATH` includes ROS library directory | Required on Linux 6.8+ kernel for FastDDS local communication on loopback, and allows Gazebo Classic to dynamically load `libgazebo_ros_factory.so` for spawning robot models. |
| A05 | Map Selection | Copied `turtlebot3_world` map to `nav2_goal_task/maps/map.yaml` | Provides a version-controlled, self-contained occupancy grid matching the Gazebo simulation environment without depending on system package paths. |
| A06 | Nav2 Parameters | Copied parameters to `nav2_goal_task/config/nav2_params.yaml` | Localizes Nav2 controller, planner, AMCL, and costmap layer parameters to the workspace for reproducible tuning. |
| A07 | Action Client Architecture | Asynchronous `ActionClient` with spinning event loop and parameter configuration | Asynchronous action invocation prevents blocking the ROS 2 executor, allowing real-time feedback logging and responsive Ctrl+C goal cancellation handling. |
| A08 | Obstacle Injection Mechanism | Gazebo entity spawner via `/spawn_entity` ROS 2 service using custom SDF box model | Uses standard Gazebo ROS factory interface to inject physical geometric obstacles at runtime without modifying the static map file, ensuring true dynamic replanning. |
| A09 | Unified Bringup Launch | Composite launch file aggregating `turtlebot3_world`, `nav2_bringup`, and `rviz2` | Enables single-command bringup of the entire simulation and navigation pipeline for reviewers while allowing modular overrides. |
| A10 | Map/Config Placement | `maps/` and `config/` duplicated inside `src/nav2_goal_navigator/` package tree | ROS 2 `ament_python` requires `data_files` sources to be relative paths from the package root; placing data inside the package tree is the canonical approach. Workspace-level copies kept as source of truth. Phase 7 clean-build test revealed this requirement. |
