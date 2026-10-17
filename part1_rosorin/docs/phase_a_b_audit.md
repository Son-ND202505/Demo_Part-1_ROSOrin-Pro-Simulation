# Part 1 — Phase A and partial Phase B audit

Audit date: 2026-10-12 (Asia/Ho_Chi_Minh).

Historical audit record: the runtime blockers below were subsequently resolved.
See [Phase B/C results](phase_b_c_results.md) for the verified 2026-10-13 result.

## Phase A — Environment and source audit

**Objective:** validate source, dependencies, environment and an isolated build
strategy before system installation or vendor modifications.

**Actions performed:** inspected the actual project root, archive hashes,
`package.xml`, `setup.py`, launch files, world/URDF files, and `.typerc`; compared
the available runtime with the vendor's Gazebo documentation; simulated the
specific missing dependency installation.

**Verification and actual results:**

| Check | Result |
| --- | --- |
| OS / Python | Ubuntu 22.04.5 / Python 3.10.12 |
| ROS | Humble; visible initial `AMENT_PREFIX_PATH` is `/opt/ros/humble` |
| Simulator | Gazebo Classic 11.10.2 present; Ignition runtime missing |
| Tools | `ros2`, `colcon`, `rviz2`, `xacro`, `rosdep` available |
| Navigation / SLAM | Nav2 and SLAM Toolbox packages visible |
| Disk / RAM | Approximately 32 GB available disk; 31 GiB RAM, 11 GiB available during audit |
| GPU | User's host terminal and training environment previously confirmed CUDA and RTX 3080 Ti Laptop; sandbox `nvidia-smi` cannot access the driver |
| Archive | `unzip -tq` passed; SHA-256 matches the stored vendor manifest |
| Git | `git status --short` fails in the tool's view; `.git` is a restricted placeholder here. Host Git state remains unverified; no Git initialization attempted |
| `.typerc` | Read only, not sourced; sets `need_compile=False`, domain 0, hardware selections, and a system Cyclone DDS config URI |
| ROS environment | No initial ROS domain or RMW override was visible; existing `GAZEBO_MODEL_PATH` includes TurtleBot3 assets |

**Issues and root causes:**

1. The vendor uses `ros_ign_gazebo`, `ros_gz_sim`, `ros_gz_bridge` and
   `ign_ros2_control`. Gazebo Classic alone cannot run those launch/plugins.
2. `room_worlds.launch.py` looks up `holonomic_sim`, although the returned path is
   unused and the corresponding custom plugin is commented out in robot Xacro.
3. `setup.py` omits some nested mesh folders and assigns the table texture entry
   from `meshes/tea_table`. Runtime impact on the selected world requires checking.
4. Both spawn and bridge files contain code for static `map -> odom`; the spawn
   declarations are not currently returned. Avoid adding a static map transform
   when later running SLAM/AMCL, which must own that transform.
5. The top-level `slam.launch.py` and `navigation.launch.py` shown in the website
   are absent in this archive; include versions exist under `launch/include/`.
   Future reproduction instructions must follow the actual source.
6. The Xacro calls `$(find ign_ros2_control)` unconditionally for a plugin path;
   verify compatibility of the installed shim and plugin filename after installing
   dependencies. `wheel.xacro` has a suspicious numeric literal
   `1000000000.0.0`; its effect has not yet been tested in the simulator.

**Fixes applied:** no vendor source, system installation, driver, or shell startup
file changed. Selected a dedicated repository workspace for the build.

**Files created:** this report, `README.md`, build script, and
actual build/launch/dependency-plan logs under `part1_rosorin/results/audit/`.

**Status: PASS for the audit.** Runtime prerequisites are not satisfied.

**Next phase:** continue Phase B after approval of the system dependency proposal.

## Phase B — Build and launch diagnostic

**Objective:** build the vendor packages, resolve them through the ROS index, and
test the official room launch without changing the vendor source.

**Actions performed:** colcon build into `part1_rosorin/ws/`, source its local
overlay in a child shell, check package prefixes, run `--show-args`, then execute
the actual room launch with a 15-second timeout.

**Verification and actual results:**

- Build exit status: 0. `Summary: 2 packages finished [1.43s]`.
- `ros2 pkg prefix` resolves both packages to this dedicated install overlay.
- `--show-args` exits 0, prints `No arguments`; it does not run the opaque setup.
- Actual launch exits 1 before simulator processes start:
  `package 'holonomic_sim' not found`.
- Build workspace uses approximately 18 MB.
- `rosdep check` reports missing `ros-humble-ros-ign-bridge`,
  `ros-humble-ros-ign-gazebo`, and `ros-humble-ign-ros2-control`.
  The bridge code uses the modern package name `ros_gz_bridge` directly; the
  proposed installation covers that actual runtime package.
- Dependency dry run: 180 new packages, 0 upgraded, 0 removed, 26 not upgraded;
  estimated download 47.4 MB, added space 309.2 MB. Nothing installed.

**Issues/root cause:** successful Python package builds do not verify simulator
runtime dependencies. The room launch fails on an unused vendor package lookup.

**Fixes applied:** build isolation and local ROS log directory only. Minimal launch
adaptation will be verified once the simulator dependency is available.

**Evidence:** [build log](../results/audit/build.log),
[actual launch log](../results/audit/vendor_launch.log),
[apt simulation](../results/audit/apt_install_plan.log).

**Status: PARTIAL.** Package build/registration passed; robot spawn has not passed.

**Next phase:** install approved dependencies, fix the launch through a local
wrapper, then complete Gazebo verification before Phase C.

## Phase C — Robot control and simulated LiDAR

**Status: BLOCKED by incomplete Phase B.** No movement, LiDAR message, TF runtime,
or RViz sensor visualization has been verified. No SLAM/navigation was started.

## Official reference

<https://wiki.hiwonder.com/projects/rosorin-pro/en/latest/docs/9_Gazebo_Simulation.html>
