# forklift_gazebo

A Gazebo Sim 8 (Harmonic) test bench for the forklift robot defined in `forklift_description`. It contains:

- the `lift_test` world
- a test parcel model
- the ROS ↔ Gazebo bridge configuration
- the simulation launch file
- the automated acceptance tests (T1–T14)

## Contents

| Path | Role |
|---|---|
| `worlds/lift_test.sdf` | Flat matte floor, one static rail pair at x = 1.5 m with the sealed parcel on top, a sun and two spot lights (all cast shadows). Systems: Physics (DART, 1 ms step, RTF 1.0), UserCommands, SceneBroadcaster, Sensors (ogre2). |
| `models/test_parcel/` | 0.20 m cube, 0.5 kg, brown. A red seal (0.10 × 0.10 × 0.002 m, visual only) sits on its −x face. |
| `config/ros_gz_bridge.yaml` | `ros_gz_bridge` config for `/clock`, `/cmd_vel`, `/odom`, `/tf`, `/joint_states`, `/fork/cmd_height` and both `camera_info` topics |
| `launch/sim.launch.py` | Starts the gz sim server (`-s`), robot_state_publisher (`use_sim:=true`), the spawn, `parameter_bridge`, `image_bridge` and the command chain (`forklift_safety/control.launch.py`: twist_mux + cmd_vel_gate), plus RViz if asked for. With `gui:=true` the Gazebo GUI (`gz sim -g`) starts only once `scripts/wait_for_models.py` sees the robot and `gui_wait_models` in the server scene (DECISIONS D-52) |
| `scripts/acceptance_test.py` | Runs the acceptance tests and writes the results to `<ws>/docs/` |

## Running

```bash
cd ~/cv_forklift_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch forklift_gazebo sim.launch.py                       # Gazebo GUI
ros2 launch forklift_gazebo sim.launch.py gui:=false headless:=true rviz:=true
```

Launch arguments:

| Argument | Meaning |
|---|---|
| `world` | World file to load |
| `gui` | `false` runs the server only |
| `gui_wait_models` | Comma-separated models, besides the robot, that must be in the scene before the Gazebo GUI opens. `station_sim.launch.py` sets it from `stations.yaml` |
| `headless` | EGL off-screen rendering for the camera sensors |
| `rviz` | Start RViz |
| `control` | Start twist_mux + cmd_vel_gate (default `true`). With it, the operator GUI can drive even when it runs in another terminal (DECISIONS D-42). `false` when something publishes `/cmd_vel` directly, as `acceptance_test.py` does: the gate's zero commands would otherwise override it. |
| `x`, `y`, `z`, `yaw` | Spawn pose |
| `params_file` | Robot parameter file |
| `verbosity` | Log verbosity |

`ros_gz_sim` adds `models/` to `GZ_SIM_RESOURCE_PATH` automatically, through the `<gazebo_ros gazebo_model_path>` export in `package.xml`. To load the world with plain `gz sim`, export the path yourself:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix forklift_gazebo)/share/forklift_gazebo/models
```

### Driving by hand

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
ros2 topic pub --once /fork/cmd_height std_msgs/msg/Float64 "{data: 0.06}"
```

## ROS interface

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS→GZ | DiffDrive, limited to ±0.5 m/s, ±1.0 m/s², ±1.5 rad/s, ±3 rad/s² |
| `/odom` | `nav_msgs/msg/Odometry` | GZ→ROS | 50 Hz, `odom` → `base_footprint` |
| `/tf` | `tf2_msgs/msg/TFMessage` | GZ→ROS | DiffDrive transform from `odom` to `base_footprint`. robot_state_publisher publishes the rest. |
| `/joint_states` | `sensor_msgs/msg/JointState` | GZ→ROS | 50 Hz, all joints |
| `/fork/cmd_height` | `std_msgs/msg/Float64` | ROS→GZ | Fork height in metres, 0–0.15. The controller moves at 0.05 m/s in velocity mode. |
| `/front_camera/image_raw`, `/lane_camera/image_raw` | `sensor_msgs/msg/Image` | GZ→ROS | 640×480 rgb8, 30 Hz, published by `image_bridge`. The `compressed`, `theora` and `zstd` image_transport variants are also published. |
| `/front_camera/camera_info`, `/lane_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | GZ→ROS | 30 Hz |
| `/clock` | `rosgraph_msgs/msg/Clock` | GZ→ROS | Published every step (1 kHz). All nodes use `use_sim_time`. |

Every camera image and `camera_info` message has `frame_id` set to `<camera>_optical_frame`.

## Acceptance tests

```bash
ros2 run forklift_gazebo acceptance_test.py                    # T1–T14 (about 5 minutes)
ros2 run forklift_gazebo acceptance_test.py --tests T1,T3,T7   # a subset
ros2 run forklift_gazebo acceptance_test.py --gui              # sessions with the Gazebo GUI
```

The script starts and stops its own simulation sessions, so do not have another simulation running on the same `GZ_PARTITION`.

| Session | Tests |
|---|---|
| A | T3, T4, T11, T14, T7, T6, T5 |
| B | T12, T13, T8, T9, T10 |

Ground truth comes from gz-transport (`/world/lift_test/dynamic_pose/info`). The results are written to:

- `docs/test_results.md` and `docs/test_results.json`
- plots and camera frames under `docs/images/`
- the TF tree in `docs/frames_sim.pdf`
- raw logs under `docs/logs/`

The scenario definitions and design choices are recorded in [`../../DECISIONS.md`](../../DECISIONS.md).
