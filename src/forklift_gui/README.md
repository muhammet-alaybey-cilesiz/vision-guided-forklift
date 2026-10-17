# forklift_gui

Qt 6 / QML operator GUI for the forklift robot. It drives the robot, commands
the fork, latches an e-stop and monitors the robot's state. Later the same GUI
will be used to watch and override the autonomy.

* **Drive:** W/A/S/D or a virtual joystick, with a speed scale, an acceleration
  ramp and a deadman. Losing window focus stops the robot at once.
* **Fork:** target slider, numeric input and presets, with "target reached" and
  "not reached in 3 s" states.
* **E-stop:** latched; the GUI starts with it engaged. Space engages it. To
  release it, hold the button for 1.5 s.
* **Cameras:** two views with FPS, resolution, frame age, STALE, a crosshair
  and the `/cv/detections` overlay.
* **Monitoring:** telemetry with an odometry trail, topic health and the
  `/rosout` log with filter and search.
* **Dataset capture:** PNG pairs plus `metadata.jsonl`, single shot or
  continuous.

Design and decisions: [`ARCHITECTURE.md` §7](../../ARCHITECTURE.md) and
[`DECISIONS.md` D-22..D-41](../../DECISIONS.md). Test report:
[`REPORT_GUI.md`](../../REPORT_GUI.md).

## Architecture in one picture

```
QML view (Forklift.Gui)  ──binds──▶  core (forklift_gui_core, Qt only)  ◀──implements──  ROS adapter (rclcpp)
 Main.qml, panels/, components/       Backend singleton, controllers,                     RosBridge: node + MultiThreadedExecutor
 theme/Theme.qml, UiState.qml         models, ports I*Port                                 thread, image → QVideoFrame
                                                                 ◀──implements──  mock adapter (--mock)
```

* The core contains no ROS header. All rules live there: limits, ramp,
  deadman, e-stop latch, fork checks, health states and throttling. QML only
  displays data and forwards input.
* ROS callbacks never touch a `QObject`. Data reaches the GUI thread through
  queued `invokeMethod` calls with latest-value mailboxes, or through atomic
  topic counters.
* Telemetry reaches QML at most 30 Hz.

## Build

Requires the Ubuntu 24.04 system Qt (6.4.2). Install list:
[ARCHITECTURE §7.1](../../ARCHITECTURE.md).

```bash
cd ~/cv_forklift_ws
colcon build --symlink-install --packages-select forklift_gui forklift_safety forklift_gazebo forklift_bringup
source install/setup.bash
```

## Run

```bash
# Gazebo window + RViz + twist_mux + cmd_vel_gate + GUI
ros2 launch forklift_bringup bringup.launch.py

# Or in two terminals: the simulation starts the command chain itself (DECISIONS D-42)
ros2 launch forklift_gazebo sim.launch.py
# GUI only (it waits for the simulation and connects when it appears)
ros2 launch forklift_gui gui.launch.py

# No ROS graph: mock robot, synthetic cameras and logs
ros2 launch forklift_gui gui.launch.py mock:=true
ros2 run forklift_gui forklift_gui --mock

# Screenshot after a delay, then exit
ros2 run forklift_gui forklift_gui --mock --screenshot /tmp/gui.png --screenshot-delay 5
```

All topic names, limits, rates, presets and thresholds come from
`config/gui_params.yaml`. The node declares every parameter without a default,
so a missing key stops start-up with the key's name. Visual constants are in
`qml/theme/Theme.qml`.

## Keyboard

| Key | Action |
|---|---|
| Space | Engage the e-stop (never releases it) |
| W / S, A / D | Drive forward / back, turn left / right. Hold the key; releasing it stops the robot |
| R / F | Fork target +5 mm / −5 mm |
| 1 – 4 | Fork presets (Yer, Temas, Taşıma, Maks) |
| C | Capture one frame pair to the dataset |
| F1 / Esc | Toggle / close the help overlay |
| Mouse | Hold the E-STOP button for 1.5 s to release |

## Topics

| Topic | Type | Dir. | QoS |
|---|---|---|---|
| `/odom` | nav_msgs/Odometry | in | sensor data |
| `/joint_states` | sensor_msgs/JointState | in | sensor data |
| `/clock` | rosgraph_msgs/Clock | in | clock (best effort, 1) |
| `/front_camera/image_raw`, `/lane_camera/image_raw` | sensor_msgs/Image | in | best effort, 1 |
| `/front_camera/camera_info`, `/lane_camera/camera_info` | sensor_msgs/CameraInfo | in | best effort, 1 |
| `/rosout` | rcl_interfaces/Log | in | reliable, transient local, 100 |
| `/cv/detections` (optional) | vision_msgs/Detection2DArray | in | sensor data |
| `/cmd_vel/teleop` | geometry_msgs/Twist | out, 20 Hz while held | reliable, 1 |
| `/fork/cmd_height` | std_msgs/Float64 | out, on change | reliable, 1 |
| `/e_stop` | std_msgs/Bool | out, 10 Hz always | reliable, 1 |

## Tests

```bash
colcon test --packages-select forklift_gui && colcon test-result --verbose
```

* `test/core/`: QtTest unit tests U1–U8, plus telemetry throttle and log
  model. They use the mock adapter, a fake clock and the offscreen platform.
* Linters: clang-format, cpplint, lint_cmake, xmllint, flake8, and qmllint
  6.4.2 (the `forklift_gui_qmllint` target).
* `test/integration/integration_test.py` is the end-to-end suite I1–I12. It
  needs an X11 session, `xdotool`, `forklift_gazebo` and `forklift_safety`. It
  opens the GUI window and drives it with real key and mouse events, so do not
  type while it runs.

  ```bash
  ros2 run forklift_gui integration_test.py                 # all
  ros2 run forklift_gui integration_test.py --tests I2,I5   # selection
  ros2 run forklift_gui integration_test.py --screenshots   # also report screenshots
  ```

  Results are written to `docs/gui_test_results.{md,json}` and logs to
  `docs/logs/gui/`.
