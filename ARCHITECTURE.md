# CV Forklift — Architecture

Status: **Phases 1–4 done**: description, simulation and load tests are implemented, and T1–T14 run automatically. See `REPORT.md` for the results.
**Operator GUI** (`forklift_gui`, `forklift_bringup`, `forklift_safety`): implemented (phases 1–5, 2026-10-15/16); unit tests U1–U8 and integration tests I1–I12 in `docs/gui_test_results.md`, report in `REPORT_GUI.md`. See §7.
**Load stations** (`forklift_gazebo`): parametric multi-bay stations with ArUco markers, a `parcel` model and the `station_test` world (2026-10-19). Tests S1–S8 are in `docs/station_test_results.md`, and the report is `REPORT_STATIONS.md`. See §8.
**Factory world and appearance** (`forklift_gazebo`, `forklift_description`): generated `factory` world in a real-scale hall, industrial station/parcel materials and a visual-only robot body (2026-10-20; D-53, D-54). Report `REPORT_FACTORY.md`. See §9.

Scope: the robot description and a simulation test bench for a small forklift
robot. It is the platform for a computer-vision learning project. Perception,
lane following, autonomy and the task world are out of scope.

Target and verified environment: Ubuntu 24.04.5, ROS 2 Jazzy, Gazebo Sim
8.15.0 (Harmonic) from the ROS vendor packages (`/opt/ros/jazzy/opt/*_vendor`),
sdformat 14.9.0, gz-sensors 8.2.2, gz-physics 7.8.0 (DART 6.13.2),
gz-rendering 8.2.3 (ogre2), gz-transport 13.6.0, ros_gz 1.0.24, xacro 2.1.1.

---

## 1. Workspace layout

```
~/cv_forklift_ws/
├── ARCHITECTURE.md   DECISIONS.md   REPORT.md
├── docs/                                  view_frames PDFs, images/, logs/ (test evidence)
└── src/
    ├── forklift_description/              ament_cmake, simulator-agnostic robot
    │   ├── config/forklift_params.yaml    single source of truth for every number
    │   ├── urdf/
    │   │   ├── forklift.urdf.xacro        top-level assembly: args, YAML load, macro calls only
    │   │   ├── common/inertial_macros.xacro   box / cylinder / sphere inertials + composite helper
    │   │   ├── common/materials.xacro     colours (visual only)
    │   │   ├── components/chassis.xacro
    │   │   ├── components/drive_wheel.xacro   macro, param side:=left|right
    │   │   ├── components/caster.xacro
    │   │   ├── components/mast.xacro
    │   │   ├── components/fork_carriage.xacro prismatic fork_lift_joint + two forks
    │   │   ├── components/camera.xacro    macro: name, parent, pose, FOV, resolution, ...
    │   │   └── gazebo/forklift.gazebo.xacro   ONLY sim content: friction, camera sensors, DiffDrive/JPC/JSP
    │   ├── rviz/forklift.rviz
    │   └── launch/display.launch.py       robot_state_publisher + joint_state_publisher_gui + RViz
    └── forklift_gazebo/                   ament_cmake, Gazebo Sim bench
        ├── worlds/lift_test.sdf           test world (flat floor, rails, parcel, lights)
        ├── worlds/station_test.sdf        station world: same physics/lights, no objects (§8)
        │   (worlds/factory.sdf            generated at build time from config/factory.yaml, §9)
        ├── models/test_parcel/            0.2 m cube with a red seal (model.config, model.sdf)
        ├── config/ros_gz_bridge.yaml      bridge for every non-image topic
        ├── config/station_params.yaml     single source for station, marker and parcel geometry (§8)
        ├── config/stations.yaml           station layout (station_test and factory): poses, bays, bay ids, occupancy
        ├── config/factory.yaml            factory world: hall, lights, markings, props (§9)
        ├── config/factory_gui.config      Gazebo GUI layout + opening camera for factory
        ├── models/factory_materials/      CC0 photo textures (committed)
        ├── xacro/                         station.sdf.xacro, parcel.sdf.xacro, common.sdf.xacro
        ├── launch/sim.launch.py           gz sim + spawn + robot_state_publisher + bridges
        ├── launch/stations.launch.py      stations.yaml → xacro → ros_gz_sim create
        ├── launch/station_sim.launch.py   sim.launch.py (station_world: station_test | factory) + stations.launch.py
        └── scripts/
            ├── acceptance_test.py         T3–T14 automation (rclpy), writes to docs/
            ├── station_acceptance.py      S1–S8 automation (gz-transport only)
            ├── station_layout.py          shared bay / marker-ID / pose computations
            ├── generate_aruco_textures.py build step: ArUco PNGs + markers.yaml → share/
            ├── generate_factory_textures.py   build step: signs, labels, stripes → model://factory_signs
            ├── generate_factory_world.py  build step: factory.sdf + factory_lanes.yaml
            └── measure_sim_performance.py RTF / camera rate / CPU / GPU per world
```

### Modularity rules

* Only `gazebo/forklift.gazebo.xacro` contains Gazebo-specific XML. It is
  included only when `use_sim:=true`. The kinematic and dynamic model
  (links, joints, inertials, collisions, limits) has no simulator content.
* The gazebo file has three separate sections: *surface*, *sensors* and
  *control* (DiffDrive, JointPositionController, JointStatePublisher). To move
  to `gz_ros2_control` later, you replace only the *control* section of that
  file.
* Xacro arguments: `prefix` (frame/joint name prefix, default empty), `use_sim`
  (default `false`) and `params_file` (default
  `$(find forklift_description)/config/forklift_params.yaml`).

### Parameter flow

```
forklift_params.yaml ──xacro.load_yaml──► forklift.urdf.xacro ──► URDF string
                                                │                   │
                       component macros derive positions            ├─► robot_state_publisher (/robot_description, TF)
                       (e.g. caster z, wheel y, fork x)             └─► ros_gz_sim create ──► sdformat URDF→SDF ──► gz-sim
```

The YAML stores each value once, in the convention the spec uses: positions
relative to `base_link`, or `ground_z*` for absolute heights. The macros derive
all other values; see the header of the YAML. World objects (floor, rails,
parcel) are not robot parameters. Their SDF files are the source for them, and
the acceptance test reads those SDF files instead of copying their numbers.

---

## 2. Frame tree (REP-105)

```
odom                                   sim only: DiffDrive TF (odom → base_footprint)
└── base_footprint                     root link, no inertial (no KDL warning), z = 0 on the floor
    └── base_link                      base_joint, fixed, (0, 0, +0.05): drive-axle centre
        ├── chassis_link               chassis_joint, fixed, (-0.12, 0, +0.045)
        ├── left_wheel_link            left_wheel_joint, continuous, axis +y, (0, +0.17, 0)
        ├── right_wheel_link           right_wheel_joint, continuous, axis +y, (0, -0.17, 0)
        ├── caster_link                caster_joint, fixed, (-0.28, 0, -0.025)
        └── mast_link                  mast_joint, fixed, (0.095, 0, -0.03): mast base centre
            ├── fork_carriage_link     fork_lift_joint, prismatic +z, [0, 0.15] m, (0.025, 0, 0.05)
            │   ├── left_fork_link     left_fork_joint, fixed, (0.135, +0.04, -0.0525)
            │   └── right_fork_link    right_fork_joint, fixed, (0.135, -0.04, -0.0525)
            ├── front_camera_link      front_camera_joint, fixed, (0.005, 0, 0.43), rpy (0, 15°, 0)
            │   └── front_camera_optical_frame   fixed, rpy (-π/2, 0, -π/2)
            └── lane_camera_link       lane_camera_joint, fixed, (0.005, 0, 0.52), rpy (0, 45°, 0)
                └── lane_camera_optical_frame    fixed, rpy (-π/2, 0, -π/2)
```

Joint origins are parent-relative and derived from the YAML. Every link frame
except `base_footprint`, `base_link`, `mast_link` and the camera frames sits at
the geometric centre of its primitive. `base_link` itself is a massless frame
link; the chassis box lives in `chassis_link`.

**Simulator view.** When sdformat converts URDF to SDF it lumps fixed joints.
In Gazebo the model therefore has only four links: `base_footprint` (chassis,
caster, mast and cameras; 9.3 kg), `left_wheel_link`, `right_wheel_link` and
`fork_carriage_link` (plate and forks; 0.9 kg). It has three joints. sdformat
carries sensor poses and per-collision friction of lumped links over to the
merged link. T1 and the SDF inspection confirmed this: only the caster collision
has μ = 0. The drive-wheel *collision* is a sphere of r 0.05; the visual and
the inertia stay a cylinder (D-20). The fork joint is driven in PID force mode
(D-10).

---

## 3. ROS ↔ Gazebo interface

The Gazebo-side topic names equal the ROS names (set in
`forklift_params.yaml → gazebo.interface` and `cameras.*`). All ROS nodes run
with `use_sim_time: true`.

| ROS topic | ROS type | Dir. | Gazebo type | Rate | Gazebo endpoint / bridge |
|---|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | GZ→ROS | `gz.msgs.Clock` | every step (1 kHz, measured) | gz-sim server / `ros_gz_bridge` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS→GZ | `gz.msgs.Twist` | user | DiffDrive `topic` / `ros_gz_bridge` |
| `/odom` | `nav_msgs/msg/Odometry` | GZ→ROS | `gz.msgs.Odometry` | 50 Hz | DiffDrive `odom_topic`, frame `odom` → `base_footprint` / `ros_gz_bridge` |
| `/tf` (odom→base_footprint) | `tf2_msgs/msg/TFMessage` | GZ→ROS | `gz.msgs.Pose_V` | 50 Hz | DiffDrive `tf_topic` / `ros_gz_bridge` |
| `/joint_states` | `sensor_msgs/msg/JointState` | GZ→ROS | `gz.msgs.Model` | 50 Hz | JointStatePublisher (all joints) / `ros_gz_bridge` |
| `/fork/cmd_height` | `std_msgs/msg/Float64` [m] | ROS→GZ | `gz.msgs.Double` | user | JointPositionController `topic` / `ros_gz_bridge` |
| `/front_camera/image_raw` | `sensor_msgs/msg/Image` | GZ→ROS | `gz.msgs.Image` | 30 Hz | camera sensor / `ros_gz_image image_bridge` |
| `/front_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | GZ→ROS | `gz.msgs.CameraInfo` | 30 Hz | camera sensor / `ros_gz_bridge` |
| `/lane_camera/image_raw` | `sensor_msgs/msg/Image` | GZ→ROS | `gz.msgs.Image` | 30 Hz | camera sensor / `ros_gz_image image_bridge` |
| `/lane_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | GZ→ROS | `gz.msgs.CameraInfo` | 30 Hz | camera sensor / `ros_gz_bridge` |
| `/tf`, `/tf_static` (robot) | `tf2_msgs/msg/TFMessage` | ROS | — | on change / latched | `robot_state_publisher` |
| `/robot_description` | `std_msgs/msg/String` | ROS | — | latched | `robot_state_publisher`, consumed by `ros_gz_sim create` |

The image and camera_info `frame_id` is `<camera>_optical_frame`, set through
`<camera><optical_frame_id>`. See DECISIONS D-08 for the source-code evidence.
Because image_bridge publishes through image_transport, the `compressed`,
`theora` and `zstd` variants are also available (D-12).

---

## 4. Launch graph

* `display.launch.py`: `xacro (use_sim:=false)` → `robot_state_publisher` +
  `joint_state_publisher_gui` (or the plain `joint_state_publisher` with `gui:=false`) +
  `rviz2` (`rviz/forklift.rviz`). The arguments are `gui`, `rviz`, `fork_height`, `prefix`,
  `params_file` and `rviz_config`.
* `sim.launch.py` does the following:
  1. Puts `forklift_gazebo/models` on `GZ_SIM_RESOURCE_PATH`, through the `<gazebo_ros gazebo_model_path>` export in `package.xml` that ros_gz_sim reads.
  2. Runs `ros_gz_sim gz_sim.launch.py` with `-r lift_test.sdf` and an optional `headless` arg.
  3. Starts `robot_state_publisher` with `xacro use_sim:=true`.
  4. Runs `ros_gz_sim create -topic robot_description -name forklift` at the pose given by the `x`, `y`, `z` and `yaw` args (default origin, facing +x).
  5. Starts `ros_gz_bridge` with `config/ros_gz_bridge.yaml`.
  6. Starts `ros_gz_image image_bridge` for both image topics. The topic names are read from `forklift_params.yaml`.
  7. Optionally starts RViz.

## 5. World (`lift_test.sdf`)

The world has DART physics with a 0.001 s step and RTF 1.0. It loads the
systems `Physics`, `UserCommands`, `SceneBroadcaster` and `Sensors`
(`render_engine` ogre2). It has a flat, matte grey floor; one directional light
and two spot lights, all casting shadows; and the robot at the origin facing
+x. At x = 1.5 it holds the rail pair (static) and the parcel, with the seal
facing the robot. Nothing else is in the world (D-16 option A and D-18 keep it
as specified).

## 6. Test infrastructure

* `acceptance_test.py` runs T1–T14 with one command.
  * T1 and T2 run as subprocesses (`xacro`, `check_urdf`, `gz sdf -p`,
    `colcon build`), and their output is logged under `docs/logs/`.
  * The simulation tests start their own sessions of `sim.launch.py`
    (headless EGL). Each session is its own process group, so shutdown is
    clean.

    | Session | Tests |
    |---|---|
    | A | T3, T4, T11, T14, T7, T6, T5 |
    | B | T12, T13, T8, T9, T10 |

  * One rclpy node drives `/cmd_vel` and `/fork/cmd_height`. It reads `/odom`,
    `/joint_states`, `/clock` and the camera topics.
  * Ground truth comes straight from gz-transport (`dynamic_pose/info`) through
    the installed `gz.transport13` and `gz.msgs10` Python bindings. Test-only
    plugins therefore stay out of the robot model.
* Outputs:
  * `docs/test_results.{md,json}`
  * `docs/frames_sim.pdf` and `docs/frames_display.pdf`
  * plots and camera frames in `docs/images/`
  * raw logs in `docs/logs/`
* `--params` reruns the whole suite against a parameter variant. The D-10 and
  D-20 experiments used it.

---

## 7. Operator GUI (`forklift_gui`, `forklift_bringup`)

Status: **implemented.** The owner approved the Phase 1 plan on 2026-10-15
(including D-24 and D-32). Phases 2–5 followed the plan in §7.13. Results are
in `REPORT_GUI.md` and `docs/gui_test_results.md`. The discovery data was measured on 2026-10-15
against a headless `sim.launch.py` session, using `ros2 topic list -t`,
`ros2 topic info -v` and `ros2 topic hz`. The decisions are D-22 to D-40 in
`DECISIONS.md`.

Purpose: drive the robot, command the fork and monitor its state today. Later
the same GUI observes the autonomy and lets the operator intervene. Out of
scope: CV algorithms, autonomy, the task world, and changes to the robot model.

### 7.1 Environment (verified)

| Item | Found | Note |
|---|---|---|
| ROS | Jazzy, rclcpp 28.1.22 | `ROS_DOMAIN_ID=61`, `GZ_PARTITION=sim61`. `ROS_LOCALHOST_ONLY=1` prints an rcl deprecation warning in every node. |
| Qt | **not installed.** Only Qt 5, which ROS/RViz pulls in, is present. apt candidate: **6.4.2** | System Qt only (spec §0.4). The 6.4 API limits are in §7.11. `/usr/bin/qmllint` is the Qt 5 qtchooser one. |
| twist_mux | **not installed**; candidate 4.5.0 | Defaults to `TwistStamped` (D-23) and never publishes a zero (D-24) |
| vision_msgs | 4.1.1, installed | `Detection2D{bbox{center{position{x,y},theta},size_x,size_y}, results[]{hypothesis{class_id,score},pose}, id}` |
| Lint | ament_cpplint 0.17.5 installed; **ament_clang_format not installed** | |
| xdotool | **not installed** (candidate 3.20160805) | Used by the integration tests to type into the GUI (D-39) |
| Display | X11, `DISPLAY=:1` | Screenshots need a real window; `offscreen` is for unit tests only |

Installation (the owner runs it; the agent does not use `sudo`):

```bash
sudo apt install \
  qt6-base-dev qt6-base-dev-tools qt6-declarative-dev qt6-declarative-dev-tools \
  qt6-multimedia-dev qt6-qpa-plugins \
  qml6-module-qtqml qml6-module-qtqml-models qml6-module-qtqml-workerscript \
  qml6-module-qtquick qml6-module-qtquick-controls qml6-module-qtquick-templates \
  qml6-module-qtquick-layouts qml6-module-qtquick-window qml6-module-qtquick-shapes \
  qml6-module-qtmultimedia \
  ros-jazzy-twist-mux ros-jazzy-ament-cmake-clang-format ros-jazzy-ament-clang-format \
  xdotool
```

`qml6-module-qtquick-controls` does not depend on `-templates` or `-layouts`,
so every QML module is listed explicitly. `libqt6multimedia6`, which
`qt6-multimedia-dev` pulls in, contains the gstreamer and ffmpeg backend
plugins that `VideoOutput` needs (D-34).

### 7.2 Packages

```
src/
├── forklift_gui/        ament_cmake, C++17 + QML; tree as in spec §2.3
├── forklift_bringup/    ament_cmake, launch only: launch/bringup.launch.py
└── forklift_safety/     ament_cmake, one rclcpp node `cmd_vel_gate` (D-24, approved)
    ├── config/twist_mux.yaml, config/cmd_vel_gate.yaml   (moved here, D-42)
    └── launch/control.launch.py   (started by forklift_gazebo/sim.launch.py, D-42)
```

`forklift_description` and `forklift_gazebo` are not changed. No remap of their
launch files is needed: the bridge already listens on `/cmd_vel`, and the
command chain (§7.6) ends there.

CMake targets inside `forklift_gui`:

| Target | Kind | Depends on | QML module |
|---|---|---|---|
| `forklift_gui_core` | static library | Qt Core, Gui, Qml, Multimedia, Concurrent; **no ROS** | `Forklift.Core` (C++ types only, static plugin) |
| `forklift_gui_ros` | static library | core + rclcpp, message packages | — |
| `forklift_gui_mock` | static library | core (Qt only) | — |
| `forklift_gui` | executable | all of the above | `Forklift.Gui` (all `.qml` files, embedded in qrc) |
| `test_*` | QtTest executables | core + mock | — |

The core target gets no ROS include paths, so a ROS `#include` in core fails to
compile. That is how the layer rule is enforced.

### 7.3 Layers (ports and adapters)

```
 QML view (Forklift.Gui)         Main.qml, theme/Theme.qml (singleton), components/*, panels/*
        │ binds to Backend.<view-model>.<property>, calls Q_INVOKABLEs; no logic
        ▼
 Core (forklift_gui_core)        Backend (QML_SINGLETON), TeleopController, ForkController,
  Qt only, no ROS                SafetyController, TelemetryModel, TopicHealthModel, LogModel,
                                 CameraStream, DatasetRecorder, GuiConfig, value types
        │ depends only on the port interfaces
        ▼
 Ports (core/ports)              IRobotCommandPort  IRobotStatePort  ICameraPort  ILogPort
        ▲ implemented by                                    ▲
        │                                                   │
 ROS adapter (forklift_gui_ros)                    Mock adapter (forklift_gui_mock)
  RosBridge: node + executor thread                 MockRobotAdapter: kinematic robot, fork,
  RosCommand/State/Camera/LogAdapter                synthetic frames, detections, logs, clock
  ImageConverter, RosParamLoader
        ▲
 Composition root: main.cpp. It parses args (`--mock`, `--screenshot`, `--screenshot-delay`),
 loads GuiConfig, picks the adapter, builds the view-models, sets Backend, loads Main.qml
```

Rules:

* All business logic is in C++ core: limits, ramps, deadman, the e-stop latch,
  fork target checks, reach and timeout detection, health states, throttling
  and the RTF calculation. QML only presents and forwards input (key
  press/release, joystick vector, button clicks).
* Message ↔ Qt conversion happens only in `forklift_gui_ros`.
* `QML_ELEMENT` registers the view-model types and `QML_UNCREATABLE` marks
  them. QML reaches the instances through the `Backend` singleton (D-28). No
  code uses `setContextProperty` or `qmlRegister*`.

### 7.4 Port interfaces

Each port is an abstract `QObject`, so it can have signals. A `QObject` cannot
inherit from two `QObject`s, so every adapter provides one object per port.

```cpp
// GUI → robot. Called from the GUI thread; rclcpp publishers are thread-safe.
class IRobotCommandPort : public QObject {
public:
  virtual void publishVelocity(double linear, double angular) = 0;  // /cmd_vel/teleop
  virtual void publishForkTarget(double height) = 0;                 // /fork/cmd_height
  virtual void publishEStop(bool engaged) = 0;                       // /e_stop
};
// Robot → GUI. Signals are always emitted on the GUI thread.
class IRobotStatePort : public QObject {
public:
  virtual QVector<TopicCounter> topicCounters() const = 0;  // count + last receive time, all topics
  virtual GraphStatus graphStatus() const = 0;
signals:
  void odometryReceived(const OdomSample &s);        // x, y, yaw, v, ω, stamp
  void jointStatesReceived(const JointSample &s);    // fork height, wheel ω, stamp
  void clockReceived(double simSec, qint64 recvNs);  // decimated to ≤ 30 Hz at the source
  void detectionsReceived(const DetectionSet &d);    // boxes in image pixels
  void graphStatusChanged(const GraphStatus &g);     // ROS up, peers (mock: "MOCK")
};
class ICameraPort : public QObject {
signals:
  void frameReceived(const QString &cameraId, const QVideoFrame &frame, const FrameMeta &meta);
  void cameraInfoReceived(const QString &cameraId, const QSize &resolution);
};
class ILogPort : public QObject {
public:
  virtual void logEvent(int level, const QString &text) = 0;  // GUI event → ROS logger
signals:
  void entriesReceived(const QVector<LogEntry> &batch);       // /rosout, batched
};
```

Implementations:

| Port | ROS adapter (`src/ros/RosBridge.cpp`) | Mock (`src/mock/MockRobotAdapter.cpp`) |
|---|---|---|
| command | `RosCommandAdapter`: three reliable keep-last-1 publishers | `MockCommandPort`: records every call (unit tests) and drives the mock robot |
| state | `RosStateAdapter` + `CounterRegistry` (atomics), graph polled at 1 Hz | `MockStatePort`: counters settable by tests |
| camera | `RosCameraAdapter`, frames from `ImageConverter` in the camera callback group | `MockCameraPort`, frames from `FrameWorker` on its own `QThread` |
| log | `RosLogAdapter` (`RCLCPP_*` into `/rosout`) | `MockLogPort` (records) |

The application layer (`src/app/KeyRouter`, `src/main.cpp`) is neither core
nor adapter. `KeyRouter` is a window event filter that hands keys to
`Backend::handleKey`.

### 7.5 Thread model

```
GUI thread (QGuiApplication)                    ROS thread: MultiThreadedExecutor (3 threads)
────────────────────────────────                ─────────────────────────────────────────────────
QML + core view-models                          group cam_front: Image → QVideoFrame (RGBX8888), info
Ros*Adapter QObjects  ◀─ invokeMethod(Queued) ─ group cam_lane:  Image → QVideoFrame, info
  (latest-value mailbox, ≤ 1 queued event       group state: odom, joint_states, clock (≤ 30 Hz),
   per stream; newest frame wins)                            detections
4 Hz health timer  ◀─ reads std::atomic ─────── per-topic counters (count, last receive time)
controllers ── publish() (thread-safe) ──────▶  rclcpp publishers
DatasetRecorder ─▶ QThreadPool(1): PNG + JSONL  group log: rosout (batched)
Qt Quick render thread: VideoOutput textures
```

* ROS callbacks never touch a `QObject`. Data reaches the GUI thread only
  through `QMetaObject::invokeMethod(..., Qt::QueuedConnection)` with
  value-type payloads, or through atomic counters for the health statistics
  (D-27).
* Image conversion (`sensor_msgs/Image` → `QVideoFrame`) runs in the camera's
  callback group. The GUI thread gets a finished, implicitly shared frame and
  calls `QVideoSink::setVideoFrame()`.
* Core caps the QML-facing telemetry at 30 Hz with a dirty flag and a timer,
  even when odometry arrives at 50 Hz.
* Shutdown: the Qt loop quits → the adapters detach → `executor.cancel()` →
  `join()` → the node is reset → `rclcpp::shutdown()`. `rclcpp::init` runs
  without its own signal handler. SIGINT and SIGTERM go through a self-pipe and
  a `QSocketNotifier` to `quit()`, so `ros2 launch` Ctrl-C, `kill` and closing
  the window all use the same path.
* The GUI node uses wall time (`use_sim_time: false`). Sim time comes only from
  `/clock` and the message headers (D-25).

### 7.6 Command chain and fail-safe

```
GUI ── /cmd_vel/teleop  Twist, 20 Hz while held ──┐
autonomy (future) ── /cmd_vel/auto  Twist ────────┤
                                                   ▼
GUI ── /e_stop  Bool, 10 Hz ─────────────▶ twist_mux 4.5.0   use_stamped: false (D-23)
   │                                        inputs: teleop prio 100 / auto prio 50, timeout 0.5 s
   │                                        lock:   e_stop prio 255, timeout 0.5 s
   │                                        cmd_vel_out → /cmd_vel/mux
   │                                                 ▼
   └──────────────────────────────────────▶ cmd_vel_gate (D-24) → /cmd_vel  Twist
                                                     ▼
                              ros_gz_bridge → gz DiffDrive (keeps the last command, no timeout)
GUI ── /fork/cmd_height  Float64, on change ─▶ ros_gz_bridge → JointPositionController (keeps target)
```

Measured facts (D-24): twist_mux forwards a message only inside the callback of
the input that has priority. A lock, active or expired, only masks inputs, and
twist_mux never sends a zero. DiffDrive keeps the last command forever. A
single 0.1 m/s message moved the robot 0.296 m in 3 s. So without the gate,
"e-stop" and "GUI crashed" mean the robot **keeps** its last velocity.

| Scenario | Spec chain (twist_mux → `/cmd_vel`) | With `cmd_vel_gate` |
|---|---|---|
| Deadman release, focus loss | GUI sends one zero, then goes quiet → stops | same |
| E-stop while the GUI drives | Lock masks teleop; no zero → **keeps moving** | Zero at the `/e_stop` rising edge (a few ms) |
| E-stop while the autonomy drives | **Keeps moving** | Zero at the rising edge |
| GUI killed while driving (I5) | **Keeps moving forever** | Input silent for 0.2 s → zero. DiffDrive decelerates at 1 m/s², so 0.2 m/s stops in ≈ 0.4–0.45 s (< 0.6 s) |
| GUI killed while the autonomy drives | Lock expires after 0.5 s, the last command stays → **keeps moving** | `/e_stop` stale for 0.5 s → zero |
| twist_mux dies | Keeps moving | Input silent → zero |
| Gate dies | — | Keeps moving: a single point of failure, as long as DiffDrive has no timeout (known limitation) |

Draft `forklift_bringup/config/twist_mux.yaml` (Phase 3; now `forklift_safety/config/`, D-42):

```yaml
twist_mux:
  ros__parameters:
    use_sim_time: false
    use_stamped: false            # the bridge's /cmd_vel is geometry_msgs/Twist (D-23)
    topics:
      teleop:
        topic: /cmd_vel/teleop
        timeout: 0.5
        priority: 100
      auto:
        topic: /cmd_vel/auto
        timeout: 0.5
        priority: 50
    locks:
      e_stop:
        topic: /e_stop
        timeout: 0.5
        priority: 255
# launch: remap cmd_vel_out → /cmd_vel/mux (with the gate) or /cmd_vel (without it)
```

### 7.7 ROS interface (measured on 2026-10-15)

Rates come from `ros2 topic hz` over 10 s, with 8 CLI processes running in
parallel. QoS is the publisher side, from `ros2 topic info -v`.

| Topic | Type | GUI dir. | Publisher QoS | GUI QoS (D-26) | Expected | Measured |
|---|---|---|---|---|---|---|
| `/odom` | nav_msgs/msg/Odometry | in | bridge, RELIABLE / VOLATILE | SensorData (best effort, 5) | 50 Hz | 50.0 Hz |
| `/joint_states` | sensor_msgs/msg/JointState | in | bridge, RELIABLE / VOLATILE | SensorData | 50 Hz | 50.0 Hz |
| `/clock` | rosgraph_msgs/msg/Clock | in | bridge, RELIABLE / VOLATILE | ClockQoS (best effort, 1) | 1 kHz | 999.98 Hz |
| `/front_camera/image_raw` | sensor_msgs/msg/Image (rgb8 640×480) | in | image_bridge, RELIABLE / VOLATILE | best effort, 1 | 30 Hz | 28.35 Hz* |
| `/lane_camera/image_raw` | sensor_msgs/msg/Image (rgb8 640×480) | in | image_bridge, RELIABLE / VOLATILE | best effort, 1 | 30 Hz | 25.73 Hz* |
| `/front_camera/camera_info` | sensor_msgs/msg/CameraInfo | in | bridge, RELIABLE / VOLATILE | best effort, 1 | 30 Hz | 30.30 Hz |
| `/lane_camera/camera_info` | sensor_msgs/msg/CameraInfo | in | bridge, RELIABLE / VOLATILE | best effort, 1 | 30 Hz | 30.31 Hz |
| `/rosout` | rcl_interfaces/msg/Log | in | every node, RELIABLE / TRANSIENT_LOCAL | reliable, transient local, 100 | events | — |
| `/cv/detections` | vision_msgs/msg/Detection2DArray | in, optional | not published yet | best effort, 1 | — | absent (expected) |
| `/cmd_vel/teleop` | geometry_msgs/msg/Twist | out | GUI | reliable, volatile, 1 | 20 Hz while held | 20 Hz (I2) |
| `/fork/cmd_height` | std_msgs/msg/Float64 | out | GUI (bridge subscribes RELIABLE / VOLATILE) | reliable, volatile, 1 | on change | on change (I3) |
| `/e_stop` | std_msgs/msg/Bool | out | GUI | reliable, volatile, 1 | 10 Hz always | 10.00 Hz |
| `/cmd_vel/mux` | geometry_msgs/msg/Twist | — | twist_mux (keep-last 1, reliable) | — | follows input | follows input |
| `/cmd_vel` | geometry_msgs/msg/Twist | — | gate; the bridge subscribes RELIABLE / VOLATILE | — | follows input, 10 Hz zeros when stopped | ≈ 9 Hz zeros (20 ms check grid) |

\* Counted by a Python subscriber. `camera_info` comes from the same sensor
and shows 30.3 Hz, and REPORT §7 notes the same Python under-count. I6 measures
the rate inside the C++ GUI.

Other topics on the graph: the image_transport variants of both cameras
(`/compressed`, `/compressedDepth`, `/theora`, `/zstd`), `/tf` (66.7 Hz),
`/tf_static`, `/robot_description` and `/parameter_events`. The GUI does not
use them.

### 7.8 Where reality differs from the GUI spec (Phase 1 findings)

1. **Twist type.** twist_mux 4.5.0 uses `TwistStamped` unless
   `use_stamped: false` is set. The bridge carries `Twist`. With the default,
   the robot would silently not move (D-23).
2. **The e-stop lock does not stop a moving robot.** twist_mux never publishes
   a zero, and DiffDrive keeps its last command, so I4 and I5 cannot pass with
   the spec's chain. Proposed fix: `cmd_vel_gate` (D-24). This needs the
   owner's approval because it adds a package.
3. **Qt 6 is missing, and the available version is 6.4.2.** Several APIs that
   newer Qt documentation assumes do not exist in it (§7.11).
4. twist_mux, ament_clang_format and xdotool are missing (§7.1).
5. `/cv/detections` has no publisher yet, as expected. Every other topic,
   type and rate matches spec §3.1.

### 7.9 Parameters

* `forklift_gui/config/gui_params.yaml`: the draft is in the repository. It
  holds topic names, camera ids and labels, drive limits and rates, fork limits
  and presets, e-stop timing, telemetry and trail sizes, health thresholds, log
  size, dataset root and rate, and the mock's frame size. The node declares
  every parameter **without** a default value (D-29).
* `forklift_safety/config/twist_mux.yaml` (above) and
  `forklift_safety/config/cmd_vel_gate.yaml` (moved from `forklift_bringup`, D-42), with `input_timeout 0.2 s`,
  `e_stop_timeout 0.5 s`, `zero_rate 10 Hz` and `use_sim_time false`.
* Visual constants (colours, fonts, spacing) are only in `qml/theme/Theme.qml`,
  a `pragma Singleton` file marked `QT_QML_SINGLETON_TYPE`. The numeric font
  is `monospace`, which fontconfig resolves to DejaVu Sans Mono on this machine.

### 7.10 Layout (reviewed; D-37)

I keep the spec's layout and add one item: a recording indicator in the TopBar.
Continuous recording must stay visible while the Dataset tab is hidden behind
Health or Log.

```
┌──────────────────────────────────────────────────────────────────────────────────────┐
│ [●ROS] [●SİM] sim 00:12:34  RTF 0.98   [●SAĞLIK]   [● KAYIT 42]   [F1 Yardım]  [ E-STOP [Space] ] │
├─────────────────────────────────────────────────────┬────────────────────────────────┤
│ CameraPanel   [Yan yana | Üst üste | Sekme] [+ Artı] │ TelemetryPanel                 │
│ ┌ Ön kamera ─────────────┐ ┌ Şerit kamerası ───────┐ │ x  y  yaw   v  ω   mesafe       │
│ │ DetectionOverlay        │ │                        │ │ OdomTrail (auto scale, heading)│
│ │ 30 FPS 640×480 0.03 s   │ │ 30 FPS 640×480 STALE   │ ├────────────────────────────────┤
│ └─────────────────────────┘ └────────────────────────┘ │ ForkPanel                      │
│                                                      │ ForkGauge │ slider │ [ mm ]     │
│                                                      │ [1 Yer] [2 Temas] [3 Taşıma] [4 Maks] │
├────────────────────────┬────────────────────────────┴────────────────────────────────┤
│ DrivePanel             │ [Sağlık] [Log] [Veri seti]                                   │
│ joystick │ scale %     │                                                              │
│ cmd / meas. v, ω       │                                                              │
└────────────────────────┴──────────────────────────────────────────────────────────────┘
E-stop active: 6 px red frame around the whole window; drive and fork controls are disabled.
```

* The panels sit in nested `SplitView`s. The minimum window is 1280×800.
* Considered and rejected: a "control row" with Drive and Fork side by side
  under the camera and a monitoring column on the right. It would have grouped
  the operator inputs, but it would cut the Log tab to about 400 px width and
  the fork gauge to about 180 px height.
* Every button label shows its shortcut, for example `[Space]` and `[C]`. F1
  opens an overlay that lists all shortcuts.

### 7.11 Qt 6.4.2 API facts that shape the code (verified)

Sources: the 6.4.2 apt package headers and CMake files (downloaded and
inspected without installing) and the `v6.4.2` qtmultimedia sources.

| Topic | Fact in 6.4.2 | Consequence |
|---|---|---|
| Loading QML | No `QQmlApplicationEngine::loadFromModule` (6.5) | `engine.load(QUrl("qrc:/qt/qml/Forklift/Gui/..."))` |
| `qt_add_qml_module` | Options include URI, VERSION, QML_FILES, SOURCES, RESOURCES, RESOURCE_PREFIX, DEPENDENCIES and IMPORTS. The default prefix is `/` (QTP0001 came in 6.5) | Set `RESOURCE_PREFIX /qt/qml` explicitly |
| QML singletons | C++: `QML_SINGLETON` plus an optional static `create(QQmlEngine*, QJSEngine*)` (`HasSingletonFactory`). QML file: `QT_QML_SINGLETON_TYPE` property | `Backend` returns the instance main.cpp built; `Theme.qml` uses `pragma Singleton` |
| Registration macros | `QML_ELEMENT`, `QML_NAMED_ELEMENT`, `QML_UNCREATABLE` and `QML_SINGLETON` are in `QtQmlIntegration/qqmlintegration.h` | Core links the header-only QmlIntegration |
| Delegates | No `pragma ComponentBehavior: Bound` (6.5) | Use `required property` for qmllint-clean delegates |
| QVideoFrame | Has `QVideoFrame(QVideoFrameFormat)`, `map(QVideoFrame::WriteOnly)`, `bits(plane)`, `bytesPerLine(plane)` and `Q_DECLARE_METATYPE`. No `QVideoFrame(QImage)` (6.8) | Allocate the frame, map it, copy the rows |
| Pixel formats | No 24-bit RGB; `Format_RGBX8888`, `BGRX8888`, `RGBA8888`, `BGRA8888` and `Y8` exist | rgb8 → RGBX8888 (≈ 0.3 MP copy per frame, ROS thread) |
| QVideoSink | `setVideoFrame()` is a no-op when no multimedia backend loads. The platform sink emits `videoFrameChanged` | The gstreamer plugin, in `libqt6multimedia6`, must load. Checked at start-up |
| VideoOutput | `videoSink` (CONSTANT), `fillMode` (`PreserveAspectFit`), `contentRect` and `sourceRect` | The overlay maps image pixels through `contentRect` |
| Screenshots | `QQuickWindow::grabWindow()` exists | `--screenshot <path>` |
| Effects | No `MultiEffect` (6.5). Qt5Compat effects are not installed | Plain rectangles and borders |
| qmllint 6.4 | Flags outer-id access in delegates and unknown `QPointF`/`QRectF`/`QSize` property types; cannot resolve `SplitHandle` through Controls.Basic | Workarounds in D-41 (`UiState` singleton, `QVariantList`/scalar properties, `QtQuick.Templates`) |

### 7.12 Test mapping

| Test | Where | How |
|---|---|---|
| U1–U8 | `test/core/test_{teleop,safety,fork,health,dataset}.cpp` (QtTest) | Mock adapter, the real `gui_params.yaml`, `QT_QPA_PLATFORM=offscreen`, fake clock (D-38). JUnit XML for `colcon test-result`. Extra: `test_telemetry.cpp` (30 Hz cap, RTF), `test_log.cpp` |
| gate | `forklift_safety/test/test_gate_logic.cpp` (gtest) | `GateLogic` state machine |
| I1 | `integration_test.py --tests I1` | Clean `colcon build` of the three packages with zero warnings; `colcon test` (unit tests, clang-format, cpplint, lint_cmake, xmllint, flake8, qmllint) |
| I2–I12 | `test/integration/integration_test.py` (installed; `ros2 run forklift_gui integration_test.py`) | Starts its own sim / control / GUI processes; XTEST keys and clicks into the real window; observes ROS topics, `/rosout` events, Qt logging categories and screenshots (D-39) |

### 7.13 Phase plan (done)

1. **Phase 2. Skeleton:** create the packages, CMake with `qt_add_qml_module`
   and both QML modules, the ports, the mock adapter and empty panels.
   `--mock` opens the window. Add the U-test skeletons and the lint wiring.
2. **Phase 3. Core and ROS:** write the controllers and models, `RosBridge`
   and the adapters, twist_mux plus `cmd_vel_gate`, and the `forklift_bringup`
   launch files. Run U1–U8, I1–I5, I7 and I8.
3. **Phase 4. Cameras and data:** build `CameraStream` with `VideoOutput`, the
   overlay and `DatasetRecorder`. Run I6, I9, I10 and I11.
4. **Phase 5. Polish:** theme, F1 help and screenshots. Run I12 and write
   `REPORT_GUI.md`.

---

## 8. Load stations and parcel (`forklift_gazebo`)

Status: **implemented** on 2026-10-19 (`feature/stations`). Decisions D-43 to
D-51. The robot, the GUI and the safety packages are not changed.

### 8.1 Frames

```
world
└── station (model frame)       origin on the floor, middle of the front edge; +x into the station
    ├── bay_i (SDF <frame>)     (0, y_i, 0), same axes; y_i = (i − (N−1)/2) · bay_pitch, bay 0 = most −y
    └── marker (per ID)         OpenCV ArUco frame: x right, y up, z out of the marker (−x_station);
                                origin at the black-square centre, on the front face x = 0.2985
```

At full entry the carriage front is at bay x = 0, `base_link` is at bay
x = −0.13, the fork tip is at 0.25 and the back plate starts at 0.30 (D-43).

### 8.2 Geometry (N = 3, values from `station_params.yaml`; not to scale)

```
top view: +x up (into the station), +y left; N = 3, metres
  x
 0.32 ┌──────────────────────────────────────────────────────────────────────────────┐
      │                           back plate (white) 0.02                            │
 0.30 ├────────┬────────────┬──────────┬────────────┬──────────┬────────────┬────────┤
      │ ledge  │  channel   │  ledge   │  channel   │  ledge   │  channel   │ ledge  │
      │ 0.065  │   bay 2    │   0.13   │   bay 1    │   0.13   │   bay 0    │ 0.065  │
      │        │    0.17    │          │    0.17    │          │    0.17    │        │
 0    ├────────┴────────────┴──────────┴────────────┴──────────┴────────────┴────────┤  ← station front edge (x = 0), origin at its middle
      │                     apron = base plate (station colour)                      │
-0.30 └──────────────────────────────────────────────────────────────────────────────┘
    +0.45           +0.30                     0                     −0.30          −0.45   (y; bay axes y_i)

front view: looking +x as the robot does (+y left, −y right); N = 3
  z
 0.30 ┌──────────────────────────────────────────────────────────────────────────────┐  back plate (white), z 0 … 0.30
      │             [k₂]                    [k₁]                    [k₀]             │  upper marker 0.08, centre z 0.17
      │                                                                              │
 0.05 ├────────┬────────────┬──────────┬────────────┬──────────┬────────────┬────────┤  ledge top z 0.05; lower marker 0.04, centre z 0.028
      │ ledge  │  [k₂+25]   │  ledge   │  [k₁+25]   │  ledge   │  [k₀+25]   │ ledge  │
 0    └────────┴────────────┴──────────┴────────────┴──────────┴────────────┴────────┘  base plate 0.002
    +0.45           +0.30                     0                     −0.30          −0.45   (y)
```

`[k]` is the upper marker ID (bay number k) and `[k+25]` is the lower one.
Collisions are boxes only: base plate (2 mm), N + 1 ledges (μ 0.8) and the back
plate. Markers are 1 mm boxes, visual only, 0.5 mm in front of the plate, with
a PBR `albedo_map` texture.

### 8.3 Data flow

```
station_params.yaml ─┬─► xacro/station.sdf.xacro ─┐
                     ├─► xacro/parcel.sdf.xacro ──┼─► stations.launch.py ──► ros_gz_sim create (stations, then parcels)
stations.yaml ───────┤                            │
                     └─► generate_aruco_textures.py (CMake, build time)
                            ├─► share/forklift_gazebo/models/aruco_markers/materials/textures/aruco_4x4_50_idNN.png
                            ├─► share/forklift_gazebo/models/aruco_markers/materials/textures/align_target.png (D-55)
                            └─► share/forklift_gazebo/config/markers.yaml   (every marker: id, station, bay, type, size, poses;
                                                                              align_targets: X-corner per bay side)
```

`scripts/station_layout.py` is the only code that computes `y_i`, the marker
IDs (upper = k, lower = k + 25, D-45), the marker poses and the nominal
parcel pose. The launch file, the build step and the test all import it.

### 8.4 Launch

* `stations.launch.py`: reads `stations.yaml`, validates it, expands the xacros
  with the xacro Python API and spawns each model with `ros_gz_sim create
  -string`, passing the pose through `-x -y -z -Y` (D-47). The creates are
  chained with `OnProcessExit`: all stations first, then the parcels.
* `station_sim.launch.py`: `sim.launch.py` with `worlds/<station_world>.sdf`
  (`station_test` by default, or `factory`, §9), plus `stations.launch.py` in a
  scoped group, so the two `params_file` arguments do not collide.
* `forklift_bringup/bringup.launch.py world:=lift_test|station_test|factory`: the
  default stays `lift_test`. The world file path is always passed explicitly,
  because a scoped include inherits the parent's bare `world` value.

### 8.5 Test infrastructure (`station_acceptance.py`)

* S1 and S2 work offline on the installed xacros, `markers.yaml` and textures.
* S3–S8 run in one headless session of `station_sim.launch.py control:=false`,
  in the order S3, S8, S4, S5, S6, S7. S3 is first so it measures from spawn,
  and S8 is second so the world is still undisturbed.
* Everything goes through gz-transport: `dynamic_pose/info` (ground truth),
  `stats` (sim time), the `/joint_states` gz Model message (fork joint),
  camera image topics, and `/cmd_vel` and `/fork/cmd_height` publishers. Odometry
  is not used. Service calls (`set_pose`, `create/blocking`,
  `remove/blocking`) go through the `gz service` CLI (D-49).
* Test-only models: the S5 camera (gravity off, no collision, moved with
  `set_pose`) and a static documentation camera for the S6/S7 images. Neither
  is part of the robot or of the world file.

## 9. Factory world and appearance (D-53, D-54)

### 9.1 Data flow

```
factory.yaml ─────────┐
stations.yaml ────────┼─► generate_factory_world.py (CMake) ─┬─► share/.../worlds/factory.sdf
station_params.yaml ──┘                                      └─► share/.../config/factory_lanes.yaml (lane ground truth)
station_params.yaml ──► generate_factory_textures.py (CMake) ──► share/.../models/factory_signs/materials/textures/*.png
models/factory_materials/ (CC0, committed) ──► model://factory_materials (finishes in station_params.yaml)
forklift_params.yaml → appearance ─┬─► urdf/components/appearance.xacro (visual-only parts)
                                   └─► generate_decals.py (CMake) ──► share/forklift_description/meshes/decals/*.obj
```

### 9.2 World content

* `factory.sdf` holds physics, systems, lights, the collision-only
  `ground_plane` (μ = 1.0), the Depot hall as a pinned Fuel include with
  `<experimental:params>` edits (floor collision and `fan_light` lights
  removed; `BOXSET`/`crates` removed when `clutter: clean`), one static model
  with every floor marking, one with the pillar guards and signs, and the Fuel
  props. Stations and parcels are not in the file: `stations.launch.py` spawns
  them exactly as in `station_test`, so the same tests can run in both worlds.
* World frame = `station_test` frame (robot at the origin, facing +x). The hall
  is placed around it (pose (−8.1, 0, 0, π)).
* `station_sim.launch.py station_world:=factory` passes
  `config/factory_gui.config` to `sim.launch.py gui_config:=…`, which adds
  `--gui-config` to `gz sim -g`.

### 9.3 Appearance layers

* Stations and parcels: `common.sdf.xacro` gained `finish_material` (albedo +
  normal + roughness maps) and `box_visual` (visual-only box). Collisions,
  inertia, frames and marker poses are unchanged; S2 checks them.
* Robot: `appearance.xacro` adds `<visual>` elements to `chassis_link`,
  `mast_link` and the wheel links. No link, joint or inertial is added, and all
  parts are behind the camera optical centres (x = 0.10).
* `forklift_description/package.xml` exports `gazebo_model_path="${prefix}/.."`
  so `model://forklift_description/meshes/...` resolves in Gazebo.

