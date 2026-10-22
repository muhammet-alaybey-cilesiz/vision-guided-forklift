# Design Decisions

Status values:

* **Accepted**: implemented or will be implemented as written.
* **Proposed**: default plan; the tests can still change it.
* **Open**: needs the owner's decision before the affected phase.

**Spec deviation** marks a departure from the task specification. **Unspecified**
marks a value the spec does not give, so we chose it.

Source references point to the exact installed versions: gz-sim 8.15.0,
gz-sensors 8.2.2, gz-math 7.7.0, gz-physics 7.8.0, DART 6.13.2, sdformat 14.9.0
and ros_gz 1.0.24.

---

## D-01 Workspace and documentation location (Accepted)
The workspace is `~/cv_forklift_ws`, as the spec says, not the folder open in
the editor (`~/otomasyon_ws`). `ARCHITECTURE.md`, `DECISIONS.md`, `REPORT.md`
and `docs/` (view_frames PDF, `docs/images/`) are at the workspace root.

## D-02 Single source of truth and derived quantities (Accepted)
* Every robot number lives in `forklift_description/config/forklift_params.yaml`.
  The xacro files contain only formula coefficients and the ±π/2 optical-frame
  convention.
* The YAML stores each value once, in the convention the spec uses. The macros
  derive the rest:
  * base_link height = wheel radius
  * wheel y = ±separation/2
  * caster z = r_caster − r_wheel
  * mast top = mast bottom + upright height
  * fork centre = x_min + length/2
* Angles are in radians (REP-103), with degrees in comments.
* The Gazebo-side topic names are also in the YAML, so `forklift.gazebo.xacro`
  holds no literals except plugin identifiers.
* World objects (floor, rails, parcel) are not robot parameters. Their SDF files
  are their source. `acceptance_test.py` parses those files instead of copying
  their numbers.

## D-03 Link structure and fixed-joint lumping (Accepted)
* `base_footprint` is the root. It has no inertial, visual or collision, so
  there is no KDL root-inertia warning.
* `base_link` is a massless frame at the drive-axle centre. The chassis box is
  in `chassis_link`, which the spec lists as a fixed link.
* `caster_link` and `mast_link` are children of `base_link`, because the spec
  gives their positions relative to `base_link`.
* The `mast_link` origin is the mast base centre, (0.095, 0, −0.03) relative to
  `base_link`. This keeps the carriage and camera offsets short.
* When sdformat converts URDF to SDF it lumps fixed joints. Gazebo then sees four
  links and three joints:
  * links: `base_footprint` (9.3 kg), the two wheels, and `fork_carriage_link`
    (plate and forks, 0.9 kg)
  * joints: two wheel joints and `fork_lift_joint`
* sdformat carries two things from lumped links over to the merged link:
  * per-collision surface extensions such as μ, matched by the lumped
    collision name (parser_urdf.cc:1652–1673)
  * sensor poses, transformed by the reduction transform
    (parser_urdf.cc:2600–2611)

  T1 inspects both in the converted SDF.
* **URDF `<collision>` elements stay unnamed. This is required, not just style.**
  * sdformat names a lumped collision
    `<parent>_fixed_joint_lump__<name>_collision[_N]`, where `<name>` is the
    collision's own name, or the link name if it has none
    (parser_urdf.cc:2899–2915, 3207–3213).
  * It applies a `<gazebo reference="<link>">` μ only when the name contains
    `_fixed_joint_lump__<link>_collision`.
  * A custom name such as `caster_collision` would therefore make the caster's
    μ = 0 silently disappear.

## D-04 Mast inertial (Accepted)
The mast is one link made of three boxes. The spec gives only the total of
1.0 kg, so the mass is split by volume:

| Part | Mass |
|---|---|
| each upright | 0.4065 kg |
| crossbar | 0.1870 kg |

The inertia about the composite COM comes from the box formula plus a
parallel-axis composite macro. No values are typed in by hand. The composite
COM is at ground z = 0.314 m.

## D-05 Crossbar height (Accepted. The spec is ambiguous: it says "üstte", on top, with no z)
The crossbar's top face is flush with the upright tops, at ground
z ∈ [0.49, 0.52]. If the bar sat on top of the uprights instead
(z ∈ [0.52, 0.55]), the lane camera's optical centre, at x = 0.10 and z = 0.54,
would be inside the bar, whose x range is [0.08, 0.11].

## D-06 Camera clip planes (Accepted, Unspecified)
* Clip planes are near 0.05 m and far 30 m for both cameras.
* The lane camera sits 2 cm above the crossbar. Its lower FOV edge is 77.2°
  below the horizontal, so it would see the front 5 mm of the bar top from about
  2.2 cm away. That fills image rows 367–480, about 23 % of the frame.
* With near = 0.05 m the bar is clipped. This is a simulation simplification.
* Alternative, which is a spec deviation: move the lane camera to x = 0.11. The
  bar then leaves the FOV with no clipping trick, and the parcel clearance in
  D-17 grows from 1.5 cm to 2.5 cm.
* Owner decision (2026-10-14): near clip. The camera stays at x = 0.10, as
  specified.
* Verified: the crossbar does not appear in the unloaded lane frame (`docs/images/t12_lane_camera_unloaded.png`). Only the fork tips are visible at the bottom.

## D-07 Camera body (Accepted, Unspecified)
* The body is a box of 0.02 × 0.06 × 0.025 m with a mass of 0.05 kg (the spec
  value).
* The optical centre is on the body's front face, so the camera never renders
  from inside its own body.
* No brackets are modelled, so both cameras appear to float. This is cosmetic.
* Approved by the owner (2026-10-14).

## D-08 Camera `frame_id` (Accepted)
* Each camera uses `<camera><optical_frame_id><name>_optical_frame</optical_frame_id>`
  and an explicit `<camera_info_topic>`. It does not use `<gz_frame_id>`.
* Evidence from gz-sensors 8.2.2:
  * The image header uses `opticalFrameId` (CameraSensor.cc:575–576).
  * The camera_info header uses `opticalFrameId` too. It takes the value of
    `<optical_frame_id>`, falling back to `FrameId()` (CameraSensor.cc:776–788).
  * `<gz_frame_id>` only sets `FrameId()` (Sensor.cc:188).
* `<optical_frame_id>` and `<camera_info_topic>` are in the SDF 1.11 schema
  (camera.sdf:225, :12). `<gz_frame_id>` is not in sensor.sdf.
* T11 checks this at runtime.

## D-09 DiffDrive velocity and acceleration limits (Accepted, Unspecified)
* Limits:

  | | Velocity | Acceleration |
  |---|---|---|
  | linear | ±0.5 m/s | ±1.0 m/s² |
  | angular | ±1.5 rad/s | ±3.0 rad/s² |

* Loaded, the COM is at x = −0.054 m and z = 0.117 m. The caster therefore
  unloads above g·0.054/0.117 ≈ 4.5 m/s² of braking (6.0 m/s² unloaded).
* Without a limiter, a 0.3 → 0 m/s step is bounded only by wheel friction:
  about 13.6 m/s² with μ = 1. T10 would fail.
* 1.0 m/s² leaves a 4.5× margin. It stops the robot from 0.3 m/s in 0.3 s over
  4.5 cm.
* The parameter names are in DiffDrive.cc:216–320. In gz-math SpeedLimiter.cc:152–170
  the min limit is the negative bound on dv/dt.
* Consequence: in T10, "sudden brake" means a step command. The robot then
  brakes at the configured limit.
* Approved by the owner (2026-10-14).

## D-10 Fork controller mode (Accepted: PID force mode, Measured)
**Decision:** `use_velocity_commands: false`, i.e. PID force mode.

| Parameter | Value |
|---|---|
| P | 10 kN/m |
| I | 2 kN/(m·s) |
| D | 130 N·s/m |
| integral clamp | ±15 N |
| cmd_max/cmd_min | ±200 N, the joint effort limit |

**Why velocity mode was dropped (measured):**
* In velocity mode the controller sends SERVO commands
  (JointPositionController.cc:438–466; gz-physics JointFeatures.cc:162–184).
* DART 6.13.2 `JointConstraint::update()` handles a DOF that violates its
  position limit (`B1 < 0`) with the limit branch and then `continue`s. It
  never reaches the servo branch (JointConstraint.cpp).
* Under gravity the carriage rests on its lower limit at q ≈ −1.3e-14 m. Every
  step it is therefore "below the limit", the servo command is ignored, and the
  fork can never leave 0.
* Measured with the spec limit [0, 0.15]: in T7 the fork stayed at 0 for the
  0.05 m and 0.10 m targets.
* Control experiment: with the lower limit moved to −0.01 m, velocity mode was
  perfect. It moved exactly 0.05 m/s, reached ±5 mm in 0.91 s / 0.91 s / 1.91 s,
  with zero overshoot. That confirms the mechanism.
* Moving the limit would have been a spec deviation, and the fork could then be
  commanded below its rest height.

**Why PID mode is safe at the limit:**
* The force command is a generalised force. The lower-limit constraint can only
  push up (impulse ≥ 0), so lifting off the stop works.
* DART still enforces the 0.05 m/s joint velocity limit. Limits are enforced
  (gz-physics SDFFeatures.cc:144), and the velocity bounds sit in the same
  enforced block of JointConstraint.

**Measured with the chosen gains (unloaded, T7):** 0.91 s to ±5 mm,
overshoot 0.64 mm, final error 0.34 mm. T9 measures the loaded hold.

**Tuning rationale:**
* With P alone the static error is g·m/P: 0.9 mm unloaded, 1.4 mm loaded.
* The integral removes that error. Its ±15 N clamp covers the loaded carriage
  weight of 13.7 N and limits windup overshoot to below 1 mm.
* ζ ≈ 0.7 with D = 130. ω_n·dt ≈ 0.1, which is stable at the 1 ms step.

## D-11 JointStatePublisher rate (Accepted, Unspecified)
`update_rate` is 50 Hz. The default of 0 publishes every physics step, which is
1 kHz (JointStatePublisher.cc:107–118). Approved by the owner (2026-10-14).

## D-12 Image bridging (Accepted)
* `image_raw` goes through `ros_gz_image image_bridge`. It publishes through
  image_transport, and the compressed, theora and zstd plugins are installed.
  Encoded variants are therefore available on demand for rosbag or remote CV
  work, and QoS can be selected.
* `camera_info` and all other topics go through `ros_gz_bridge` with a YAML
  config.
* The raw conversion cost is the same in both bridges. The choice is about
  transport features. T11 and T14 confirm the rates.

## D-13 Ground truth for tests (Accepted, Verified)
* `acceptance_test.py` reads robot and parcel world poses from gz-transport
  (`/world/lift_test/dynamic_pose/info`, `gz.msgs.Pose_V`). It uses the installed
  `gz.transport13`/`gz.msgs10` Python bindings.
* No test-only plugins go into the robot and no extra ROS topics are created.
* Verified in Phase 3: about 59 Hz. Model poses are in the world frame, link poses relative to their model, and every message carries a sim-time stamp (SceneBroadcaster.cc:523–588).

## D-14 Wheel joint limits (Accepted, Unspecified)
* Effort is 5 N·m and velocity 20 rad/s (≈ 1 m/s). Damping and friction are 0.
* Wheel–floor friction caps the usable torque at μ·N·r ≈ 2.1–2.3 N·m per wheel
  (static load, unloaded and loaded), so the effort limit never binds in normal
  use.
* The limits make the URDF ready for ros2_control.
* Approved by the owner (2026-10-14).

## D-15 Friction combination and floor friction (Accepted)
* DART combines two surfaces as min(μA, μB) (DART 6.13.2 ContactSurface.cpp:114).
  gz-physics keeps that rule (SimulationFeatures.cc:333–341).
* The floor has μ = 1.0. Wheel/floor is therefore 1.0 and caster/floor 0.0, as
  specified.
* The parcel has μ = 1.0, so the fork value (1.0) governs parcel/fork contact.

## D-16 T8 scenario (Accepted: option A, Spec deviation)
The scenario cannot run as written, for two independent reasons:

1. **"Drive 1 m forward" after pickup is blocked.**
   * With the carriage 1 cm from the parcel, base_link is at x = 1.26 and the
     mast front at 1.37. The rails start at 1.40.
   * The mast uprights (|y| 0.085–0.115, bottom z 0.02) and the chassis
     (|y| ≤ 0.15, bottom z 0.02) overlap the rails (|y| 0.08–0.11, top z 0.05).
   * Free travel is therefore 3 cm. Turning in place at the pickup also sweeps
     the chassis and wheels into the rails.
2. **Lowering on the flat floor cannot release the parcel.** At the lower
   limit the fork top is at z = 0.025 m. The parcel stays on the forks and is
   dragged along during the 0.3 m reverse. Release needs rails (fork pockets)
   under the parcel.

Options:
* **A (recommended)**: keep the world. Approach, lift 0.06 m, reverse 1 m, turn
  +90°, turn −90°, drive forward 1 m, lower onto the same rails, then reverse
  0.3 m.
* **B**: add a second rail pair (a drop-off station) at (0.26, 0.74) facing +y.
  Approach, lift, reverse 1 m, turn +90°, drive forward 0.5 m, lower, then
  reverse 0.3 m. This breaks "no other objects" in the world.

**Owner decision (2026-10-14): option A.** The world stays as specified. The
T8 sequence becomes: approach → lift 0.06 m → reverse 1 m → +90° → −90° →
forward 1 m → lower onto the same rails → reverse 0.3 m. The criteria are
unchanged: relative slip < 1 cm while carrying, no drop, and the parcel upright
on the rails afterwards.

## D-17 Approach stop distance (Accepted, Verified)
* The approach stops with the carriage 1 cm before the parcel face, at
  base_link x = 1.26 m.
* The lane camera's central ray clears the carried parcel's far top edge by:

  | Carriage–parcel gap | Clearance |
  |---|---|
  | 0 cm | 2.5 cm |
  | 1 cm | 1.5 cm |
  | 2.5 cm (parcel centred on the forks) | 0 cm |

* The camera geometry in §2.3 of the spec therefore assumes the parcel sits
  against the carriage.
* Verified in T8 and T12 with the approach stopping at x = 1.2606–1.2608 m
  (target 1.260):
  * In the loaded lane frame the parcel's top edge on the centre column is at
    row 252; the 21×21 centre window is 100 % floor.
  * The predicted edge row was 252 (Phase 1).

## D-18 Measuring T10 caster contact and the T13 "sim" COM (Accepted: no world change)
**Owner decision (2026-10-14):** no Contact system and no contact sensor. The
world keeps the four systems from the spec.
* **T10:** from ground truth, the caster sphere's lowest point must stay within
  1 mm of the floor during braking. The base pitch is reported too. The caster
  centre comes from the `base_footprint` link pose (the caster is lumped into
  it) plus the caster offset from the YAML.
* **T13 sim:** take the mass properties of the SDF that gz actually loads (the
  lumped links). Place them with the live link poses and the live parcel pose
  from gz, then compute the COM and compare it with the analytic value from the
  YAML.
* Rejected: contact sensor plus the `Contact` system. It would give a direct
  measurement (Contact.cc:200–222, 310–313), but it would add a fifth world
  system.

## D-19 T10 runs inside the T8 transport (Accepted)
* T10 needs a loaded robot braking from **forward** motion. Braking while
  reversing moves load onto the caster instead of off it, so it would not test
  the tipping risk.
* After the +90° turn of the T8 sequence the robot is at x ≈ 0.26 m, facing +y,
  in free space. The script then does the following:
  1. drives forward at 0.3 m/s for 0.25 m (the robot reaches a steady 0.3 m/s
     after 0.045 m);
  2. commands a step to zero and waits for the robot to stop;
  3. reverses the same distance before the −90° turn.
* T8's slip criterion therefore also covers the braking event.
* Caster contact is judged from ground truth (D-18). The caster's lowest point
  must stay within 1 mm of its pre-brake height, and |Δpitch| < 1°.

## D-20 Drive-wheel collision shape: sphere (Accepted, Spec deviation, Measured)
* The collision is a sphere of r = 0.05 m at the wheel centre. The visual and
  the inertia stay the specified cylinder (r 0.05, width 0.03, 0.5 kg).
* With the 3 cm cylinder collision, the in-place 360° turn (T6) was 392.8° in
  ground truth against 360.0° in odometry. That is 9.1 % too much rotation, an
  effective track of 0.312 m, as if the wheels touched at their inner edges
  (y = ±0.155).
* The cylinder touches the plane at its two rim points. During a turn those
  points scrub against each other with different lever arms, and the solver
  ends up pivoting on the inner edge.
* Straight driving is unaffected (T5: 1.0002 m in both odometry and ground
  truth), because both rim points roll at the same speed.
* With the sphere the contact is a single point under the wheel centre.
  Measured: T6 odometry 360.02°, ground truth 360.01°.
* The sphere reaches ±5 cm sideways (|y| 0.12–0.22 m). In the test world nothing
  is that close to the wheels.

## D-21 Acceptance-test method (Accepted)
* **Sessions:** the script launches `sim.launch.py` itself (headless EGL) in two
  fresh sessions:

  | Session | Tests |
  |---|---|
  | A | T3, T4, T11, T14, T7, T6, T5 |
  | B | T12, T13, T8, T9, T10 |

  Each session is its own process group, so shutdown also stops gz sim and the
  bridges.
* **T3:** the first 10 s of sim time after the model's first ground-truth
  sample (the spawn).
* **Spawn:** z = 0, so the wheels and caster are tangent to the floor.
* **Ground truth:** `dynamic_pose/info` at about 60 Hz (the SceneBroadcaster
  default).
* **Open-loop moves:** odometry is used for distance and yaw. The stop command
  is issued v²/(2a) early, using the D-09 limits.
* **T11:** the rate is counted by the test's own rclpy subscriber (best-effort
  QoS) against the wall clock, so it is a lower bound of the publish rate.
* **T14:** measured on the headless server. The rate with the GUI is reported
  separately.
* **T12, seal visible:** more than 500 red pixels in the front frame.
* **T12, centre region:** the central 21×21 px window of the lane image is at
  least 90 % floor (low-saturation) pixels.
* **T13 sim:** gz's lumped SDF masses placed with live link and parcel poses.

---

# Operator GUI (`forklift_gui`, `forklift_bringup`)

These decisions come from Phase 1 (discovery and plan), on 2026-10-15. The
owner approved the whole plan the same day, so they are all Accepted. They
were checked against the installed or apt-candidate versions: Qt 6.4.2 (Ubuntu
24.04 packages, whose headers and CMake files were inspected before
installation), twist_mux 4.5.0 (package headers and the `4.5.0` tag sources),
gz-sim 8.15.0, rclcpp 28.1.22 and vision_msgs 4.1.1. ARCHITECTURE §7 holds the
diagrams.

## D-22 Toolkit: system Qt 6.4.2 (Accepted)
The GUI uses the Ubuntu 24.04 apt Qt, version 6.4.2: qt6-base-dev
6.4.2+dfsg-21.1build5, qt6-declarative-dev 6.4.2+dfsg-4build3 and
qt6-multimedia-dev 6.4.2-11build3. The spec forbids pip and online-installer Qt
(§0.4), and no Qt 6 is installed yet. Version 6.4 is older than most current
documentation. These limits were confirmed in the package headers and CMake
macros, and they shape the code (ARCHITECTURE §7.11):

* no `loadFromModule`
* the default `RESOURCE_PREFIX` is `/`
* no `pragma ComponentBehavior`
* no `QVideoFrame(QImage)` constructor and no 24-bit RGB video format
* no `MultiEffect`

## D-23 twist_mux uses unstamped Twist: `use_stamped: false` (Accepted, Verified in source)
twist_mux 4.5.0 publishes and subscribes `TwistStamped` unless `use_stamped` is
set to false.

* `TwistMux::init()` starts from `bool use_stamped = true`. When the parameter
  is missing it only logs "use_stamped is not declared as parameter,
  defaulting to true" (twist_mux.cpp at tag 4.5.0; the same string is in the
  installed binary).
* The `/cmd_vel` bridge is `geometry_msgs/msg/Twist` ↔ `gz.msgs.Twist`
  (ros_gz_bridge.yaml). `ros2 topic list -t` confirmed this on the live
  system.

With the default setting, the bridge's Twist subscription and twist_mux's
TwistStamped publisher would share the topic without matching, and the robot
would not move and would report no error. `forklift_bringup/config/twist_mux.yaml`
therefore sets `use_stamped: false`, and the GUI publishes plain
`geometry_msgs/msg/Twist` on `/cmd_vel/teleop`. The node declares its
parameters from overrides, so `get_parameter()` sees the YAML value.

## D-24 Fail-safe stop: `cmd_vel_gate` after twist_mux (Accepted, Spec deviation)
Spec §3.3 assumes that a stale `/e_stop` lock stops the robot. On this stack it
does not.

Verified behaviour:

* **twist_mux** forwards a message only inside the callback of the input that
  currently has priority (topic_handle.hpp `VelocityTopicHandle::callback`,
  twist_mux.cpp `hasPriority`). A lock, active or expired
  (`isLocked() = hasExpired() || data`), only masks inputs. Nothing is
  published on a lock or a timeout, and no zero is ever generated.
* **gz-sim 8.15.0 DiffDrive** stores each command (`OnCmdVel`,
  DiffDrive.cc:637–642) and applies it every step (:614). The file has no
  command timeout.
* **Experiment (2026-10-15, headless sim):** one `/cmd_vel` message of 0.1 m/s,
  then silence. The robot held 0.100 m/s and covered 0.296 m in 3 s. It stopped
  only on an explicit zero.

So in the spec's chain, an e-stop or a GUI crash freezes the last command
instead of stopping the robot, and I4 and I5 would fail.

**Proposal:** a small rclcpp node, `cmd_vel_gate`, in a new package
`forklift_safety`. twist_mux's output is remapped to `/cmd_vel/mux`, and the
gate sits between it and `/cmd_vel`. The gate:

* forwards its input while `/e_stop` is false
* publishes a zero Twist at once on a rising edge of `/e_stop`
* republishes the zero at 10 Hz while `/e_stop` is true, `/e_stop` is older
  than 0.5 s, or the input is older than 0.2 s (all wall-clock ages)

twist_mux keeps arbitration and the lock, so the e-stop still blocks the future
autonomy as the spec requires. The gate turns "no input allowed" into "zero
commanded".

Alternatives considered:

| Alternative | Why rejected |
|---|---|
| The GUI publishes a zero before it locks | Does not cover a crash, and depends on message ordering |
| nav2_velocity_smoother `velocity_timeout` | Brings nav2 and a lifecycle manager for one timeout; not installed |
| A DiffDrive timeout | The package must not be changed, and gz-sim 8 has no such option |

The spec's package list does not include this node, and `forklift_bringup` must
stay config and launch only. **Owner decision (2026-10-15): approved**, as
proposed (new package `forklift_safety`).

## D-25 Wall clock for safety timing (Accepted)
twist_mux, `cmd_vel_gate` and the GUI node run with `use_sim_time: false`.
Reasons:

* The GUI's 20 Hz teleop and 10 Hz e-stop timers must keep running before the
  sim starts and while it is paused.
* I5 is a wall-clock criterion.
* twist_mux takes the age of each message as `now().seconds() -
  stamp_.seconds()` (topic_handle.hpp `hasExpired`), so its initial
  system-time `stamp_(0)` cannot raise a clock-type mismatch. With sim time,
  however, the e-stop lock would count as *not* expired before the first
  `/clock` (now = 0), which fails open.

Sim time is used only for display, RTF and dataset metadata. It is read from
`/clock` and from the message headers.

## D-26 QoS (Accepted)
Measured on 2026-10-15 with `ros2 topic info -v`: every bridge publisher is
RELIABLE/VOLATILE, and every `/rosout` publisher is RELIABLE/TRANSIENT_LOCAL.

| GUI endpoint | QoS | Why |
|---|---|---|
| `/odom`, `/joint_states`, `/cv/detections` subscriptions | SensorDataQoS (best effort, depth 5) | Best effort matches both reliable and best-effort publishers, so a future CV node may use either |
| Image and `camera_info` subscriptions | best effort, depth 1 | Only the newest frame matters, and depth 1 prevents queueing latency |
| `/clock` subscription | `rclcpp::ClockQoS()` (best effort, depth 1) | Only the latest time matters |
| `/rosout` subscription | reliable, transient local, depth 100 | Recent logs of nodes started before the GUI also appear |
| `/cmd_vel/teleop`, `/fork/cmd_height`, `/e_stop` publishers | reliable, volatile, keep-last 1 | twist_mux and the bridge subscribe reliable/volatile; with depth 1 a slow reader gets the newest command, never a backlog |

Transient local on `/fork/cmd_height` was rejected: the bridge subscribes
volatile, so it would never receive the stored history.

## D-27 Threads and cross-thread data (Accepted)
**Executor.** One rclcpp node runs in a `MultiThreadedExecutor` with 3 threads
(a parameter) in a `std::thread`. It has four callback groups, so image
conversion never delays odometry:

* one mutually exclusive group per camera (image and info)
* one group for state: odom, joints, clock and detections
* one group for `/rosout`

**Data to the GUI thread.** ROS callbacks never touch a `QObject`. Data crosses
in only two ways:

1. Value-type payloads go through `QMetaObject::invokeMethod(...,
   Qt::QueuedConnection)`. A latest-value mailbox with a pending flag keeps at
   most one queued event per stream:
   * the newest image frame replaces an older one that is still waiting
   * `/clock` is decimated to at most 30 Hz at the source
   * `/rosout` messages are batched
2. Per-topic `std::atomic` counters (count and last receive time) are read by a
   4 Hz timer on the GUI thread for the health model. Posting one event per
   message would mean 1 kHz events for `/clock` alone.

**Throttling.** Core coalesces the QML-facing telemetry to at most 30 Hz with a
dirty flag and a timer. Because this happens in core, the mock adapter and the
unit tests exercise the same throttle as the ROS adapter.

**Publishing and shutdown.** Publishers are called from the GUI thread; rclcpp
publishers are thread-safe. ARCHITECTURE §7.5 gives the shutdown order and the
signal path (SignalHandlerOptions::None, then a self-pipe and a
`QSocketNotifier`).

## D-28 QML exposure and module structure (Accepted)
`setContextProperty` and `qmlRegister*` are excluded, and the view-models need
their ports injected. So:

* **Composition.** main.cpp builds the adapters and the view-models, then hands
  them to one `Backend` object. `Backend` is a `QML_SINGLETON` whose static
  `create()` returns that pre-built instance with `QJSEngine::CppOwnership`
  (6.4 supports this through qqmlprivate.h `HasSingletonFactory`). It exposes
  each view-model as a `CONSTANT` property.
* **View-model types** are `QML_ELEMENT` and `QML_UNCREATABLE`, so QML and
  qmllint know them.
* **Two QML modules.** `Forklift.Core` is backed by the `forklift_gui_core`
  static library and its generated static plugin, and contains C++ types only.
  `Forklift.Gui` is backed by the executable and contains every `.qml` file;
  `Theme.qml` is a `pragma Singleton` file marked with
  `QT_QML_SINGLETON_TYPE`.

Unit tests link only `forklift_gui_core`.

Build detail (verified in Phase 2): Qt 6.4's `qmltyperegistrar` includes the
registered headers by file name only (`#include <Backend.hpp>`), so
`include/forklift_gui/core` is a PRIVATE include directory of the core target.
The executable imports the static plugin with
`Q_IMPORT_QML_PLUGIN(Forklift_CorePlugin)` and adds `qrc:/qt/qml` to the import
path, which 6.4 does not do by default.

## D-29 One parameter file, no defaults in code (Accepted)
`forklift_gui/config/gui_params.yaml` is the only source of topic names,
limits, rates, presets and thresholds. The node declares every parameter
without a default value. A missing key therefore stops start-up and names the
parameter, instead of letting a value from the code take its place. In `--mock`
mode there is no node: main.cpp reads the same file with
`rclcpp::parameter_map_from_yaml_file()` (rclcpp/parameter_map.hpp), which
parses YAML and starts no ROS graph. Both paths fill the same plain `GuiConfig`
struct. Camera and preset labels are configuration data and are shown as
given; static UI text is wrapped in `qsTr()`.

## D-30 Drive semantics (Accepted)
**Ramp.** While the deadman is held, every change of the command is
rate-limited by `max_linear_accel` / `max_angular_accel`: speeding up, slowing
down and reversing.

**Safety stops.** Deadman release, loss of window focus and e-stop each send
one zero command at once, after which publishing stops. That stop is not
ramped. It is limited only by DiffDrive's 1.0 m/s² (D-09), and T10 showed that
braking safe from 0.3 m/s with a load. On focus loss all held keys are
cleared, so no key is left "stuck" after alt-tab.

**Inputs.** Keys give axis values of ±1: W/S for linear, A/D for angular. A is
+ω (REP-103), also when reversing. The joystick gives continuous values in the
unit disc. If a key is held, the keyboard wins.

**Limits.** The GUI limits (0.3 m/s, 1.0 rad/s, 0.5 m/s², 2.0 rad/s²) are
inside DiffDrive's (D-09).

## D-31 E-stop interaction (Accepted)
* **Latch and heartbeat.** The e-stop is latched. The application starts
  engaged and publishes `/e_stop` at 10 Hz from start-up, whether true or false.
* **Engage.** The button or Space engages it. The new state is published
  immediately, not at the next heartbeat.
* **Release.** Press and hold the button for `safety.release_hold` (1.5 s)
  while a progress ring fills. Space never releases.

A hold was chosen over a confirmation dialog for three reasons: it needs no
modal dialog that captures keyboard focus, a stray Enter cannot confirm it, and
the tests can drive it.

Additions from Phase 3:

* **Space always works.** A window-level event filter (`KeyRouter`) sends Space
  to `Backend::handleKey` before any item sees it, so a focused button or text
  field cannot swallow the e-stop. While a text field has focus the other
  shortcuts are left to the field, but releases of W/A/S/D are still forwarded,
  so no key stays "held".
* **Orderly shutdown engages the e-stop.** Window close, SIGINT and SIGTERM call
  `Backend::stop()`, which engages the e-stop first: one zero teleop command,
  the fork hold and `/e_stop=true` go out at once, instead of waiting for the
  twist_mux lock timeout (0.5 s) or the gate's input timeout (0.2 s).

## D-32 Fork under e-stop (Accepted)
The JointPositionController also keeps its last target. Without action, a fork
that is moving when the e-stop is pressed finishes its move (at most 3 s at
0.05 m/s).

**Decision (owner, 2026-10-15):** on engage, send one "hold at the measured
height" command, then send no fork command until release.

**Alternative (strict reading of §4.2):** send nothing.

Either way the fork has no fail-safe path through twist_mux. If the GUI
crashes, the last fork target stays active (known limitation).

## D-33 Fork panel semantics (Accepted)
* Targets are clamped to `[min_height, max_height]`.
* A command is published only when the target changes.
* "Reached" means |measured − target| ≤ 5 mm.
* The 3 s timeout starts at each target change. A new target or reaching the
  target clears the warning.
* After a (re)connection, which means the first `/joint_states` after a gap,
  the target is reset to the measured height and nothing is published. This
  keeps the GUI from pushing a stale target into a sim that just restarted.

The vertical slider sets the target on release, not on every step while it is
dragged, so one drag publishes one command.

## D-34 Camera pipeline (Accepted, Verified in source)
`QVideoSink` with a QML `VideoOutput` is kept.

* **Backend requirement.** In Qt 6.4.2, `QVideoSink::setVideoFrame()` does
  nothing if no multimedia backend loads (qvideosink.cpp).
  `libqt6multimedia6` ships `libgstreamermediaplugin.so` and
  `libffmpegmediaplugin.so`, and the default choice is gstreamer
  (qplatformmediaintegration.cpp). Its sink emits `videoFrameChanged`
  (qplatformvideosink_p.h), which `VideoOutput` consumes.
* **Conversion,** in the camera callback:

  | ROS encoding | Video format |
  |---|---|
  | `rgb8` | `Format_RGBX8888` |
  | `bgr8` | `Format_BGRX8888` |
  | `rgba8`, `bgra8` | `Format_RGBA8888`, `Format_BGRA8888` |
  | `mono8` | `Format_Y8` |

  Any other encoding is counted, dropped and reported once.
* **Overlay.** It maps image pixels to the item through
  `VideoOutput.contentRect`, which accounts for the aspect-fit letterbox.

If the backend fails at run time in Phase 4, the fallback is a custom
`QQuickItem` that uploads a `QSGTexture`. That decision would be recorded then.

**Verified in Phase 4** (desktop session, Qt 6.4.2): `qt.multimedia.plugin`
reports the backends `gstreamer` and `ffmpeg` and loads `gstreamer`;
`VideoOutput` creates its video node and shows both cameras at 30 FPS in mock
mode and in the simulation. No fallback was needed. The FPS, frame age and STALE
statistics are refreshed at `health.update_rate` (4 Hz), which is enough for a
1 s STALE threshold and keeps the numbers readable.

## D-35 Health model (Accepted)
Each topic moves through these states:

| State | Condition |
|---|---|
| Grey ("waiting") | no message received yet |
| Yellow | measured rate below `warn_rate_ratio` (0.8) × expected, or last message older than `warn_age` |
| Red | last message older than `error_age` |

`/rosout` and `/cv/detections` are event topics. They have no rate check, and
their absence is not an error.

TopBar summary LEDs:

* **ROS:** green when the node is up and at least one other node is
  discovered; yellow when the node is alone. Mock mode shows a blue "MOCK"
  instead.
* **SİM:** follows the `/clock` health.
* **SAĞLIK:** the worst state among the required topics.

All values are in gui_params.yaml.

## D-36 Dataset capture (Accepted)
* **Pair.** A capture takes the newest frame of each camera. If either frame is
  missing or STALE (older than 1 s), the capture is skipped and logged.
* **Metadata.** Each JSONL line stores both frames' header stamps, so the time
  skew within the pair is known.
* **Writing.** A one-thread `QThreadPool` does the writes, so PNGs and JSONL
  lines stay in order and the GUI thread never blocks. PNGs are RGB888, so the
  RGBX padding is dropped.
* **Layout:** `<root>/<YYYYmmdd_HHMMSS>/{front,lane}/NNNNNN.png` plus
  `metadata.jsonl`.

## D-37 Layout (Accepted)
The spec's layout is kept, with a TopBar recording indicator added
(ARCHITECTURE §7.10). The indicator matters because continuous recording must
stay visible while the Dataset tab is hidden.

A variant was rejected. It had Drive and Fork side by side under the camera and
a monitoring column on the right. It would have narrowed the Log tab to about
400 px and the fork gauge to about 180 px.

## D-38 Mock adapter and deterministic test time (Accepted)
The mock produces:

* unicycle motion with the DiffDrive limits
* a fork that moves at 0.05 m/s
* synthetic 640×480 frames at 30 Hz, made on a worker thread so the mock uses
  the same cross-thread path as the ROS adapter
* synthetic detections, rosout lines and a clock at RTF 1

Controllers run on an explicit `update(now)`. A QTimer calls it in the
application, and the tests call it directly, so U2, U3, U6 and U7 are
deterministic and need no sleeps.

The mock clock runs at 1 kHz like the simulation's `/clock`, so the health model
(which expects 1 kHz) shows it green; mock frames are BGRX (exercising the
`bgr8` path of the dataset writer).

## D-39 Integration test method (Accepted)
**Input.** Integration tests drive the real GUI process with X11 keyboard
input (`xdotool`), the same key path an operator uses.

**Observation channels:**

* **ROS topics:** `/cmd_vel*`, `/e_stop`, `/odom`, `/joint_states`.
* **`/rosout`:** GUI events (e-stop, fork reached or timed out, recording,
  connection changes) go to the LogPanel and also to the node's ROS logger, so
  rosbag and the tests can see them.
* **Qt logging categories** (`forklift.gui.*`, enabled with
  `QT_LOGGING_RULES`): internal numbers such as the displayed FPS.

**I2 setup.** I2 sets `teleop.max_linear: 0.2` and `teleop.scale_default: 1.0`
as launch overrides, so holding W gives exactly 0.2 m/s.

**Requirements.** The tests need an X11 session, which exists here
(`DISPLAY=:1`). The packaged xdotool (2016) has no `windowquit`, so I8 closes
the GUI with SIGINT/SIGTERM and with the window manager's close action.

Findings while implementing (Phase 3):

* `xdotool key --window` uses XSendEvent; Qt ignores those key events while
  its window is not active. The tests therefore activate the GUI window, check
  with `xdotool getactivewindow` that it really is the active window, and only
  then send XTEST keys. Releases are always sent, wherever the focus is.
* X key auto-repeat is switched off for the duration of the run (`xset r off`)
  and restored at exit, so a key held by the test can never repeat into another
  window, for example after `kill -9` of the GUI.
* The GUI binary is started directly (not through `ros2 run`), so its PID is the
  GUI process itself (needed for `kill -9` in I5 and for CPU accounting in I11).
* I4 latency is measured from the first `/e_stop=true` sample to the first zero
  on `/cmd_vel`. Because the GUI publishes the zero teleop command before the
  lock (D-31), the zero may arrive first; the latency is then 0.
* Mouse targets (e-stop button, dataset tab and button) are computed from the
  default 1280×800 layout and the window's device pixel ratio; every click is
  confirmed through the GUI event it must produce.

## D-40 Lint set (Accepted)
Linters that run:

* ament_cmake_clang_format and ament_cmake_cpplint, which the spec requires
* ament_cmake_lint_cmake, ament_cmake_xmllint, and ament_cmake_flake8 for the
  launch files
* qmllint 6.4.2, run as a test over every QML file with the build's import path

Excluded:

* uncrustify, because it conflicts with clang-format
* ament_copyright, because the workspace files have no full license headers

Two adjustments came out of the first lint run (Phase 2):

* cpplint's own `legal/copyright` rule cannot be switched off through
  `ament_lint_auto` (the hook only takes file excludes). Every C++ file
  therefore starts with `// Copyright 2026 muhammet61` and
  `// SPDX-License-Identifier: Apache-2.0`, matching the package license.
* With `BasedOnStyle: Google`, clang-format regroups all angle-bracket includes
  into one sorted block, which puts `<rclcpp/...hpp>` between C++ standard
  headers. cpplint then reports `build/include_order`. ROS headers use the
  quoted form (`"rclcpp/rclcpp.hpp"`), the usual ROS 2 convention, so both tools
  agree on the order.

---

## D-41 qmllint 6.4 workarounds and view state (Accepted)
qmllint 6.4.2 reported warnings for constructs that newer Qt accepts. The code
avoids them instead of suppressing them:

| Warning in 6.4 | Workaround |
|---|---|
| Delegates may not use ids of the enclosing component ("unqualified access"; `pragma ComponentBehavior: Bound` is 6.5+) | `ListView.view`, `parent`, values precomputed into the model, and view state in singletons (`Theme`, `UiState`) |
| `QPointF`, `QRectF`, `QSize` property types are unknown | Trail as `QVariantList` of points, bounds as four doubles, frame sizes as ints |
| `SplitHandle` attached type not found via `QtQuick.Controls.Basic` | `import QtQuick.Templates as T`, `T.SplitHandle.pressed` |
| Model roles `state` and `required` clash with `Item.state` and the `required` keyword in delegates | Health roles are named `status` and `essential` |

`UiState.qml` is a QML singleton holding only view preferences (camera layout,
selected camera tab, crosshair). It contains no logic; it exists so that
delegates can read these values without referring to outer ids.

## D-42 The command chain starts with the simulation (Accepted, 2026-10-17)
**Problem.** When the simulation and the GUI ran in separate terminals
(`forklift_gazebo/sim.launch.py` + `forklift_gui/gui.launch.py`), WASD did not
move the robot. Nobody subscribed to `/cmd_vel/teleop`: twist_mux and
`cmd_vel_gate` came only from `forklift_bringup/control.launch.py`, a third
launch that was easy to forget. The fork still worked, because
`/fork/cmd_height` goes straight to the bridge. This hid the problem.

A second fault made `bringup.launch.py` no help: it never started the GUI.
`IncludeLaunchDescription` turns its `launch_arguments` into
`SetLaunchConfiguration` actions in the **parent** context (launch, Jazzy). The
`gui:=<sim_gui>` passed to the simulation (default `false`) therefore overwrote
bringup's own `gui` argument before the GUI include was evaluated.

**Decision.**
* `twist_mux.yaml`, `cmd_vel_gate.yaml` and `control.launch.py` move from
  `forklift_bringup` to `forklift_safety` (the package that owns the gate).
* `forklift_gazebo/sim.launch.py` includes `forklift_safety/control.launch.py`
  by default (new argument `control`, default `true`). `forklift_gazebo` gets an
  `exec_depend` on `forklift_safety`. The owner approved this change to
  `forklift_gazebo` on 2026-10-17.
* `bringup.launch.py` wraps each include in `GroupAction(scoped=True)`. It passes
  `control` on to the simulation, and its defaults are now `sim_gui:=true`
  and `rviz:=true`: Gazebo window, RViz and the operator GUI.
* `forklift_gazebo/scripts/acceptance_test.py` and the GUI integration test start
  the simulation with `control:=false`. The acceptance test publishes `/cmd_vel`
  directly, and the gate's zero commands would override it. The integration
  test starts the chain itself as before.

| Alternative | Why rejected |
|---|---|
| The GUI launch starts the chain | The gate would die with the GUI terminal. DiffDrive keeps the last command, so stopping would depend on the GUI's final zero winning a race (see D-24). |
| Keep the chain in `forklift_bringup` and include it from `sim.launch.py` | Package cycle: `forklift_bringup` depends on `forklift_gazebo` |
| Only document "also run control.launch.py" | This was the state that failed in practice |

---

# Load stations and parcel (`forklift_gazebo`)

These decisions come from the station task. The owner approved the Phase 1 plan
on 2026-10-19, with one change: the base plate keeps its collision (D-46).
Versions checked: OpenCV 4.6.0 (apt `python3-opencv`), gz-sim 8.15.0,
sdformat 14.9.0, xacro 2.1.1. The robot numbers come from
`forklift_params.yaml`, and the derivations in D-43 were computed from that
file and `station_params.yaml`, not copied.

## D-43 Station frames, bay order and robot fit (Accepted)
* **`station` frame** = the station model frame: origin on the floor at the
  middle of the front edge, +x into the station (the direction the fork goes
  in), +y left, +z up.
* **`bay_i` frame**: origin on the floor on the channel axis of bay `i`, at the
  front edge, same axes. `y_i = (i - (N-1)/2) * bay_pitch`; bay 0 is the most
  −y bay (on the right when you face the station). The SDF defines a
  `<frame name="bay_i">` for each bay.
* All geometry lives in `forklift_gazebo/config/station_params.yaml` and all
  placement in `config/stations.yaml`. `scripts/station_layout.py` is the
  only code that computes bay positions, marker IDs and poses. The texture
  generator, the spawn launch and the acceptance test all import it.

Fit with the robot (fork outer span 0.12, fork top 0.025 at joint 0, fork
length 0.25, carriage front x = 0.13 from `base_link`). Full entry means the
carriage front is at bay x = 0, so `base_link` is at bay x = −0.13.

| Check | Derivation | Value |
|---|---|---|
| Fork clearance in the channel | (0.17 − 0.12) / 2 | ±25 mm (5 mm left at a 2 cm offset; at 3 cm the fork lands 5 mm on a ledge) |
| Parcel overlap on the ledges | (0.24 − 0.17) / 2 = 35 mm per side; with 10 mm minimum overlap | ±25 mm |
| Carry height (joint 0.06) | fork bottom 0.010 + 0.06 = 0.070 > ledge top 0.05 | the robot enters above the ledges; lateral accuracy matters only while lowering |
| Transfer | fork top = 0.05 at joint 0.025; at joint 0 the fork top is 0.025 and its bottom 0.010 (plate top 0.002) | parcel rests on the ledges, the fork is free in the channel |
| Full-entry depth | fork tip at 0.25 | 50 mm to the back plate |
| Robot body at full entry | chassis front −0.050, wheel front −0.080, mast upright −0.020, carriage 0.000 (bay x) | only the forks enter the bay (x > 0) |
| Neighbour bays | neighbour parcel inner edge \|y\| = 0.18 at x ≥ 0.01; carriage \|y\| ≤ 0.12, wheels \|y\| ≤ 0.185 but x ≤ −0.08; carried parcel to neighbour parcel ≥ 30 mm at a 3 cm offset | no contact |

## D-44 Lower marker centre at z = 0.028 (Superseded by D-56: markers removed)
The first spec placed the lower 0.04 m marker at z = 0.005–0.045. A 4×4 ArUco
marker is 6 cells wide including its black border, so one cell is
0.04 / 6 ≈ 6.7 mm. Detection needs a white quiet zone of at least one cell
around the black square, and 5 mm above the floor is less than that. The
marker is at z = 0.008–0.048 (centre 0.028), so the quiet zone spans
z = 0.0013–0.0547. The bottom 0.7 mm of the quiet zone is hidden by the 2 mm
base plate. The black square ends 2 mm under the parcel bottom (z = 0.05), and
the top 4.7 mm of the quiet zone are above it. From a fork camera at
z = 0.02, the parcel's back bottom edge hides the plate above z = 0.0537 at
0.8 m and above z = 0.0565 at 0.5 m, so up to 1 mm of the quiet zone can be lost
at 0.8 m. S5 measures the effect.

## D-45 Different IDs for the upper and lower markers (Superseded by D-56: markers removed)
Pose estimation assumes one marker size for each ID. If the same ID appeared
at 0.08 m and at 0.04 m, the estimate would scale the distance by 2 for one of
them, and the front camera can see both markers at once from a distance. The
scheme is: bay number `k` (0–24, unique across all stations, `bay_ids` in
`stations.yaml`), **upper ID = k**, **lower ID = k + 25**. DICT_4X4_50 has
exactly the 50 IDs this needs. `station_layout.validate_stations` and the
station xacro reject duplicates and out-of-range values.

## D-46 The base plate has a collision (Accepted, owner decision 2026-10-19)
The Phase 1 plan proposed a visual-only plate, because a 2 mm step tilts the
robot. The owner chose a physical plate. Expected effect at full entry: the
drive wheels (bay x = −0.13) stand on the plate and the caster (centre at
bay x = −0.41) stands on the floor, because the apron ends at x = −0.30.
The robot pitches about 0.4° nose up, and the fork tip rises about 4.7 mm.
At joint 0 the fork top is still about 20 mm under the ledges. S6 and S7
measure the real effect. The plate uses the default SDF friction (μ = 1, like
the floor).

## D-47 ArUco textures: build-time generation, OpenCV 4.6 API, `model://` path (Partly superseded by D-56: the build-time principle now covers the alignment-target textures)
* The installed OpenCV is **4.6.0**, so the code uses the pre-4.7 API:
  `cv2.aruco.Dictionary_get`, `drawMarker(..., borderBits=1)`,
  `DetectorParameters_create` and `detectMarkers`. `ArucoDetector` and
  `generateImageMarker` do not exist in this version (checked with `dir(cv2.aruco)`).
* `scripts/generate_aruco_textures.py` runs as a CMake custom command at build
  time. It writes all 50 IDs as 800×800 PNGs: 6 cells plus a one-cell white
  quiet zone on each side, 100 px per cell, so the edges stay sharp. It reads
  each PNG back with `detectMarkers` and stops the build if an ID does not
  match. The PNGs go to `share/forklift_gazebo/models/aruco_markers/materials/textures/`
  and `config/markers.yaml` goes to `share/forklift_gazebo/config/`. Nothing
  is written to the source tree, and `.gitignore` covers both paths in case
  the script is run by hand.
* The marker visual is a 1 mm box with no collision, 0.5 mm in front of the
  back plate. Its material is `<pbr><metal><albedo_map>model://aruco_markers/...`.
  The box edge is the texture edge: black square × 8/6.
* Verified on 2026-10-19 with a headless probe on the installed gz-sim 8.15.0.
  A box spawned with `ros_gz_sim create -string` resolved its `model://`
  texture through `GZ_SIM_RESOURCE_PATH`. Its **−x face shows the texture
  without mirroring and upright**: `detectMarkers` returned the right ID, with
  corner 0 at the top left. Two side findings: `create` overrides the model's
  own `<pose>`, so poses go through `-x -y -z -Y`; and a quiet zone on a
  darker background (a lit grey marker on a light sky) was **not** detected,
  but it was detected with a white plate behind it, which is the station
  design.
* Writing SDF with xacro 2.1.1: `range`, `sorted` and `set` must be called as
  `python.range` and so on (the bare names are deprecated or missing), and
  `--` is not allowed inside XML comments.

## D-48 Station acceptance test definitions (Accepted, 2026-10-19; S4/S5 removed by D-56)
* **S4 distance** = horizontal distance from the front-camera optical centre
  to the marker plane. In the ±15° case, the robot stands on an arc around
  the marker centre and its yaw points at the marker. The upper marker of a
  full bay can be seen above the parcel only when that distance is under
  0.53 m, so it is hidden at 0.6, 1.0 and 1.5 m.
* **S5 camera**: a static model that the test adds to the world (no change to
  the robot; the owner confirmed on 2026-10-19 that no third camera goes on
  the vehicle). Matrix: z = 0.02 with and without a parcel; z = 0.08 without a
  parcel. With z = 0.08 and a parcel there is no line of sight: the camera is
  above the parcel bottom (0.05), and at 0.2 m it is inside the parcel. That
  case is recorded for information, with "not detected" expected. The yaw
  criterion (< 1° at 0.5 m) is at the limit of what can be observed: 1° yaw
  changes the left/right edge heights of the 31 px marker by only 0.043 px. The
  test reports a 30-frame mean and the per-frame spread.
* **S6/S7 location**: the middle bay of the source station, with full
  neighbours. The green parcel is moved onto the forks. S6 thresholds: parcel
  bottom 0.05 ± 2 mm, tilt < 2°, fork joint < 2 mm, drag while reversing
  < 5 mm.

## D-49 Test service calls go through the `gz service` CLI (Accepted, Measured)
The test process subscribes to five continuous gz topics: dynamic poses (60 Hz),
joint states (50 Hz) and two cameras (30 Hz, about 1 MB per frame). While those
subscriptions were active, gz-transport's Python `Node.request()` was not
reliable. `/create/blocking` never returned, even for an empty box and even with
a 60 s timeout, and `set_pose` sometimes waited out its 5 s timeout before a
retry worked. The same requests from a separate process return in about
0.3 s. A second `Node` in the same process did not help. Probable cause:
gz-transport shares one receive path per process, and the Python callbacks
compete with it. A minimal script with one image subscription did **not**
reproduce the problem, so the mechanism is not proven. Decision: `Gz._req` in
`station_acceptance.py` builds the request as protobuf text and calls
`gz service` in a subprocess. Two more facts were measured:
* `/world/<w>/create` (async) returns `data: true` even when the name already
  exists, and it returns before the entity exists. The test uses
  `/create/blocking` and `/remove/blocking`, because only they report the
  real outcome.
* Creating a new render camera can take longer than 5 s. The S5 test camera
  is therefore spawned **once**, as a dynamic model with gravity off, no
  collision and 10 g of mass. For each view it is moved with `set_pose`, and
  its real pose is checked in `dynamic_pose/info`. A static model has neither
  of these properties.

## D-50 ArUco detector parameters (Superseded by D-56: markers removed)
`station_params.yaml → markers.detector`: `CORNER_REFINE_SUBPIX` and
`perspectiveRemoveIgnoredMarginPerCell = 0.3` (OpenCV default 0.13). With the
default, the lower marker was missed at 0.6 m (z = 0.02) and at 0.7–0.8 m
(z = 0.08), but it was found at 0.5 and 0.7 m. That non-monotonic pattern
comes from texture sampling: at 25–30 px (about 4 px per cell) the cell edges
move by ±1 px, and bit sampling at the cell borders fails. The parameter was
swept offline on the 46 saved S4/S5 frames. 0.25–0.45 all give 21/21 lower
marker detections, unchanged S4 results and no wrong or hidden IDs. 0.3 is
inside that range. Caveat: the value was tuned on the test frames, and the
plateau is the evidence that it is not overfitted. The future detection node
should read the same block.

## D-51 Findings from S4/S5 that affect the fork-camera step (Closed by D-55/D-56; camera-noise finding still open)
1. **Camera noise is not applied.** Both robot cameras and the S5 camera
   declare Gaussian noise with σ = 0.007. Consecutive frames are identical
   (mean difference 0.001 grey levels), and the sky region has a spatial
   standard deviation of exactly 0. This was measured in headless EGL **and**
   in windowed mode (`lift_test`, 2026-10-19). The 30-frame averages in S5
   are therefore one deterministic sample. Robot scope, not changed here; see
   REPORT_STATIONS §9.
2. **Pose of the 4 cm lower marker at 0.5 m:** the lateral error is
   −0.8 mm, and ≤ 1.6 mm in the ±1/±2 cm and ±2/±5° sweeps. The yaw error is
   +21.6°. The two IPPE solutions are +21.6° and −21.9°, with reprojection
   errors of 0.17 and 0.29 px. The detected corners are up to 1.5 px from the
   true projection: the black square renders about 1–2 px smaller, more at
   its lower edge. In the sweeps the estimated yaw barely moves when the true
   viewing angle changes (lateral ±2 cm → ±2.3°), so yaw relative to the
   viewing ray is not observable at this size. This matches the Phase 1
   estimate of 0.043 px per degree. The S5 yaw criterion (< 1°) fails.
   A detector setting cannot fix it. Meeting it would need a geometry change
   (owner's decision), for example a horizontal baseline of two lower markers
   per bay, or a criterion change, for example yaw from lateral offsets measured
   at two distances along the approach.
3. **Upper-marker distance bias (front camera):** estimates are too long by
   +1.5 % at 0.6 m, +2.5 % at 1.0 m and +3.7 % at 1.5 m. This is the same
   inward corner shift. It is information only and not part of the S4
   criterion.

## D-52 Gazebo GUI opens only after the world is populated (Accepted, owner decision 2026-10-20)
Symptom: in `station_test` the red parcel (`parcel_source_0`) was sometimes
missing from the Gazebo GUI while both robot cameras showed it. The launch log
reported `Entity creation successful.` for it, and S3 sampled its pose from the
server. The next launch, with no code change, showed it. Cause: the GUI client
takes the scene once at startup (`/world/<w>/scene/info`) and then follows
only incremental updates, so a model created between the two never appears in
the GUI. Sensors render on the server, so the cameras and every test were
unaffected. The robot is spawned the same way and was exposed to the same race.
Decision: `sim.launch.py` always runs the server with `-s`. With `gui:=true`,
`scripts/wait_for_models.py` polls `scene/info` through the `gz service` CLI
(D-49) until the robot and the names in `gui_wait_models` are present. Then
`gz sim -g` starts in the same launch. The gate is the scene itself and not
the exit of the `create` processes, because async `/create` returns before the
entity exists (D-49). `station_sim.launch.py` fills `gui_wait_models` with the
station and parcel names from `stations.yaml`. On timeout (90 s) the GUI still
opens, and the missing names are printed. Closing the GUI window still shuts
the launch down. Considered and not chosen: embedding stations and parcels in
the world SDF (does not cover the robot), and a fixed delay (no guarantee).
Measured: `station_test` reported 8 models in the scene after 3.0 s and
`lift_test` reported 1 after 2.2 s; the GUI started after that. `gui:=false`
starts neither the waiter nor the GUI.

## D-53 Factory world and industrial station look (Accepted, owner request 2026-10-20)
Goal: the station scene looked like a playground (saturated full-colour plates
on an endless grey plane). The owner asked for a professional factory look
that still serves the CV work. CV tests (S3–S7) are deferred to the CV phase
by the owner.
* **New world, old one kept.** `worlds/factory.sdf` is generated at build time
  by `scripts/generate_factory_world.py` from `config/factory.yaml` (+
  `stations.yaml`, `station_params.yaml`). `station_test.sdf` is unchanged and
  stays the deterministic test world. Both use the same `stations.yaml`; the
  robot is still at the origin facing +x. `station_sim.launch.py
  station_world:=factory` and `bringup.launch.py world:=factory` open it.
* **Hall.** Open Robotics' `Depot` (Fuel, version 6 pinned, CC-BY 4.0) gives a
  real-scale brick hall (≈ 30 × 15 m, roof ≈ 8.5 m, mezzanine, roll-up doors,
  baked lightmap). Concept: a real-scale hall with a small AMR cell, not a
  shrunk hall. Edits through `<experimental:params>`:
  * Its 100 × 100 m floor collision is removed; our `ground_plane` collision
    (μ = 1.0, D-15) stays, without a visual.
  * Its two `fan_light` point lights are removed. All three attenuation
    coefficients are 0 (1/0). Measured with every world light switched off:
    our models were still saturated in some cameras and black in others,
    depending on the view. After the removal they are black in all cameras,
    as expected. The lightmap is not involved (a lightmap-free copy behaved
    the same while the lights were present).
  * `hall.clutter: full | clean`: `clean` also removes the Depot's shelves,
    barrels and crates (`BOXSET`, `crates`). The default is `full`; the owner's
    choice between the two is still open.
  * The hall is rotated by π (pose (−8.1, 0, 0, π)) so that the background of
    the source station, where the front camera reads the parcels and seals,
    is the plain roll-up-door wall. The red-barrel shelves end up 4.4 m
    behind the blue destination.
* **Lighting** is calibrated, not eyeballed: the front camera at the spawn pose
  sees the three source seals within ±6 % of their `station_test` RGB values
  (blue 35/73/142 vs 35/74/145). Ambient 0.45, one shadowed skylight
  (directional, 0.6), a 2 × 2 high-bay grid over the cell (0.9, first spot
  shadowed) and a 2 × 2 grid for the hall (0.5, no shadows). Light specular is
  low so the Depot floor does not show rippled highlights.
* **Floor markings** (visual only, 1 mm high): yellow/black hazard band around
  the cell, a 50 mm yellow guide lane from the spawn point to each station
  apron with a white stop bar, and a green walkway with stencils. The lanes are
  also written to `config/factory_lanes.yaml` as ground truth for the future
  lane detector. Measured: on a box top face the texture u axis runs along
  local −y, so textured strips put their long side on local y; the −x and +y
  faces read without mirroring (as in D-47).
* **Props** outside the cell (Fuel, pinned): AWS RoboMaker warehouse shelves,
  pallet loads, pallet jack and bin (CC-BY 4.0), fire extinguishers (CC0);
  hazard-striped pillar guards and signs on the two pillars next to the cell.
  The pillar positions (0.46 m square, world (−0.56, −3.615) and
  (−0.56, 3.885)) were measured from the mesh vertices. Nothing brown or
  red/green/blue stands within 5 m behind the source station.
* **Station and parcel look** (all worlds). Collision, inertia and frames are
  byte-identical to the previous xacro output (compared for N = 1, 3, 5 and a
  parcel); S1 and S2 pass (1154 checks, 0 errors). Visuals: tread-plate base
  with a perimeter band in the station colour (instead of a fully coloured
  plate), aluminium back plate and profile frame, a station sign, bay-number
  labels, rubber ledges, white label plates behind the ArUco markers, kraft
  cardboard parcels with tape and a shipping label. The seal is unchanged. The
  back-plate visual sits 1 mm behind its collision so the label plate fits
  between it and the marker; marker poses are unchanged. Bay labels have no
  square outline, because a black square contour is an ArUco candidate.
* **Textures.** Photo textures are ambientCG CC0, downscaled and committed
  (`models/factory_materials`, 1.5 MB, provenance in
  `scripts/fetch_factory_textures.py`), so builds need no network. Signs,
  labels and stripes are generated at build time
  (`scripts/generate_factory_textures.py` → `model://factory_signs`). Metalness
  maps are not used: without an environment map ogre2 renders metal nearly
  black, so metalness is a small constant per finish.
* **Gazebo GUI** for `factory` uses `config/factory_gui.config` (the gz-sim 8
  default plus an opening camera pose and a dark background), passed with
  `gz sim -g --gui-config` through the new `sim.launch.py` argument
  `gui_config`.
* Fuel models download on the first run into `~/.gz/fuel`; after that the
  world loads offline. Performance is in REPORT_FACTORY.md §6.

## D-54 Robot appearance, visual only (Accepted, owner request 2026-10-20)
The plain yellow chassis box was the most toy-like element left. New
`urdf/components/appearance.xacro` adds `<visual>` elements to existing links
only: graphite skirt, yellow body, deck, counterweight with rounded rear
corners, rubber bumpers and side strips, LED strip, electronics tower with a
LiDAR, amber beacon, e-stop, front safety scanners, yellow wheel hubs and a
lift cylinder behind the mast. Every size is in `forklift_params.yaml →
appearance`.
* **Physics and TF unchanged.** Links, joints, inertials, collisions and the
  `<gazebo>` blocks are identical to the previous URDF (57 elements compared).
* **Cameras unchanged.** Every new part is behind the optical centres
  (x = 0.10), so neither robot camera can see it; the visible parts (forks,
  carriage, mast front) keep their geometry and colour. Measured: front and
  lane images with the old and the new body differ in 0.4 % of the pixels by
  at most 6/255, the same as two renders of one unchanged scene (render noise).
* **Decals.** sdformat ignores URDF visual names (it renames them
  `<link>_visual_<n>`), so a per-visual material through `<gazebo reference>`
  cannot be applied. The hazard stripe and the "FKL-01" side label are
  textured OBJ quads generated at build time (`scripts/generate_decals.py` →
  `meshes/decals`). `package.xml` exports `gazebo_model_path="${prefix}/.."`
  so gz resolves the converted `model://forklift_description/...` URIs.
* **Colours.** Measured: sdformat multiplies URDF colours by 1.25, and ogre2
  converts linear to sRGB, which lifts dark tones a lot (0.24 grey showed as
  ≈ 185/255). The dark parts therefore use low values (graphite 0.035).
* **RViz draws each link in one colour** (found 2026-10-20). rviz2 Jazzy
  (`robot_link.cpp`, `createVisualizable`) passes an empty material name for
  every visual, so `getVisualWithMaterial` returns the link's first visual and
  all visuals of the link get its material; inline colours do not help
  (tested). Gazebo is not affected. With the graphite skirt first, the whole
  chassis was black in RViz. The yellow shell is therefore the first chassis
  visual, so RViz shows a yellow body (the details keep their colours only in
  Gazebo). Checked in an isolated Xephyr display with `display.launch.py`.
  Alternative not chosen: one multi-material mesh per link (new build step).

## D-55 Channel alignment targets on the ledge fronts (Accepted, owner request 2026-10-20)
Problem: the 4 cm lower marker gives the lateral offset well but not the yaw
(D-51: +21.6° at 0.5 m). The owner proposed reference marks on the ledge
fronts on both sides of each channel, for fork alignment and centring.
* **Pattern (owner decision after an A/B test): bullseye.** A 36 mm white
  plate with a black ring (28/20 mm) and a 12 mm black dot; 4 mm white margin,
  ring and gap. The measured point is the dark-mass centroid inside the plate
  (radially symmetric pattern ⇒ centroid = centre). `markers.align.pattern`
  keeps `checker` (24 mm 2×2 checker) as an option; both textures are built
  and self-checked. A colour mark was rejected: yellow is everywhere in the
  factory and red/green/blue are the seal colours.
* **A/B result** (`docs/logs/stations/align_compare.txt`; 7 lane-camera views
  0.45–1.2 m with lateral and yaw offsets, Gaussian noise σ = 0 / 0.007 /
  0.02 with 20 repeats; same pairing for all). The method mattered more than
  the pattern: with the centroid, worst cases at σ = 0.02 were bullseye 0.22 mm
  / 0.07° near (≤ 0.6 m) and 0.20 mm / 0.20° far (0.9–1.2 m), checker 0.18 mm
  / 0.10° and 0.22 mm / 0.24°; cornerSubPix on the checker reached 4.0 mm /
  3.45° far; separating the thin bullseye ring failed in every view (the 45°
  view compresses the face vertically and the texture filter blurs 4 mm
  features). 100 % pairing success for the centroid methods. Caveats: the
  centroid uses the whole plate, so a shadow across it can bias the point
  (to test on real approaches); the bullseye pattern test also accepts the
  lower ArUco markers, which the geometric pairing (same front line, 216 ±
  20 mm baseline) removes. The owner's small direction triangle was left out:
  4 mm would be 1–2 px.
* **Placement.** Two targets per bay, on the ledge front faces right next to
  the channel: the plate's channel-side edge is 5 mm from the channel edge
  (owner's proposal of "at the very edge", with a margin so the plate is not
  cut), vertically centred on the 50 mm face (z = ledge_height / 2). Centre
  offset from the channel axis = channel_width / 2 + edge_gap + size / 2 =
  0.108 m, so the pair midpoint is exactly the channel axis and the 0.216 m
  baseline makes yaw observable. Per-bay pairs (not one mark per ledge) keep
  the midpoint on the axis for the end bays too, whose outer ledges are half
  as wide. They fit: 41 of 65 mm on the end ledges, 82 of 130 mm on the shared
  ones (`station_layout.validate_align`, checked at build time).
* **Implementation.** Visual-only boxes in `station.sdf.xacro` (standoff and
  thickness as the markers, textured −x face, no shadow); sizes in
  `station_params.yaml → markers.align`; `align_target.png` is generated with
  the ArUco textures and must pass a self-check (centre corner within 0.5 px
  of the texture centre, a 2 px shift fails it). `markers.yaml` gains
  `align_targets` (point in the station and world frames for every target).
  S2 checks count, position, size, edge gap, "on a ledge, not over the
  channel", texture and markers.yaml (1723 checks, 0 errors; a 15 mm edge-gap
  mutation produced 12 failures). Collisions unchanged. The ArUco markers
  still decode with the targets in view (IDs 3–5, 28–30; no false IDs).
* **First trial** (checker + cornerSubPix, noise-free,
  `docs/logs/stations/align_trial.txt`) was superseded by the A/B above. Targets
  are found without ground truth; rays are intersected with z = ledge_height /
  2. This addresses the yaw part of D-51 (lower marker: +21.6° at 0.5 m); the
  S5 criterion itself is redefined in the CV phase.

## D-56 ArUco markers removed; station identity from the sign, bay lanes (Accepted, owner decision 2026-10-21)
Owner's reasoning, checked against what each marker still did:
* **Occupancy** belongs to object detection of the parcel and its seal, not to
  "the parcel hides the upper marker".
* **Identity**: the robot can tell the station from its sign. The coloured
  block (≈ 15 × 15 cm) is larger than the 8 cm upper marker, so it is visible
  from farther away, and it matches the task logic (drop a blue-sealed parcel
  at the blue sign). The source sign is the neutral one. The bay index comes
  from the order of the alignment-target pairs (left to right), so bay IDs are
  not needed. Limit: two stations of one colour would be ambiguous; fine for
  four stations.
* **Global pose from a surveyed marker map** is not wanted: the robot works
  relative to what it sees. `markers.yaml` stays as ground truth for
  evaluation only and is marked "the robot does not read it".
* **Fine alignment**: the bullseye targets (D-55) replace the lower marker
  (0.2° vs 21.6° yaw error), and the lower markers produced false bullseye
  candidates.

Changes:
* Removed: upper and lower markers and their white label plates, the ID
  scheme (k / k + 25), the 50 ArUco textures, `markers.dictionary`,
  `lower_id_offset`, quiet zone, `markers.detector` (D-50), and tests S4/S5
  with the ArUco reader and the S5 test camera. S4/S5 stay in the results
  table as **KALDIRILDI** so the numbering does not shift; an alignment test
  on the bullseye targets replaces S5 in the CV phase. Bay IDs stay for the
  yellow bay labels.
* `generate_aruco_textures.py` → `generate_station_textures.py`;
  `model://aruco_markers` → `model://station_targets` (alignment textures);
  `markers.yaml` now holds `align_targets` and `bay_lines`.
* **Bay lanes**: a 40 mm line on the tread plate along every bay axis, from
  the inner edge of the apron band to the channel entry (x = 0), in the
  station colour (yellow at the source). Part of the station model, so it
  exists in both worlds and moves with the station. Lane camera: follow the
  bay line from ~2 m to ~0.5 m, then the bullseye pair for the last
  millimetres and the stop distance (the line end alone gives neither).
* **Coloured floor lanes** (factory): each lane has its station's colour, the
  lane to the source is yellow, so "follow the blue line" needs no junction
  logic. Lanes start 0.12 m from the spawn point so the four colours do not
  overlap (z-fighting). Owner's alternative, a yellow lane with small symbols,
  is kept as a fallback if the coloured floor confuses seal detection (the
  seals sit on parcels above the floor, so a parcel ROI separates them).
* S2 now checks the bay lanes (count, start/end, axis, width, top face,
  colour, markers.yaml) and that no marker visuals are left: 1231 checks,
  0 errors. S3, S6, S7 and S8 re-run after the change: all pass (drop ±2.5 cm,
  pick 0.00 mm slip, RTF 1.000).
* Observed: with the lower markers gone, the yellow bay label on the back plate
  is the one remaining false bullseye candidate (dark centre, light edge); the
  same-front-line rule removes it (it sits 0.3 m behind the targets).

---

# First CV + autonomy slice (`feature/cv-autonomy`)

The plan is `docs/PLAN_CV_AUTONOMY.md` (Turkish, owner-approved 2026-10-21; it
keeps its own change log). Results are in `REPORT_CV.md`.

## D-57 Slice plan and sim-to-real rules (Accepted, owner decision 2026-10-21)
* **Goal of slice 1 (v0.4.0):** take the three parcels from the source one by
  one and drop each into a free bay of the station whose sign colour matches its
  seal, in `station_test` and `factory`, side bays included, without manual
  help. Sub-slices: perception → approach + pick → transit + drop → mission and
  faults. The plan decisions K1–K14 were accepted as proposed, with one change
  (K13, below).
* **Sim-to-real (owner's project goal).** The CV and autonomy code must move to
  a real robot with models trained on real data and without large changes. So
  the robot perceives the world from its cameras; Gazebo poses and object
  information are for evaluation only. Rules R1–R9 in the plan, in short:
  perception and autonomy may use images + `camera_info`, TF from the URDF,
  wheel odometry, the fork joint, `/e_stop` and operator commands; they must not
  use gz poses, gz services, gz-transport, the layout files (`stations.yaml`,
  `markers.yaml`, `factory_lanes.yaml`, `factory.yaml`) or model names.
  Detectors sit behind typed messages, so a learned model can replace a
  classical one (`backend` parameter); thresholds live in configuration;
  labels use COCO/YOLO so real labelled data plugs in. `station_params.yaml` is
  catalog knowledge (shapes, sizes, colours; no positions or occupancy). Test
  C0 checks the rules.
* **GUI modes (K13, changed by the owner).** The operator GUI gets Manual /
  Auto mode buttons, and the mission starts from the GUI. The mode is owned by
  the autonomy node (`/autonomy/set_mode`, `forklift_interfaces/srv/SetMode`)
  and shown from `/autonomy/state`. In Auto the GUI's fork panel is disabled;
  a drive key takes over (the GUI switches to Manual and the mission pauses).
  The panel comes in F3 with the autonomy node.

## D-58 Package layout and shared simulation test tools (Accepted)
* New packages: `forklift_interfaces` (messages and services, ament_cmake),
  `forklift_perception` (ament_python; a ROS-free core under
  `forklift_perception/core` plus thin nodes), `forklift_evaluation`
  (ament_python; the only package that reads ground truth). `forklift_autonomy`
  follows in F3. Every perception message is stamped with the source image
  stamp and expressed in `base_footprint`; lanes are polylines, so curves will
  not change the message.
* The gz helpers of `station_acceptance.py` (pose maths, `SimSession`, `Gz`,
  `RobotGeom`, placement) moved unchanged into
  `forklift_gazebo/scripts/sim_tools.py` (owner approval K9). `SimSession`
  gained `station_world` (from `st.world`), `control` and extra launch
  arguments. `forklift_evaluation` imports it from `lib/forklift_gazebo`, as the
  launch files already import `station_layout.py`. `station_layout.py` gained
  `sign_rect()` (the sign face in the station frame, same formula as the xacro).
  Regression after the move: S1–S3 and S6–S8 pass with identical numbers
  (2026-10-21; evidence kept outside the repository so the committed results
  are not overwritten).
* The evaluation process runs rclpy (images, `camera_info`, TF, `/odom`) and
  gz-transport subscriptions (poses, stats, joint states) in one process; it
  worked without stalls. Service calls still go through the `gz service` CLI
  (D-49).

## D-59 Gazebo's principal point is half a pixel from `camera_info` (Accepted: option (c), owner decision 2026-10-22, may be revisited)
Measured in F1 by projecting ground truth into both robot cameras
(`f1_probe overlay`, `station_test`, 20 views): the image is rendered with its
optical centre at (W − 1)/2, (H − 1)/2 = (319.5, 239.5) in OpenCV pixel-centre
coordinates, but `camera_info` reports cx = 320, cy = 240. Evidence:
* bullseye dark-mass centroids minus projected centres, with the plate mask
  kept 1 px inside the plate: about −0.4 px in u and v, the same in both
  cameras and independent of image position and distance (a geometric error
  would change with both);
* a second feature, the bay-lane centre line (row-wise colour centroid minus
  the projected centre line): −0.50 px median over 72 lanes.
Effect: about 1.5 mm at 1 m for the lane camera, against a 2 mm alignment
budget. Options: (a) leave it and accept the bias; (b) a fixed −0.5 px
correction in perception code (simulator-specific, against R3); (c)
**recommended:** treat the simulated camera like a real one. An evaluation
tool calibrates it from ground truth and writes a calibration file;
perception gets an optional `calibration_file` parameter that overrides
`camera_info`'s K and D. On a real robot the same parameter takes the real
calibration.

**Decision (owner, 2026-10-22):** option (c). The calibration tool spawns a
checkerboard in the simulation and runs the standard OpenCV procedure
(`findChessboardCorners`, `cornerSubPix`, `calibrateCamera`) on many views. It
writes ROS camera-calibration YAML files, which perception loads through
`calibration_file`. The owner may change this later.

**Result (2026-10-22, `calibrate_cameras`, 9 × 6 inner corners, 25 mm squares,
board placed at random positions, distances and ±40° tilts over the whole image):**

| Camera | Views | RMS | cx | cy | fx | fy |
|---|---|---|---|---|---|---|
| front | 37 | 0.066 px | 319.80 | 239.52 | 554.63 | 554.64 |
| lane | 27 | 0.064 px | 319.77 | 239.38 | 381.26 | 381.23 |

The principal point is within 0.3 px of (319.5, 239.5), which confirms the
finding with a method that uses no ground truth. Focal lengths differ from the
nominal values by at most 0.07 %; distortion coefficients are below 0.005. A
first run with only 3–7 views gave cx 320.6 / 319.0: with few views the
principal point trades off against the board poses (as with a real camera).
The files are `forklift_perception/config/calibration/sim_<camera>.yaml`; the
parameter is `calibration_dir`.

## D-60 Wheel odometry comes from the wheel joints (Accepted, Measured)
The plan allows wheel odometry for short relative moves (K3), because a real
robot has wheel encoders. Checked in F1 (`f1_probe odom`): teleporting the
robot 0.5 m with `set_pose` left `/odom` unchanged (0.000 m). Driving 0.3 m
gave 0.3009 m in both ground truth and `/odom`. gz DiffDrive therefore
integrates the wheel joints, like an encoder, and does not copy the model pose.
On this floor there is no slip, so the simulated odometry is better than a real
one would be. The autonomy must still re-measure with the cameras at each
approach.

## D-61 Centroid masks must stay inside the plate (Accepted, lesson for F2)
In the same measurement the plate mask was first drawn from rounded polygon
corners, so it included dark ledge pixels along the right and lower edges. That
moved the dark-mass centroid by +0.6 px (u) and +0.9 px (v). Shrinking the mask
by 1 px removed the effect. The alignment-target detector (F2) must build its
weight mask strictly inside the plate, for example eroded or from a
sub-pixel quad, and C4 compares against ground truth to catch this.

## D-62 Final approach: align first, then decide the drop from a CV distance (Accepted, owner decision 2026-10-22)
F1 measured that, with a parcel on the forks, the bullseye pair leaves the
parcel's silhouette at 0.50 m but is in its shadow until 0.575 m (camera to
ledge front line), which leaves about 0.55 m to full entry. Owner's decision:
finish the alignment (lateral and yaw) on the bullseye pair where it is
unshadowed, then approach; the stop and the drop are decided from a camera
distance estimate, not from odometry alone. The cue is the bay label: a 45 mm
yellow square on the back plate, centred on the bay axis at z = 0.258. The
lane camera should see the upper back plate over the parcel, so the label can
give the distance and the lateral offset down to full entry. Odometry only
fills the gaps between frames. The label number is not read; only its geometry
is used (catalog knowledge, R7). Test C9 measures it (F2); if the label is not
visible over the parcel, the alternative is decided at gate K-2.

## D-63 Perception structure: ROS-free core, catalog copy, thresholds from data (Accepted)
* `forklift_perception/core` holds the camera model and the detectors and has no ROS imports;
  `pipelines.py` runs them per camera. The ROS nodes (`nodes.py`) and the offline evaluation
  (`forklift_evaluation cv_eval`) call the same pipelines, so the tests measure the code that runs
  on the robot.
* Two configuration files: `perception.yaml` (ROS parameters: topics, frames, file paths; no
  defaults in code, D-29) and `detectors.yaml` (detector thresholds, read by the core, so the offline
  evaluation needs no ROS parameter server).
* **Catalog copy.** `core/catalog.py` derives station geometry (target offset and baseline, bay-line
  extent, sign and label placement, parcel size) from `station_params.yaml` with the same formulas
  as `forklift_gazebo/scripts/station_layout.py`. This is a deliberate duplicate: a real robot will
  not have the simulation package (R8). `forklift_evaluation/test/test_catalog_consistency.py` keeps
  the two equal. The sign formula assumes stations with at least three bays (full sign width).
* **Colour thresholds from data.** `color_stats` projects ground-truth regions into the recorded
  frames and reports HSV percentiles per class; the thresholds in `detectors.yaml` come from those
  numbers. Kraft (H 17), the bay-label yellow (H 20–23) and the lane yellow (H 26–30) are only 2–3
  hue units apart, so every detector also checks shape and context. In the factory, spot-lit lanes
  and parcel tops saturate to V = 255; the thresholds accept that.
* **Self mask.** Forks, carriage and (when the load check says so) the carried parcel are projected
  from `forklift_params.yaml` and the fork joint, and are excluded from target, label and lane
  detection.

## D-64 Lanes: bird's-eye view, perpendicular removal, edge-aware centre line (Accepted)
* The lane camera image is warped to the ground plane (5 mm per pixel). Per colour: structures
  perpendicular to the approach (60–120°) are found with line-kernel openings and removed first
  (otherwise the station's front band joins bay lines and side bands into one U-shaped component),
  then openings along ±45° keep lane-like structures; connected components with lane width and
  length become lanes.
* The centre line is computed per 2 cm slice. When one edge of a slice touches the self mask or
  the edge of the view, the centre is taken half a nominal tape width inside the visible edge (a fork
  covering the lane edge moved the centre by 12.5 mm before). Lateral offset and heading come from a
  line fit to these centres; the message carries the polyline, so curves need no message change.
* Kind: a lane on a bay axis (a detected bullseye pair's axis, or that axis ± k·bay_pitch for
  neighbouring bays whose pair is not detected) ending at the front line is a bay lane; a station-
  coloured stripe off those axes that ends at or beyond the front line is the station's band and is
  dropped; everything else is a floor lane.

## D-65 Final-approach distance: plane intersection with apron pitch compensation (Accepted, Measured)
* The bay label (D-62) is visible over the parcel down to full entry: 100 % of the recorded final-
  approach frames, loaded and unloaded, in both worlds.
* The distance comes from the ray / plane intersection at the label's known height (repeatable to
  about ±1 mm). It is biased when the robot pitches: with the drive wheels on the 2 mm apron plate
  the body turns nose-up about the caster contact by atan(0.002 / 0.28) = 0.41° (S6 measured −0.41°),
  which moved the distance by up to −10 mm. Ground truth showed half that pitch (−0.205°) at the end
  bays, where the outer wheel is off the plate (the plate is as wide as the station).
* Compensation: when the camera distance to the front line is shorter than the apron depth, the
  measurement is repeated with the camera model pitched about the caster contact; the pitch is
  scaled by the number of drive wheels on the plate, which the robot sees as neighbouring bays (labels
  or pairs at ± bay_pitch). Results are expressed in the body frame (the forks and odometry move with
  the body).
* Tried and rejected: PnP on the label corners. A 45 mm label is about 40 px wide at this range, so
  half-pixel corner errors changed the distance by ±5–18 mm.
* Real robot: an IMU would measure the pitch directly; recommended for the hardware step.

## D-66 Station sign: identity and coarse pose only (Proposed, owner decision at gate K-2)
* Two paths, results merged: (A) white-panel anchor: the 3:1 white panel is found, extended left by
  one third to the colour block, the pose is fitted to the four sign corners, the colour is read from
  the block; needed for the source sign, whose dark neutral block merges with dark frames and walls;
  (B) colour block: the block's corners give the pose, refined with the sign's right edge. Both fit
  only x, y and yaw (the sign is vertical at a known height).
* Physical limit: yaw comes from the difference in depth of the sign's two ends; for a 0.60 m sign
  that is about 1 px per degree at 1.3 m and 1 px per 4° at 2.5 m. The plan's C1 criterion (yaw ≤ 5°
  at ≤ 3 m) cannot be met by the sign alone. Proposal: the sign gives identity, bearing and coarse
  distance; the precise station pose comes from the bullseye pairs, visible from about 1.4 m.

---

## Spec deviations (summary)
| ID | What | Status |
|---|---|---|
| D-16 | T8 sequence: reverse/turn/turn back/forward, lowering onto the same rails | Accepted (option A) |
| D-19 | T10 runs inside the T8 transport (loaded forward leg in free space) | Accepted |
| D-20 | Drive-wheel collision is a sphere (r 0.05); visual and inertia stay a cylinder | Accepted |
| D-06 alt. | Lane camera x 0.10 → 0.11 | Not adopted |
| D-18 alt. | Contact system in `lift_test.sdf` | Not adopted |
| D-24 | GUI: new package `forklift_safety` (`cmd_vel_gate`); twist_mux output moves to `/cmd_vel/mux` | Accepted |
| D-37 | GUI: recording indicator added to the TopBar | Accepted |
| D-42 | Command chain moves to `forklift_safety` and is started by `forklift_gazebo/sim.launch.py` | Accepted |
| D-44 | Stations: lower ArUco marker centre z 0.025 → 0.028 (one-cell quiet zone) | Accepted |
| D-45 | Stations: upper ID = k, lower ID = k + 25 (not the same ID) | Accepted |
| D-48 | Stations: S5 at z = 0.08 tested without a parcel only (no line of sight with one) | Accepted |

## Assumptions still to verify
1. ~~In `gz sdf -p`, lumped collisions keep their μ and lumped sensors keep their
   poses~~. Verified: the caster's lumped collision has μ = 0, the forks μ = 1,
   and the sensor poses were moved to `base_footprint`.
2. ~~Whether DART enforces joint velocity limits~~. Verified in source and in
   T7 (5 cm in 0.91 s).
3. ~~What `dynamic_pose/info` contains~~. Verified: non-static models in world
   frame and their links relative to the model, at 59 Hz.
4. ~~The `/clock` publication rate~~. Verified: 1 kHz, the physics step.
5. Spot-light shadows in ogre2. Check this in the Gazebo GUI screenshot (Phase 4).
6. ~~GUI: the gstreamer multimedia backend loads in this desktop session, so
   `VideoOutput` shows frames (D-34)~~. Verified in Phase 4 (see D-34).
7. ~~GUI: qmllint 6.4.2 can be made warning-free with the Controls 6.4 types~~.
   Verified: warning-free with the workarounds in D-41.
8. ~~GUI: `rcl_yaml_param_parser` accepts the gui_params.yaml draft~~.
   Verified on 2026-10-15. The draft was loaded with `--params-file` into an
   rclpy node, which uses the same rcl parser. All 92 keys arrived with the
   expected types (DOUBLE_ARRAY presets, STRING_ARRAY ids, INTEGER counts), and
   the UTF-8 labels were intact. The twist_mux draft uses block style.
