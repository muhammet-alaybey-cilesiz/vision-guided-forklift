# forklift_description

This package describes a small forklift robot in xacro. The description does not depend on any simulator. The robot is the platform for a computer-vision learning project.

Every dimension, mass, limit and sensor parameter lives in
[`config/forklift_params.yaml`](config/forklift_params.yaml). The xacro files hold no numbers of their own.

## Contents

| Path | Role |
|---|---|
| `config/forklift_params.yaml` | Single source of truth (SI units, REP-103) |
| `urdf/forklift.urdf.xacro` | Top-level assembly: arguments, YAML load, macro calls |
| `urdf/common/inertial_macros.xacro` | Box, cylinder, sphere and composite-box inertials computed from geometry |
| `urdf/common/materials.xacro` | Visual colours |
| `urdf/components/*.xacro` | Chassis and base frames, drive wheel (`side:=left/right`), caster, mast, fork carriage with the prismatic lift joint and forks, camera |
| `urdf/components/appearance.xacro` | Visual-only body details (counterweight, bumpers, LiDAR tower, beacon, e-stop, scanners, hubs, lift cylinder, decals); sizes in `appearance` (DECISIONS D-54) |
| `scripts/generate_decals.py` | Build step: textured OBJ decals (hazard stripe, side label) → `share/forklift_description/meshes/decals` |
| `urdf/gazebo/forklift.gazebo.xacro` | Simulation-only content. It is included only when `use_sim:=true`, and it is provided by Phase 3 |
| `launch/display.launch.py` | Starts robot_state_publisher, joint_state_publisher(_gui) and RViz |
| `rviz/forklift.rviz` | RViz layout: robot model and TF. Camera image panels are present but disabled |

## Xacro arguments

| Argument | Default | Meaning |
|---|---|---|
| `prefix` | `""` | Prefix added to every frame and joint name |
| `use_sim` | `false` | Include `gazebo/forklift.gazebo.xacro` |
| `params_file` | `$(find forklift_description)/config/forklift_params.yaml` | Parameter file |

```bash
xacro $(ros2 pkg prefix forklift_description)/share/forklift_description/urdf/forklift.urdf.xacro > forklift.urdf
```

## Usage

```bash
ros2 launch forklift_description display.launch.py                  # sliders + RViz
ros2 launch forklift_description display.launch.py gui:=false fork_height:=0.06
```

The launch arguments are `gui`, `rviz`, `fork_height` (in metres; it works only with an empty prefix), `prefix`, `params_file` and `rviz_config`.

## Frames

The frame tree follows REP-105. `base_footprint` is the root and has no inertial. `base_link` sits at the drive-axle centre, 0.05 m above the floor.

```
base_footprint → base_link → {chassis_link, left/right_wheel_link, caster_link,
                              mast_link → {fork_carriage_link → left/right_fork_link,
                                           front_camera_link → front_camera_optical_frame,
                                           lane_camera_link  → lane_camera_optical_frame}}
```

The `<name>_optical_frame` frames are rotated by rpy = (−π/2, 0, −π/2) from `<name>_link`. That gives z forward, x right and y down. Image messages use the optical frame as their `frame_id`.

## Notes

- The `appearance` parts add visuals only: links, joints, inertials and collisions are the same as without them, and every part is behind the camera optical centres (x = 0.10), so the robot's own images do not change (DECISIONS D-54). `package.xml` exports `gazebo_model_path="${prefix}/.."` so Gazebo can resolve the decal meshes (`package://` → `model://forklift_description/...`).
- `<collision>` elements are deliberately left unnamed. When sdformat lumps fixed joints, it matches per-link friction such as the caster's μ = 0 through the collision names it generates. A custom collision name breaks that match.
- The drive-wheel **collision** is a sphere of r = 0.05 m (`drive_wheel.collision_shape: sphere`). The visual and the inertia stay the specified cylinder. A 3 cm cylinder touches the floor at two rim points; during in-place turns those points scrub against each other and the robot turned 9 % more than odometry measured (DECISIONS D-20). Set `collision_shape: cylinder` to get the literal shape back.
- The fork controller runs in PID force mode. In velocity mode DART does not lift the fork off its lower limit (DECISIONS D-10).
- `camera.xacro` builds only the frames, the camera body and its mass. The Gazebo sensor block reads the same YAML entry (FOV, resolution, noise, clip) inside `forklift.gazebo.xacro`.
- Design decisions are recorded in [`../../DECISIONS.md`](../../DECISIONS.md). The architecture is described in [`../../ARCHITECTURE.md`](../../ARCHITECTURE.md).
