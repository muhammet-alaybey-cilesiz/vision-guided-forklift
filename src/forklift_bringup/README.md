# forklift_bringup

This package holds only `launch/bringup.launch.py`, which starts the whole
system. The command chain (twist_mux + cmd_vel_gate) and its configuration
live in `forklift_safety`. `forklift_gazebo/sim.launch.py` starts them, so the
GUI can drive even when the simulation and the GUI run in separate terminals
(DECISIONS D-42).

```bash
ros2 launch forklift_bringup bringup.launch.py                 # Gazebo window + RViz + chain + GUI
ros2 launch forklift_bringup bringup.launch.py sim_gui:=false rviz:=false   # headless cameras, no RViz
ros2 launch forklift_bringup bringup.launch.py gui:=false      # without the operator GUI

# Equivalent in two terminals
ros2 launch forklift_gazebo sim.launch.py rviz:=true
ros2 launch forklift_gui gui.launch.py
```

| Argument | Default | Meaning |
|---|---|---|
| `sim` | `true` | Run `forklift_gazebo/sim.launch.py` |
| `sim_gui` | `true` | Open the Gazebo client window (`false`: the cameras render headless with EGL) |
| `rviz` | `true` | Run RViz |
| `control` | `true` | Run twist_mux + cmd_vel_gate (passed to `sim.launch.py`; needs `sim:=true`) |
| `gui` | `true` | Run the operator GUI |
| `gui_params` | `forklift_gui/config/gui_params.yaml` | GUI parameter file |

Each included launch file runs in a scoped group. Without the scope,
`IncludeLaunchDescription` writes the arguments it passes (here `gui:=<sim_gui>`
for the simulation) into the parent context. That overwrote this file's own
`gui` argument, and the operator GUI never started.

## Command chain

| Input | Priority | Timeout |
|---|---|---|
| `/cmd_vel/teleop` (GUI) | 100 | 0.5 s |
| `/cmd_vel/auto` (future autonomy) | 50 | 0.5 s |
| lock `/e_stop` | 255 | 0.5 s |

twist_mux publishes on `/cmd_vel/mux`; cmd_vel_gate then publishes on
`/cmd_vel`, which the DiffDrive bridge reads.

**`use_stamped: false` is required.** Without it, twist_mux 4.5.0 uses
`TwistStamped`, but the bridge expects `geometry_msgs/Twist`. The robot then
does not move, and no error is reported (DECISIONS D-23).

Topic names in `forklift_safety/config/` must match `forklift_gui/config/gui_params.yaml`
(`topics.cmd_vel`, `topics.e_stop`).
