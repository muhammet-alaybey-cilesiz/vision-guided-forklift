# forklift_safety

`cmd_vel_gate` is a fail-safe stop that sits between twist_mux and the
DiffDrive bridge:

```
twist_mux ──/cmd_vel/mux──▶ cmd_vel_gate ──/cmd_vel──▶ ros_gz_bridge ──▶ gz DiffDrive
             /e_stop ──────▶
```

## Why it exists

Two behaviours of the installed stack combine badly (DECISIONS D-24):

* **twist_mux only masks inputs.** When a lock is active or has timed out, it
  stops forwarding messages but never publishes a zero.
* **gz-sim DiffDrive keeps the last command forever.** It has no command
  timeout.

So without this node, an e-stop or a crashed GUI would leave the robot driving
at its last speed. The gate turns "nothing is allowed" into "zero commanded".

## Behaviour

| Condition | Output on `output_topic` |
|---|---|
| `/e_stop` false and fresh, input arriving | the input, forwarded unchanged |
| `/e_stop` rising edge | one zero Twist **at once** |
| `/e_stop` true, older than `e_stop_timeout`, or never received | zero at `zero_rate`; inputs are dropped |
| No input for `input_timeout` | zero at `zero_rate` |

All times use the wall clock (`use_sim_time: false`). The node declares no
defaults. Parameters, including the topic names, come from
`config/cmd_vel_gate.yaml`.

## Command chain

This package also holds the whole chain (DECISIONS D-42):

| File | Purpose |
|---|---|
| `config/twist_mux.yaml` | twist_mux 4.5.0 inputs, `/e_stop` lock and `use_stamped: false` (D-23) |
| `config/cmd_vel_gate.yaml` | Gate topics and timeouts |
| `launch/control.launch.py` | twist_mux + cmd_vel_gate; twist_mux's output is remapped to the gate's `input_topic` |

`forklift_gazebo/sim.launch.py` includes `control.launch.py` by default
(`control:=true`). The gate thus lives with the robot, not with the GUI: if the GUI
is closed or crashes, the gate keeps publishing zeros. To run the chain alone:

```bash
ros2 launch forklift_safety control.launch.py
```

## Tests

`test/test_gate_logic.cpp` (gtest) checks the ROS-free state machine
`GateLogic`. The end-to-end behaviour is measured by `forklift_gui`'s
integration tests:

* **I4:** e-stop while moving → zero on `/cmd_vel` within 100 ms.
* **I5:** `kill -9` of the GUI while driving → robot stopped within 0.6 s.
