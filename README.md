# Vision-Guided Forklift

A small differential-drive forklift robot in simulation, built as the platform
for a computer-vision learning project. The workspace contains a
simulator-agnostic robot description, a Gazebo Harmonic test bench with
automated acceptance tests, a velocity-command safety chain and a Qt 6 / QML
operator GUI.

Perception, lane following and autonomy are the next steps and are not part of
this repository yet.

![Operator GUI connected to the simulation](docs/images/gui_main_sim.png)

## Environment

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim 8 (Harmonic, ROS vendor packages),
system Qt 6.4.2. Exact versions and the apt install list are in
[ARCHITECTURE.md §7.1](ARCHITECTURE.md).

## Packages

| Package | Purpose |
|---|---|
| [`forklift_description`](src/forklift_description) | Modular xacro model (chassis, drive wheels, caster, mast, prismatic fork, two cameras). Every number lives in one YAML file. |
| [`forklift_gazebo`](src/forklift_gazebo) | Test world, sealed test parcel, ROS ↔ Gazebo bridges, `sim.launch.py` and the T1–T14 acceptance test. |
| [`forklift_safety`](src/forklift_safety) | Command chain: `twist_mux` arbitration and the `cmd_vel_gate` fail-safe node. Started by `sim.launch.py`. |
| [`forklift_gui`](src/forklift_gui) | Operator GUI: C++17 core (ports and adapters, no ROS in core), ROS and mock adapters, QML view. |
| [`forklift_bringup`](src/forklift_bringup) | One launch file for simulation + RViz + GUI. |

## Build

```bash
mkdir -p ~/cv_forklift_ws && cd ~/cv_forklift_ws
git clone git@github.com:muhammet-alaybey-cilesiz/vision-guided-forklift.git .
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# Simulation + RViz + command chain + operator GUI
ros2 launch forklift_bringup bringup.launch.py

# Simulation only (also starts the command chain)
ros2 launch forklift_gazebo sim.launch.py

# GUI without ROS: mock robot, synthetic cameras and logs
ros2 launch forklift_gui gui.launch.py mock:=true

# Robot model with joint sliders in RViz
ros2 launch forklift_description display.launch.py
```

## Tests

```bash
colcon test && colcon test-result --verbose          # unit tests + linters
ros2 run forklift_gazebo acceptance_test.py           # simulation T1–T14 (~5 min)
```

Results and evidence: [docs/test_results.md](docs/test_results.md),
[docs/gui_test_results.md](docs/gui_test_results.md), `docs/logs/`,
`docs/images/`.

## Documentation

- [ARCHITECTURE.md](ARCHITECTURE.md) — frames, ROS ↔ Gazebo interface, launch graph, GUI layers and thread model
- [DECISIONS.md](DECISIONS.md) — design decision log (D-01 … D-42)
- [REPORT.md](REPORT.md) — simulation phase report (Turkish)
