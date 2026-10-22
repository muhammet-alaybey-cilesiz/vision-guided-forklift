# forklift_evaluation

Simulation-only evaluation. This is the only package that reads ground truth (Gazebo poses and
the generated layout files); it is not installed on a real robot (DECISIONS D-57, R8).

| Tool | Purpose |
|---|---|
| `f1_probe` | Ground-truth projection overlays, occlusion and visibility limits, odometry source, recording of evaluation sets (`--tasks record --sets f1|f2 [--resume]`) |
| `calibrate_cameras` | Checkerboard calibration of both robot cameras in the simulation (D-59) |
| `color_stats` | HSV distributions of ground-truth regions (colour threshold calibration) |
| `cv_eval` | Acceptance tests C0–C7, C9 on the recorded sets, optional image noise |
| `perf_probe` | C8: live rates, RTF, CPU, subscriptions (C0 runtime part), optional GUI screenshot |

Recorded sets live in `~/cv_forklift_ws/data/` (not in git; reproducible with `f1_probe`).
