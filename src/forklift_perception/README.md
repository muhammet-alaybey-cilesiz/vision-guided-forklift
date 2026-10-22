# forklift_perception

Camera perception for the forklift. A ROS-free core (`forklift_perception/core`) holds the
camera model and the detectors; two thin nodes publish their results. The robot uses only
what a real robot has: images, `camera_info` (or a calibration file), TF and the fork joint.
It never reads simulator poses or the world layout files (DECISIONS D-57, test C0).

| Node | Camera | Publishes |
|---|---|---|
| `front_perception` | `/front_camera` | `/perception/signs`, `/perception/parcels`, `/cv/detections` (GUI overlay) |
| `lane_perception` | `/lane_camera` | `/perception/bay_targets` (bullseye pairs and bay labels), `/perception/lanes`, `/perception/load` |

```bash
ros2 launch forklift_perception perception.launch.py [debug:=true]
```

* `config/perception.yaml`: ROS parameters (topics, frames, files); no defaults in code (D-29).
* `config/detectors.yaml`: detector thresholds, shared by the nodes and the offline evaluation.
  Colour thresholds were measured from ground-truth regions (`forklift_evaluation color_stats`).
* `config/calibration/sim_<camera>.yaml`: camera intrinsics from a checkerboard calibration
  (`forklift_evaluation calibrate_cameras`, D-59). On a real robot, put the real calibration here.
* Station geometry comes from the station catalog (`station_params.yaml`: sizes and colours, no
  positions); `core/catalog.py` repeats the formulas of `station_layout.py`, and
  `forklift_evaluation/test/test_catalog_consistency.py` keeps them equal (D-63).
