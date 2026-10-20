# worlds

| World | Source |
|---|---|
| `lift_test.sdf` | Hand-written, in this directory |
| `station_test.sdf` | Hand-written, in this directory; stations and parcels are spawned at launch from `config/stations.yaml` |
| `factory.sdf` | **Generated at build time, not stored here.** Source: [`config/factory.yaml`](../config/factory.yaml) (+ `stations.yaml`, `station_params.yaml`), generator: [`scripts/generate_factory_world.py`](../scripts/generate_factory_world.py). After `colcon build` it is at `install/forklift_gazebo/share/forklift_gazebo/worlds/factory.sdf` |

`factory.sdf` is derived data: the guide lanes and stop bars follow the station
poses, and the same build step writes the lane ground truth
(`config/factory_lanes.yaml`). Edit `factory.yaml` and rebuild; do not edit the
generated file (DECISIONS D-53).
