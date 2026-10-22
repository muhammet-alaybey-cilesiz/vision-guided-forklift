# forklift_interfaces

Messages and services between perception, autonomy and the operator GUI. Every perception
message carries the source image stamp and is expressed in `base_footprint` on the ground plane,
so a learned detector can replace a classical one without changing its users (plan rule R4).
Lanes are polylines, so curved routes will not change the message. `SetMode` and
`AutonomyState` serve the GUI's Manual / Auto mode panel (D-57).
