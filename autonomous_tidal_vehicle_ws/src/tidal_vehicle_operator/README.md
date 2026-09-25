# `tidal_vehicle_operator`

Owns the operator-facing view of the live mission. Start with RViz/Foxglove-compatible ROS topics before building a custom dashboard.

The minimum display includes vehicle pose, outbound and return routes, LiDAR
obstacles, battery, Safety-calculated return energy/margin/ETA, tide risk,
payload status and safety state/reason. The interface must offer a goal-selection
workflow and a visible emergency abort action.
