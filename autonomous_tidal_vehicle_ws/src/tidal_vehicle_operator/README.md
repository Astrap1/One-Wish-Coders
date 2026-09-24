# `tidal_vehicle_operator`

Owns the operator-facing view of the live mission. Start with RViz/Foxglove-compatible ROS topics before building a custom dashboard.

The minimum display includes vehicle pose, planned route, LiDAR obstacles, battery/return reserve, tide risk, payload status and safety state. The interface must offer a goal-selection workflow and a visible emergency abort action.
