# Contributing

## Working agreement

- Keep each change within one workstream where possible.
- Do not change another workstream's public ROS topic, message or launch contract without recording the change in `docs/INTERFACES.md`.
- Use a short-lived branch and a focused pull request for each feature.
- Do not commit `build/`, `install/`, `log/`, rosbag recordings or generated results.

## Definition of done

A feature is ready to merge when it:

1. launches or runs through its documented entry point;
2. has a stated input/output contract in `docs/INTERFACES.md` when it communicates with another subsystem;
3. does not break the normal delivery, blocked-route or unsafe-return scenario; and
4. has been demonstrated to the integration owner.

## Integration ownership

The vehicle-simulation and integration workstream owns the complete-system launch path. Other workstreams should provide independently runnable nodes, configuration and a short README describing how to integrate them.
