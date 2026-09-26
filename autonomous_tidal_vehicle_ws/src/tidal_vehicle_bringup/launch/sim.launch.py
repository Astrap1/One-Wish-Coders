"""sim.launch.py — the one-command launch path (vehicle simulation & integration).

    ros2 launch tidal_vehicle_bringup sim.launch.py                        # Version 2 in the tidal corridor
    ros2 launch tidal_vehicle_bringup sim.launch.py vehicle:=v1            # Version 1 fallback
    ros2 launch tidal_vehicle_bringup sim.launch.py vehicle:=v3 cameras:=all   # Version 3 (in development)
    ros2 launch tidal_vehicle_bringup sim.launch.py world:=vehicle_tests/integration_test tide:=false
    ros2 launch tidal_vehicle_bringup sim.launch.py headless:=true foxglove:=true gpu:=nvidia
    ros2 launch tidal_vehicle_bringup sim.launch.py dashboard:=true

Starts:
  * Gazebo Harmonic with `world` (path relative to tidal_vehicle_simulation/worlds, no .sdf)
  * the `vehicle` (v2: hovercraft_v2, tracked; v1: hovercraft, wheeled; v3: hovercraft_v3,
    high-speed series hybrid, not yet the demo vehicle), spawned at HOME
    unless the world already includes it
  * tide:=true (default) starts Person 3's tide_manager (/terrain_state and the changing
    /terrain_costmap); tide:=false starts the static integration cost map instead
  * ros_gz_bridge (config/ros_gz_bridge.yaml) -> /odom /tf /joint_states /points /imu /gps/fix
    /camera/image_raw + vehicle-internal /vehicle/* topics
  * robot_state_publisher (URDF from tidal_vehicle_description) -> /robot_description, joint TFs
  * lidar_scan_node       /points -> /scan
  * vehicle_mobility_node /cmd_vel -> hover fans or wheels; /vehicle_health
  * tide_manager           dynamic terrain state and costmap for tidal_corridor
    (or terrain_costmap_node for explicit non-tidal integration worlds)
  * global_planner + path_follower + safety_supervisor
  * foxglove_bridge on ws://0.0.0.0:8765 (Foxglove Desktop on Windows: ws://localhost:8765)
  * browser dashboard on http://localhost:8000 when dashboard:=true
  * RViz (optional)

Only the safety supervisor publishes /cmd_vel. Autonomy publishes proposals on
/cmd_vel_proposed.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _prepend(var, *paths):
    old = os.environ.get(var, "")
    return ":".join([str(p) for p in paths] + ([old] if old else []))


# Per-vehicle files. Topic names on the ROS side are identical for all vehicles;
# Version 3 adds /vehicle/collision, fuel_percent and the extra camera topics.
VEHICLES = {
    "v1": {"model": "hovercraft", "urdf": "hovercraft.urdf",
           "params": "vehicle_mobility.yaml", "bridge": "ros_gz_bridge.yaml",
           "obstacle_inflation_radius_m": 0.75},
    "v2": {"model": "hovercraft_v2", "urdf": "hovercraft_v2.urdf",
           "params": "vehicle_mobility_v2.yaml", "bridge": "ros_gz_bridge_v2.yaml",
           "obstacle_inflation_radius_m": 1.7},
    # Version 3 (in development; AGENTS.md "Vehicle Version 3"). Its zone speed
    # limits (split cost bands) also go to the path follower and Safety.
    "v3": {"model": "hovercraft_v3", "urdf": "hovercraft_v3.urdf",
           "params": "vehicle_mobility_v3.yaml", "bridge": "ros_gz_bridge_v3.yaml",
           "cameras_bridge": "ros_gz_bridge_v3_cameras.yaml",
           "obstacle_inflation_radius_m": 2.0,
           "planner": {"obstacle_min_range_m": 8.0, "obstacle_max_range_m": 30.0,
                       "obstacle_brake_decel_mps2": 1.0,
                       "obstacle_reaction_time_s": 1.0,
                       "goal_event_tolerance_m": 0.3},
           "follower": {"max_linear_speed": 13.9, "max_angular_speed": 0.45,
                        "lookahead_distance": 3.0, "heading_gain": 0.5,
                        "rotate_in_place_angle": 0.70,
                        "slow_down_distance": 8.0,
                        "zone_speed_limits_mps": [4.2, 13.9, 2.8, 2.8],
                        "brake_decel_mps2": 1.0, "lateral_accel_limit_mps2": 0.7,
                        "yaw_rate_damping": 0.6,
                        "corner_preview_sample_distance_m": 3.0,
                        "turn_brake_angle_rad": 0.45,
                        "turn_brake_speed_mps": 0.8,
                        "lookahead_time_s": 1.8, "reaction_time_s": 1.0,
                        "obstacle_detection_range_m": 30.0,
                        "obstacle_clearance_m": 2.0},
           "safety": {"zone_speed_limits_mps": [4.2, 13.9, 2.8, 2.8], "brake_decel_mps2": 1.0,
                      "cruise_linear_speed_limit_mps": 13.9,
                      "caution_linear_speed_limit_mps": 2.8,
                      "return_linear_speed_limit_mps": 8.3},
           "collision_monitor": True},
}
SPAWN_Z = {"tidal_corridor": 0.25}     # 0.25 m above the HOME plateau at z = 0 m


def _setup(context):
    world = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    gpu = LaunchConfiguration("gpu").perform(context).lower()
    vehicle = LaunchConfiguration("vehicle").perform(context)
    if vehicle not in VEHICLES:
        raise RuntimeError(f"vehicle must be one of {sorted(VEHICLES)}, got {vehicle!r}")
    if gpu not in {"auto", "nvidia"}:
        raise RuntimeError("gpu must be 'auto' or 'nvidia'")
    veh = VEHICLES[vehicle]
    bringup = Path(get_package_share_directory("tidal_vehicle_bringup"))
    sim_share = Path(get_package_share_directory("tidal_vehicle_simulation"))
    desc_share = Path(get_package_share_directory("tidal_vehicle_description"))
    safety_share = Path(get_package_share_directory("tidal_vehicle_safety"))
    sim_lib = Path(get_package_prefix("tidal_vehicle_simulation")) / "lib"

    world_file = sim_share / "worlds" / f"{world}.sdf"
    world_name = Path(world).name                        # <world name="..."> == file stem
    bridge_cfg = Path("/tmp") / f"tidal_bridge_{world_name}_{vehicle}.yaml"
    bridge_cfg.write_text((bringup / "config" / veh["bridge"]).read_text()
                          .replace("WORLD", world_name))
    cameras = LaunchConfiguration("cameras").perform(context).lower()
    if cameras not in {"front", "all"}:
        raise RuntimeError("cameras must be 'front' or 'all'")
    extra_bridge = []
    if cameras == "all" and "cameras_bridge" in veh:
        # A camera is only rendered while something subscribes to it, so the
        # extra cameras cost nothing unless they are bridged.
        cams_cfg = Path("/tmp") / f"tidal_bridge_cameras_{world_name}_{vehicle}.yaml"
        cams_cfg.write_text((bringup / "config" / veh["cameras_bridge"]).read_text()
                            .replace("WORLD", world_name))
        extra_bridge = [Node(package="ros_gz_bridge", executable="parameter_bridge",
                             name="ros_gz_bridge_cameras", output="screen",
                             parameters=[{"config_file": str(cams_cfg), "use_sim_time": True}])]
    urdf = (desc_share / "urdf" / veh["urdf"]).read_text()
    params = str(sim_share / "config" / veh["params"])
    safety_params = str(safety_share / "config" / "safety_params.yaml")
    scenario_config = ("scenario_5deg.yaml" if world_name == "tidal_corridor_5deg"
                       else "scenario_defaults.yaml")
    scenario_params = str(sim_share / "config" / scenario_config)
    tide = LaunchConfiguration("tide").perform(context).lower() in ("true", "1", "yes")
    # Spawn the vehicle at HOME (0, 0) unless the world file already includes it.
    spawn = f"<uri>model://{veh['model']}</uri>" not in world_file.read_text()

    gz_cmd = ["gz", "sim", "-r", str(world_file)]
    # empty mode_policy = the vehicle's own default (v1: hover_only, v2: terrain_auto)
    policy = LaunchConfiguration("mode_policy").perform(context)
    mode_policy = {"mode_policy": policy} if policy else {}
    if headless:
        gz_cmd[2:2] = ["-s", "--headless-rendering"]

    gpu_environment = []
    if gpu == "nvidia":
        # WSLg otherwise may select llvmpipe (software rendering) even when an
        # NVIDIA adapter is available. Keep this opt-in: some GUI driver stacks
        # have their own adapter policy, while the headless demo is validated
        # with this D3D12 path.
        gpu_environment = [
            SetEnvironmentVariable("GALLIUM_DRIVER", "d3d12"),
            SetEnvironmentVariable("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA"),
        ]

    return [
        *gpu_environment,
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", _prepend(
            "GZ_SIM_RESOURCE_PATH", desc_share / "models", sim_share / "models", sim_share / "worlds")),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", _prepend(
            "GZ_SIM_SYSTEM_PLUGIN_PATH", sim_lib)),
        ExecuteProcess(cmd=gz_cmd, output="screen"),
        *([Node(package="ros_gz_sim", executable="create", output="screen",
                arguments=["-world", world_name, "-name", veh["model"],
                           "-file", str(desc_share / "models" / veh["model"] / "model.sdf"),
                           "-x", "0", "-y", "0", "-z", str(next((z for w, z in SPAWN_Z.items() if world_name.startswith(w)), 0.002))])]
          if spawn else []),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="ros_gz_bridge",
             output="screen", parameters=[{"config_file": str(bridge_cfg), "use_sim_time": True}]),
        *extra_bridge,
        *([Node(package="tidal_vehicle_simulation", executable="collision_monitor_node.py",
                name="collision_monitor", output="screen", parameters=[params])]
          if veh.get("collision_monitor") else []),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": urdf, "use_sim_time": True}]),
        Node(package="tidal_vehicle_simulation", executable="lidar_scan_node.py", name="lidar_scan",
             output="screen", parameters=[params]),
        Node(package="tidal_vehicle_simulation", executable="vehicle_mobility_node.py",
             name="vehicle_mobility", output="screen",
             parameters=[params, mode_policy,
                         # Tide manager is the only terrain-state publisher in the
                         # tidal corridor. Keep the placeholder for test worlds.
                         {"publish_terrain_state": not tide}]),
        (Node(package="tidal_vehicle_simulation", executable="tide_manager.py",
              name="tide_manager", output="screen", parameters=[scenario_params])
         if tide else
         Node(package="tidal_vehicle_simulation", executable="terrain_costmap_node.py",
              name="terrain_costmap", output="screen", parameters=[params])),
        Node(package="tidal_vehicle_autonomy", executable="global_planner",
             name="global_planner", output="screen",
             parameters=[{"use_sim_time": True,
                          "obstacle_inflation_radius_m":
                              veh["obstacle_inflation_radius_m"],
                          **veh.get("planner", {})}]),
        Node(package="tidal_vehicle_autonomy", executable="path_follower",
             name="path_follower", output="screen",
             parameters=[{"use_sim_time": True, **veh.get("follower", {})}]),
        Node(package="tidal_vehicle_safety", executable="safety_supervisor",
             name="safety_supervisor", output="screen",
             # Version 2 is hover-first; tracks deploy only for an exceptional
             # steep firm-land climb, which a 2D cost map cannot infer. Use the
             # conservative hover profile for every ordinary route segment.
             parameters=[safety_params, {"use_sim_time": True,
                                         "track_mode_enabled": False,
                                         **veh.get("safety", {})}]),
        Node(package="foxglove_bridge", executable="foxglove_bridge", output="screen",
             parameters=[{"port": 8765, "address": "0.0.0.0", "use_sim_time": True}],
             condition=IfCondition(LaunchConfiguration("foxglove"))),
        Node(package="tidal_vehicle_operator", executable="browser_dashboard", output="screen",
             parameters=[{"use_sim_time": True}],
             condition=IfCondition(LaunchConfiguration("dashboard"))),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", str(desc_share / "rviz" / "hovercraft.rviz")],
             parameters=[{"use_sim_time": True}],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="tidal_corridor",
                              description="world under tidal_vehicle_simulation/worlds, without .sdf"),
        DeclareLaunchArgument("vehicle", default_value="v2",
                              description="v2 (tracked, the demo vehicle) | v1 (wheeled fallback) "
                                          "| v3 (high-speed, in development)"),
        DeclareLaunchArgument("cameras", default_value="front",
                              description="front | all (Version 3: also the rear, left and "
                                          "right cameras)"),
        DeclareLaunchArgument("tide", default_value="true",
                              description="run the tide manager instead of the static integration map"),
        DeclareLaunchArgument("mode_policy", default_value="",
                              description="hover_only | terrain_auto; empty = vehicle default "
                                          "(v1 hover_only, v2 terrain_auto)"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("gpu", default_value="auto",
                              description="auto | nvidia (WSLg D3D12 NVIDIA adapter)"),
        DeclareLaunchArgument("foxglove", default_value="true"),
        DeclareLaunchArgument("dashboard", default_value="false",
                              description="start the browser operator dashboard on port 8000"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
