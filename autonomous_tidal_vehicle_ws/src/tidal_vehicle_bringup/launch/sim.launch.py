"""sim.launch.py — the one-command launch path (vehicle simulation & integration).

    ros2 launch tidal_vehicle_bringup sim.launch.py                        # Version 2 in the tidal corridor
    ros2 launch tidal_vehicle_bringup sim.launch.py vehicle:=v1            # Version 1 fallback
    ros2 launch tidal_vehicle_bringup sim.launch.py world:=vehicle_tests/integration_test tide:=false
    ros2 launch tidal_vehicle_bringup sim.launch.py headless:=true foxglove:=true

Starts:
  * Gazebo Harmonic with `world` (path relative to tidal_vehicle_simulation/worlds, no .sdf)
  * the `vehicle` (v2: hovercraft_v2, tracked; v1: hovercraft, wheeled), spawned at HOME
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


# Per-vehicle files. Topic names on the ROS side are identical for both vehicles.
VEHICLES = {
    "v1": {"model": "hovercraft", "urdf": "hovercraft.urdf",
           "params": "vehicle_mobility.yaml", "bridge": "ros_gz_bridge.yaml"},
    "v2": {"model": "hovercraft_v2", "urdf": "hovercraft_v2.urdf",
           "params": "vehicle_mobility_v2.yaml", "bridge": "ros_gz_bridge_v2.yaml"},
}
SPAWN_Z = {"tidal_corridor": 0.25}     # drop height above z = 0 (the corridor ground is uneven)


def _setup(context):
    world = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    vehicle = LaunchConfiguration("vehicle").perform(context)
    if vehicle not in VEHICLES:
        raise RuntimeError(f"vehicle must be one of {sorted(VEHICLES)}, got {vehicle!r}")
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
    urdf = (desc_share / "urdf" / veh["urdf"]).read_text()
    params = str(sim_share / "config" / veh["params"])
    safety_params = str(safety_share / "config" / "safety_params.yaml")
    scenario_params = str(sim_share / "config" / "scenario_defaults.yaml")
    tide = LaunchConfiguration("tide").perform(context).lower() in ("true", "1", "yes")
    # Spawn the vehicle at HOME (0, 0) unless the world file already includes it.
    spawn = f"<uri>model://{veh['model']}</uri>" not in world_file.read_text()

    gz_cmd = ["gz", "sim", "-r", str(world_file)]
    # empty mode_policy = the vehicle's own default (v1: hover_only, v2: terrain_auto)
    policy = LaunchConfiguration("mode_policy").perform(context)
    mode_policy = {"mode_policy": policy} if policy else {}
    if headless:
        gz_cmd[2:2] = ["-s", "--headless-rendering"]

    return [
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
             name="global_planner", output="screen", parameters=[{"use_sim_time": True}]),
        Node(package="tidal_vehicle_autonomy", executable="path_follower",
             name="path_follower", output="screen", parameters=[{"use_sim_time": True}]),
        Node(package="tidal_vehicle_safety", executable="safety_supervisor",
             name="safety_supervisor", output="screen",
             # Version 2 is hover-first; tracks deploy only for an exceptional
             # steep firm-land climb, which a 2D cost map cannot infer. Use the
             # conservative hover profile for every ordinary route segment.
             parameters=[safety_params, {"use_sim_time": True,
                                         "track_mode_enabled": False}]),
        Node(package="foxglove_bridge", executable="foxglove_bridge", output="screen",
             parameters=[{"port": 8765, "address": "0.0.0.0", "use_sim_time": True}],
             condition=IfCondition(LaunchConfiguration("foxglove"))),
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
                              description="v2 (tracked, the demo vehicle) | v1 (wheeled fallback)"),
        DeclareLaunchArgument("tide", default_value="true",
                              description="run the tide manager instead of the static integration map"),
        DeclareLaunchArgument("mode_policy", default_value="",
                              description="hover_only | terrain_auto; empty = vehicle default "
                                          "(v1 hover_only, v2 terrain_auto)"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("foxglove", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
