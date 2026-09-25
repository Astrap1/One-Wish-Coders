"""sim.launch.py — the one-command launch path (vehicle simulation & integration).

    ros2 launch tidal_vehicle_bringup sim.launch.py
    ros2 launch tidal_vehicle_bringup sim.launch.py world:=vehicle_tests/debris_test rviz:=true
    ros2 launch tidal_vehicle_bringup sim.launch.py headless:=true foxglove:=true

Starts:
  * Gazebo Harmonic with `world` (path relative to tidal_vehicle_simulation/worlds, no .sdf)
  * ros_gz_bridge (config/ros_gz_bridge.yaml) -> /odom /tf /joint_states /points /imu /gps/fix
    /camera/image_raw + vehicle-internal /vehicle/* topics
  * robot_state_publisher (URDF from tidal_vehicle_description) -> /robot_description, joint TFs
  * lidar_scan_node       /points -> /scan
  * vehicle_mobility_node /cmd_vel -> hover fans or wheels; /vehicle_health; placeholder /terrain_state
  * terrain_costmap_node   static integration map until Person 3's tide manager is ready
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


def _setup(context):
    world = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    bringup = Path(get_package_share_directory("tidal_vehicle_bringup"))
    sim_share = Path(get_package_share_directory("tidal_vehicle_simulation"))
    desc_share = Path(get_package_share_directory("tidal_vehicle_description"))
    safety_share = Path(get_package_share_directory("tidal_vehicle_safety"))
    sim_lib = Path(get_package_prefix("tidal_vehicle_simulation")) / "lib"

    world_file = sim_share / "worlds" / f"{world}.sdf"
    world_name = Path(world).name                        # <world name="..."> == file stem
    bridge_cfg = Path("/tmp") / f"tidal_bridge_{world_name}.yaml"
    bridge_cfg.write_text((bringup / "config" / "ros_gz_bridge.yaml").read_text()
                          .replace("WORLD", world_name))
    urdf = (desc_share / "urdf" / "hovercraft.urdf").read_text()
    params = str(sim_share / "config" / "vehicle_mobility.yaml")
    safety_params = str(safety_share / "config" / "safety_params.yaml")

    gz_cmd = ["gz", "sim", "-r", str(world_file)]
    if headless:
        gz_cmd[2:2] = ["-s", "--headless-rendering"]

    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", _prepend(
            "GZ_SIM_RESOURCE_PATH", desc_share / "models", sim_share / "models", sim_share / "worlds")),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", _prepend(
            "GZ_SIM_SYSTEM_PLUGIN_PATH", sim_lib)),
        ExecuteProcess(cmd=gz_cmd, output="screen"),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="ros_gz_bridge",
             output="screen", parameters=[{"config_file": str(bridge_cfg), "use_sim_time": True}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": urdf, "use_sim_time": True}]),
        Node(package="tidal_vehicle_simulation", executable="lidar_scan_node.py", name="lidar_scan",
             output="screen", parameters=[params]),
        Node(package="tidal_vehicle_simulation", executable="vehicle_mobility_node.py",
             name="vehicle_mobility", output="screen",
             parameters=[params, {"mode_policy": LaunchConfiguration("mode_policy")}]),
        Node(package="tidal_vehicle_simulation", executable="terrain_costmap_node.py",
             name="terrain_costmap", output="screen", parameters=[params]),
        Node(package="tidal_vehicle_autonomy", executable="global_planner",
             name="global_planner", output="screen", parameters=[{"use_sim_time": True}]),
        Node(package="tidal_vehicle_autonomy", executable="path_follower",
             name="path_follower", output="screen", parameters=[{"use_sim_time": True}]),
        Node(package="tidal_vehicle_safety", executable="safety_supervisor",
             name="safety_supervisor", output="screen",
             parameters=[safety_params, {"use_sim_time": True}]),
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
        DeclareLaunchArgument("world", default_value="vehicle_tests/integration_test",
                              description="world under tidal_vehicle_simulation/worlds, without .sdf"),
        DeclareLaunchArgument("mode_policy", default_value="hover_only",
                              description="hover_only | terrain_auto"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("foxglove", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
