from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    package_share = FindPackageShare("tidal_vehicle_simulation")
    gz_share = FindPackageShare("ros_gz_sim")
    models_path = PathJoinSubstitution([package_share, "models"])
    plugin_path = PathJoinSubstitution([package_share, "..", "..", "lib"])
    world_path = PathJoinSubstitution([package_share, "worlds", "tidal_corridor.sdf"])
    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", [models_path, ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")]),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", [plugin_path, ":", EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value="")]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([gz_share, "launch", "gz_sim.launch.py"])),
            launch_arguments={"gz_args": [world_path, " -r"]}.items(),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="scenario_event_bridge",
            output="screen",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/scenario_event@std_msgs/msg/String]gz.msgs.StringMsg",
            ],
        ),
        Node(
            package="tidal_vehicle_simulation",
            executable="tide_manager.py",
            name="tide_manager",
            output="screen",
            parameters=[PathJoinSubstitution([package_share, "config", "scenario_defaults.yaml"])],
        ),
    ])
