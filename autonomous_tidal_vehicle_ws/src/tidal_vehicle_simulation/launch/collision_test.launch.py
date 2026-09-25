from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    share=FindPackageShare("tidal_vehicle_simulation")
    gz=FindPackageShare("ros_gz_sim")
    models=PathJoinSubstitution([share,"models"])
    plugins=PathJoinSubstitution([share,"..","..","lib"])
    world=PathJoinSubstitution([share,"worlds","tidal_corridor.sdf"])
    test_model=PathJoinSubstitution([share,"models","collision_test_body","model.sdf"])
    bridge=Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen", arguments=["/collision_test/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"])
    spawn=Node(package="ros_gz_sim", executable="create", output="screen", arguments=["-world","tidal_corridor","-file",test_model,"-name","collision_test_body"])
    return LaunchDescription([
      SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", [models,":",EnvironmentVariable("GZ_SIM_RESOURCE_PATH",default_value="")]),
      SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", [plugins,":",EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH",default_value="")]),
      SetEnvironmentVariable("TIDE_VISUAL_DISABLED", "1"),
      IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([gz,"launch","gz_sim.launch.py"])), launch_arguments={"gz_args":[world," -r"]}.items()),
      TimerAction(period=2.0, actions=[spawn]),
      TimerAction(period=5.0, actions=[bridge]),
    ])
