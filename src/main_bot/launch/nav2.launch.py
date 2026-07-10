import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# teb_local_planner (one of the four controller_server FollowPath plugins
# selectable via params_file below) links against g2o. There's no
# ros-jazzy-libg2o installed system-wide on this machine (no root access when
# this was set up), so it was extracted from the .deb into this user-local
# prefix instead - controller_server needs this on LD_LIBRARY_PATH to dlopen
# libg2o_csparse_extension.so at plugin-load time (the other g2o libs resolve
# via the rpath baked in at link time, this one doesn't, for reasons not
# worth chasing further). Harmless no-op for the other three controllers. If
# ros-jazzy-libg2o is ever installed properly via apt, this becomes a no-op
# and can be removed.
_G2O_LOCAL_LIB = "/home/dvt/.local/ros-extra-deps/opt/ros/jazzy/lib/x86_64-linux-gnu"


def generate_launch_description():

    pkg_share = get_package_share_directory("main_bot")

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(pkg_share, "maps", "warehouse.yaml"),
        description="Full path to the map yaml file saved from slam_toolbox",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        # config/nav2_{mppi,teb,dwb,rpp}.yaml all exist - same server set,
        # different controller_server.FollowPath plugin. GUI's "Local
        # planner" buttons pick one explicitly; this default covers running
        # this launch file directly with no arguments.
        default_value=os.path.join(pkg_share, "config", "nav2_mppi.yaml"),
        description="Full path to the Nav2 parameters file (config/nav2_{mppi,teb,dwb,rpp}.yaml)",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use the Gazebo /clock as the ROS time source",
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            )
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "slam": "False",
            # Composed (single-process) bringup crashes with a SIGSEGV inside
            # ImageMagick while loading the map image; run isolated processes instead.
            "use_composition": "False",
        }.items(),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "LD_LIBRARY_PATH",
                _G2O_LOCAL_LIB + ":" + os.environ.get("LD_LIBRARY_PATH", ""),
            ),
            map_arg,
            params_file_arg,
            use_sim_time_arg,
            nav2_bringup,
        ]
    )
