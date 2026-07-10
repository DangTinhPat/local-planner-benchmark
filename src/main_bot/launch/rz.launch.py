import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, UnsetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("main_bot")

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(pkg_share, "rviz", "nav2_view.rviz"),
        description="Full path to the RViz config file to use",
    )

    return LaunchDescription([
        rviz_config_arg,
        # VS Code's snap packaging injects GTK_PATH into every integrated
        # terminal it spawns. Qt's gtk3 platform theme then resolves the
        # canberra-gtk-module from inside that snap, which drags in the
        # snap's bundled (older) libpthread and crashes rviz2 on startup
        # with "symbol lookup error: ... undefined symbol: __libc_pthread_init".
        UnsetEnvironmentVariable('GTK_PATH'),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])