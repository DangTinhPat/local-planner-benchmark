import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    UnsetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

import xacro


def robot_actions(pkg_share, robot_name, x, y, yaw, z=0.05):
    """Everything needed to bring up one namespaced robot in an already-running world.

    Uses the plain gz-sim DiffDrive plugin (not ros2_control/gz_ros2_control):
    running multiple controller_manager instances in one Gazebo process proved
    unreliable (controller_manager sometimes failing to initialize, and once
    crashing the whole sim), so multi-robot sims skip ros2_control entirely.
    """
    xacro_file = os.path.join(pkg_share, "description", "robot.urdf.xacro")
    robot_description = xacro.process_file(
        xacro_file, mappings={"robot_name": robot_name, "use_ros2_control": "false"}
    ).toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=robot_name,
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        # tf2_ros hardcodes "/tf" and "/tf_static" as absolute topic names, so
        # namespace= alone does NOT namespace them - without this remap both
        # robots' static transforms (base_link->lidar_frame, ->wheels, etc.)
        # land on one shared global /tf_static and clobber each other, since
        # frame_ids themselves are deliberately left unprefixed (see lidar.xacro).
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", f"{robot_name}/robot_description",
            "-name", robot_name,
            "-x", str(x), "-y", str(y), "-z", str(z),
            "-Y", str(yaw),
        ],
        output="screen",
    )

    # Every gz-transport topic here is namespaced ("<robot_name>/...", set via
    # gz_diff_drive.xacro's frame_prefix) so multiple robots don't collide; no
    # node namespace= on the bridge itself, or the ROS side would double up.
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            f"{robot_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            f"{robot_name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            f"{robot_name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            f"{robot_name}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            f"{robot_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        output="screen",
    )

    # Handed back so callers can chain the *next* robot's spawn off this one's
    # completion, instead of guessing a fixed delay (a fixed delay tuned against
    # headless testing fired robot1's spawn before a real GUI's slower startup
    # had the world ready, silently dropping it while robot2's later delay
    # happened to land after the world was ready).
    return [robot_state_publisher_node, spawn_entity_node, bridge_node], spawn_entity_node


def generate_launch_description():

    pkg_share = get_package_share_directory("main_bot")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_share, "worlds", "warehouse.sdf"),
        description="Gazebo world to load (name or full path to an .sdf file)",
    )

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description=(
            "Run Gazebo without its 3D GUI (physics/sensors still run, just no "
            "render window) - the GUI alone costs 2+ CPU cores, so this is the "
            "knob to reach for on hardware-constrained machines. Watch the sim "
            "through RViz instead."
        ),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": [
                LaunchConfiguration("world"),
                PythonExpression(
                    ["' -r -s' if '", LaunchConfiguration("headless"), "' == 'true' else ' -r'"]
                ),
            ]
        }.items(),
    )

    # /clock is world-wide and must be bridged exactly once, shared by every robot.
    clock_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # robot1: charging bay 1, robot2: charging bay 2 (10cm east of bay 1's wall),
    # both facing north out of their bay - see worlds/warehouse.sdf chrg2_*.
    robot1_actions, robot1_spawn_marker = robot_actions(
        pkg_share, "robot1", x=3.75, y=-4.175, yaw=1.5707963267948966
    )
    robot2_actions, _ = robot_actions(
        pkg_share, "robot2", x=4.75, y=-4.175, yaw=1.5707963267948966
    )

    # Spawning two entities back-to-back has crashed gz-sim's renderer in testing
    # (a null pointer deep in its scene sync); wait for robot1's spawn attempt to
    # actually finish (however long the world takes to become ready) before
    # starting robot2's, rather than guessing a fixed delay.
    delayed_robot2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot1_spawn_marker,
            on_exit=robot2_actions,
        )
    )

    return LaunchDescription(
        [
            world_arg,
            headless_arg,
            # VS Code's snap packaging injects GTK_PATH into every integrated
            # terminal it spawns. gz sim's GUI then resolves the canberra-gtk-module
            # from inside that snap, which drags in the snap's bundled (older)
            # libpthread and crashes with "symbol lookup error: ... undefined
            # symbol: __libc_pthread_init" right after entities spawn, tearing
            # down the whole sim before Nav2 has anything to act on.
            UnsetEnvironmentVariable('GTK_PATH'),
            gz_sim,
            clock_bridge_node,
            *robot1_actions,
            delayed_robot2,
        ]
    )
