"""Warehouse waypoint table for the putaway job loop.

Coordinates in worlds/warehouse.sdf are in the Gazebo WORLD frame; Nav2 goals
must be in the MAP frame. The map is anchored at robot1's charging pad
(world 3.75,-4.175, yaw pi/2) then rotated -90deg (see
launch/multi_robot_nav2.launch.py and config/nav2_robot{1,2}.yaml's baked-in
initial_pose comments). Verified against both robots' known initial_pose:
robot1 world (3.75,-4.175,pi/2) -> map (0,0,0); robot2 world
(4.75,-4.175,pi/2) -> map (0,-1,0).
"""
import math
from dataclasses import dataclass


def _normalize(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def world_to_map(world_x, world_y, world_yaw=0.0):
    map_x = world_y + 4.175
    map_y = 3.75 - world_x
    map_yaw = _normalize(world_yaw - math.pi / 2)
    return map_x, map_y, map_yaw


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float


# Receiving docks: dock_mark_* in warehouse.sdf are visual-only (no collision
# box), so their raw pose is a safe goal as-is.
_RAW_DOCKS = [
    ("dock_mark_1", 3.75, 3.5),
    ("dock_mark_2", 6.5625, 3.5),
    ("dock_mark_3", 9.375, 3.5),
]
_DOCK_APPROACH_WORLD_YAW = math.pi / 2  # arrive facing north, into the dock
DOCKS = [Waypoint(name, *world_to_map(x, y, _DOCK_APPROACH_WORLD_YAW)) for name, x, y in _RAW_DOCKS]

# Shelf racks (rack_rA..rack_rF, north/south) DO have a 0.375(x) x 2.25(y) x
# 1.1(z) collision box centered exactly at their listed pose - a goal placed
# there is inside a lethal obstacle. An earlier version offset the goal +0.7m
# into the same-row aisle between adjacent rack columns, but measuring actual
# clearance-to-nearest-obstacle on the saved map (maps/warehouse.pgm) showed
# that aisle peaks at only ~0.60m clearance at its widest point - under the
# local costmap's inflation_radius (0.70, config/nav2_robot{1,2}.yaml), which
# in testing made the robot perpetually trip collision_monitor's
# FootprintApproach warning while approaching head-on and never settle inside
# the goal tolerance (repeated "Failed to make progress" -> failed spin
# recovery -> goal abort). The open east-west corridor at y=0 between the
# north/south rack rows (the same corridor the docks sit in) measured >=0.95m
# clearance at every column - use that instead: one waypoint per rack column,
# aligned in x with that column, yaw facing whichever row (north/south) it
# represents. Symbolic "delivered to rack rX_n/s" rather than nose-up-to-the-
# shelf precision, which is fine for a simulated putaway loop.
_RACK_ROWS = [
    ("rA", -11.0625),
    ("rB", -9.375),
    ("rC", -7.6875),
    ("rD", -6.0),
    ("rE", -4.3125),
    ("rF", -2.625),
]
_NORTH_WORLD_YAW = math.pi / 2
_SOUTH_WORLD_YAW = -math.pi / 2
_RAW_SHELVES = [(f"rack_{row}_n", x, 0.0, _NORTH_WORLD_YAW) for row, x in _RACK_ROWS] + [
    (f"rack_{row}_s", x, 0.0, _SOUTH_WORLD_YAW) for row, x in _RACK_ROWS
]
SHELVES = [Waypoint(name, *world_to_map(x, y, yaw)) for name, x, y, yaw in _RAW_SHELVES]

# Each robot's home is its own Nav2 map-frame origin, baked into
# config/nav2_robot{1,2}.yaml's initial_pose - returning here reproduces the
# exact spawn/charging pose.
HOME = {
    "robot1": Waypoint("chrg_pad", 0.0, 0.0, 0.0),
    "robot2": Waypoint("chrg2_pad", 0.0, -1.0, 0.0),
}


@dataclass
class Job:
    id: int
    dock: Waypoint
    shelf: Waypoint


def make_pose_stamped(waypoint, clock):
    from geometry_msgs.msg import PoseStamped

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = clock.now().to_msg()
    pose.pose.position.x = waypoint.x
    pose.pose.position.y = waypoint.y
    pose.pose.orientation.z = math.sin(waypoint.yaw / 2)
    pose.pose.orientation.w = math.cos(waypoint.yaw / 2)
    return pose
