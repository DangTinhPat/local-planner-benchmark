"""Central fleet dispatcher for the 2-robot warehouse putaway demo.

Assigns (dock, shelf) jobs from a shared pending pool to whichever of
robot1/robot2 goes idle, nearest-dock-first, then drives that robot
dock -> shelf -> its own charging pad, forever. Deliberately a single
process with two BasicNavigator instances polled round-robin - no threads,
no locks; this matches Nav2's own documented pattern for simple multi-robot
control and is more than adequate for 2 robots.
"""
import math
import random
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from fleet_dispatcher.waypoints import DOCKS, HOME, SHELVES, Job, make_pose_stamped

DWELL_SECONDS = 2.0
MAX_LEG_RETRIES = 3
# bt_navigator/controller_server need a moment to fully reset their behavior
# tree after aborting a goal; resubmitting instantly (no delay at all) made
# every subsequent goal abort within milliseconds too, in testing - a
# runaway failure loop that never recovered. This backoff is what actually
# lets a retry succeed instead of instant-failing forever.
RETRY_BACKOFF_SECONDS = 2.0
TICK_SECONDS = 0.05

IDLE = "IDLE"
EN_ROUTE_TO_DOCK = "EN_ROUTE_TO_DOCK"
AT_DOCK = "AT_DOCK"
EN_ROUTE_TO_SHELF = "EN_ROUTE_TO_SHELF"
AT_SHELF = "AT_SHELF"
EN_ROUTE_TO_CHARGE = "EN_ROUTE_TO_CHARGE"
WAITING_RETRY = "WAITING_RETRY"


class RobotAgent:
    """One robot's navigation client + putaway-loop state machine."""

    def __init__(self, robot_name):
        self.name = robot_name
        self.home = HOME[robot_name]
        self.navigator = BasicNavigator(node_name=f"fleet_dispatcher_{robot_name}", namespace=robot_name)
        self.navigator.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.last_pose = None
        self.navigator.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self._on_pose, pose_qos)

        self.state = IDLE
        self.job = None
        self.dwell_until = None
        self.retries = 0
        self.jobs_completed = 0
        # Gates both same-job retries and fresh job assignment after a
        # give-up - either way we just submitted/aborted a goal and need to
        # let bt_navigator settle before the next one.
        self.next_action_at = 0.0
        self._pending_leg = None

    def _on_pose(self, msg):
        self.last_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def log(self, msg):
        self.navigator.get_logger().info(msg)

    def current_xy(self):
        return self.last_pose if self.last_pose is not None else (self.home.x, self.home.y)

    def start_job(self, job):
        self.job = job
        self.retries = 0
        self._go_to(job.dock, EN_ROUTE_TO_DOCK, "DOCK", job.dock.name)

    def _go_to(self, waypoint, next_state, leg_label, waypoint_name):
        pose = make_pose_stamped(waypoint, self.navigator.get_clock())
        self.log(
            f"job #{self.job.id}: -> navigating to {leg_label} {waypoint_name} "
            f"(map {waypoint.x:.2f}, {waypoint.y:.2f})"
        )
        self.navigator.goToPose(pose)
        self.state = next_state

    def _arm_retry(self, waypoint, next_state, leg_label, waypoint_name):
        """Defer a goToPose call until RETRY_BACKOFF_SECONDS from now."""
        self._pending_leg = (waypoint, next_state, leg_label, waypoint_name)
        self.next_action_at = time.monotonic() + RETRY_BACKOFF_SECONDS
        self.state = WAITING_RETRY

    def tick(self):
        if self.state == IDLE:
            return

        if self.state == WAITING_RETRY:
            if time.monotonic() >= self.next_action_at:
                self._go_to(*self._pending_leg)
                self._pending_leg = None
            return

        if self.state in (AT_DOCK, AT_SHELF):
            if time.monotonic() >= self.dwell_until:
                if self.state == AT_DOCK:
                    self._go_to(self.job.shelf, EN_ROUTE_TO_SHELF, "SHELF", self.job.shelf.name)
                else:
                    self._go_to(self.home, EN_ROUTE_TO_CHARGE, "HOME", self.home.name)
            return

        if not self.navigator.isTaskComplete():
            return

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.retries = 0
            if self.state == EN_ROUTE_TO_DOCK:
                self.log(f"job #{self.job.id}: arrived at DOCK {self.job.dock.name}, dwelling {DWELL_SECONDS}s")
                self.state = AT_DOCK
                self.dwell_until = time.monotonic() + DWELL_SECONDS
            elif self.state == EN_ROUTE_TO_SHELF:
                self.log(f"job #{self.job.id}: arrived at SHELF {self.job.shelf.name}, dwelling {DWELL_SECONDS}s")
                self.state = AT_SHELF
                self.dwell_until = time.monotonic() + DWELL_SECONDS
            elif self.state == EN_ROUTE_TO_CHARGE:
                self.log(f"job #{self.job.id}: docked at HOME {self.home.name}, job complete (total done: {self.jobs_completed + 1})")
                self.jobs_completed += 1
                self.job = None
                self.state = IDLE
                self.next_action_at = time.monotonic() + RETRY_BACKOFF_SECONDS
            return

        # FAILED/CANCELED/UNKNOWN: retry the same leg a few times (with a
        # cooldown - see _arm_retry), then give up on the remaining legs and
        # just try to get home.
        self.retries += 1
        self.log(f"job #{self.job.id}: NAV FAILED in state {self.state} (result={result}), retry {self.retries}/{MAX_LEG_RETRIES}")
        if self.retries >= MAX_LEG_RETRIES:
            if self.state == EN_ROUTE_TO_CHARGE:
                self.log(f"job #{self.job.id}: giving up, staying idle where we are")
                self.job = None
                self.state = IDLE
                self.next_action_at = time.monotonic() + RETRY_BACKOFF_SECONDS
            else:
                self.log(f"job #{self.job.id}: giving up on remaining legs, heading HOME")
                self.retries = 0  # fresh retry budget for the HOME leg, not the failed leg's leftovers
                self._arm_retry(self.home, EN_ROUTE_TO_CHARGE, "HOME", self.home.name)
            return

        if self.state == EN_ROUTE_TO_DOCK:
            self._arm_retry(self.job.dock, EN_ROUTE_TO_DOCK, "DOCK", self.job.dock.name)
        elif self.state == EN_ROUTE_TO_SHELF:
            self._arm_retry(self.job.shelf, EN_ROUTE_TO_SHELF, "SHELF", self.job.shelf.name)
        elif self.state == EN_ROUTE_TO_CHARGE:
            self._arm_retry(self.home, EN_ROUTE_TO_CHARGE, "HOME", self.home.name)


class Dispatcher:
    def __init__(self, agents):
        self.agents = agents
        self.pending_jobs = []
        self._next_job_id = 1

    def _refill_jobs(self):
        jobs = [Job(0, dock, shelf) for dock in DOCKS for shelf in SHELVES]
        random.shuffle(jobs)
        for job in jobs:
            job.id = self._next_job_id
            self._next_job_id += 1
        self.pending_jobs.extend(jobs)

    def _busy_dock_names(self, exclude_agent):
        """Docks another agent is currently at or actively en route to.

        Both robots' charging pads sit only ~1m apart, and the nearest-dock
        heuristic tends to pick the same dock for whichever robot just went
        idle near home - without this check they'd repeatedly get assigned
        the same dock and jam each other at its narrow approach (observed in
        testing: repeated "Failed to make progress" until both gave up).
        """
        busy = set()
        for agent in self.agents:
            if agent is exclude_agent or agent.job is None:
                continue
            if agent.state in (EN_ROUTE_TO_DOCK, AT_DOCK):
                busy.add(agent.job.dock.name)
            elif agent.state == WAITING_RETRY and agent._pending_leg is not None and agent._pending_leg[1] == EN_ROUTE_TO_DOCK:
                busy.add(agent.job.dock.name)
        return busy

    def _assign(self, agent):
        if not self.pending_jobs:
            self._refill_jobs()
        busy_docks = self._busy_dock_names(agent)
        candidates = [j for j in self.pending_jobs if j.dock.name not in busy_docks] or self.pending_jobs
        px, py = agent.current_xy()
        job = min(candidates, key=lambda j: math.hypot(j.dock.x - px, j.dock.y - py))
        self.pending_jobs.remove(job)
        agent.log(f"job #{job.id} assigned: dock={job.dock.name} shelf={job.shelf.name} (pending={len(self.pending_jobs)})")
        agent.start_job(job)

    def tick(self):
        for agent in self.agents:
            if agent.state == IDLE and time.monotonic() >= agent.next_action_at:
                self._assign(agent)
            agent.tick()


def main():
    rclpy.init()

    agents = [RobotAgent("robot1"), RobotAgent("robot2")]
    for agent in agents:
        agent.log(f"{agent.name}: waiting for Nav2 to activate...")
        agent.navigator.waitUntilNav2Active(localizer="amcl")
        agent.log(f"{agent.name}: Nav2 is ready for use!")

    dispatcher = Dispatcher(agents)

    try:
        while rclpy.ok():
            for agent in agents:
                rclpy.spin_once(agent.navigator, timeout_sec=TICK_SECONDS)
            dispatcher.tick()
    except KeyboardInterrupt:
        pass
    finally:
        for agent in agents:
            agent.navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
