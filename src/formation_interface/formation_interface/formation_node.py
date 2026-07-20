"""Formation node - runs on/near the drones.

Acts as a ``GoToFormation`` action server:

  1. On a goal, compute per-drone target positions from the requested formation.
  2. Assign targets to drones (nearest-first) to keep travel short.
  3. Stream setpoints to each drone through its commander backend.
  4. Watch the OptiTrack poses and publish feedback until every drone is within
     ``position_tolerance`` (held for ``settle_cycles``), or timeout / cancel.

A free-running control timer publishes the offboard heartbeats; PX4 needs those
streaming continuously, independent of the action lifecycle.
"""

import math

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from formation_interfaces.action import GoToFormation

from formation_interface.drone_commander import make_commander
from formation_interface.formations import (
    CUSTOM,
    FORMATIONS,
    compute_targets,
    transform_offsets,
)


def _dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


class FormationNode(Node):
    def __init__(self):
        super().__init__("formation_node")

        # ------------------------------ parameters ------------------------------
        self.declare_parameter(
            "drone_namespaces", ["px4_1", "px4_2", "px4_3", "px4_4", "px4_5"])
        self.declare_parameter("system_ids", [2, 3, 4, 5, 6])
        self.declare_parameter("pose_topics", [
            "/vrpn_mocap/drone1/pose", "/vrpn_mocap/drone2/pose",
            "/vrpn_mocap/drone3/pose", "/vrpn_mocap/drone4/pose",
            "/vrpn_mocap/drone5/pose"])
        self.declare_parameter("pose_topic_type", "PoseStamped")   # or "Odometry"
        self.declare_parameter("backend", "px4")                   # or "sim"
        self.declare_parameter("control_rate", 20.0)               # Hz
        self.declare_parameter("position_tolerance", 0.15)         # m
        self.declare_parameter("settle_cycles", 10)
        self.declare_parameter("auto_arm", False)
        self.declare_parameter("goal_timeout", 60.0)               # s

        g = self.get_parameter
        self.namespaces = list(g("drone_namespaces").value)
        self.system_ids = list(g("system_ids").value)
        self.pose_topics = list(g("pose_topics").value)
        self.pose_type = g("pose_topic_type").value
        self.backend = g("backend").value
        self.control_rate = float(g("control_rate").value)
        self.tolerance = float(g("position_tolerance").value)
        self.settle_cycles = int(g("settle_cycles").value)
        self.auto_arm = bool(g("auto_arm").value)
        self.goal_timeout = float(g("goal_timeout").value)

        self.n = len(self.namespaces)
        self._cb = ReentrantCallbackGroup()

        # ------------------------------ per-drone state -------------------------
        self.poses = [None] * self.n       # latest (x, y, z) ENU per drone
        self.targets = [None] * self.n     # active (x, y, z) ENU target or None
        self._armed = [False] * self.n
        self._active = False               # a goal is currently executing

        self.commanders = []
        for i, ns in enumerate(self.namespaces):
            sysid = self.system_ids[i] if i < len(self.system_ids) else i + 2
            topic = self.pose_topics[i] if i < len(self.pose_topics) else f"/{ns}/pose"
            self.commanders.append(
                make_commander(self.backend, self, ns, sysid, topic))
            self._make_pose_sub(i, topic)

        # ------------------------------ ros interfaces --------------------------
        self.create_timer(
            1.0 / self.control_rate, self._control_loop, callback_group=self._cb)

        self._server = ActionServer(
            self, GoToFormation, "go_to_formation",
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb)

        self.get_logger().info(
            f"formation_node ready: {self.n} drones, backend='{self.backend}', "
            f"auto_arm={self.auto_arm}, tol={self.tolerance} m")

    # ------------------------------ pose intake --------------------------------
    def _make_pose_sub(self, idx, topic):
        if self.pose_type == "Odometry":
            self.create_subscription(
                Odometry, topic,
                lambda m, i=idx: self._store_pose(i, m.pose.pose.position),
                10, callback_group=self._cb)
        else:
            self.create_subscription(
                PoseStamped, topic,
                lambda m, i=idx: self._store_pose(i, m.pose.position),
                10, callback_group=self._cb)

    def _store_pose(self, idx, position):
        self.poses[idx] = (position.x, position.y, position.z)

    # ------------------------------ control loop -------------------------------
    def _control_loop(self):
        """Stream setpoints; handle the (optional) auto-arm sequence per drone."""
        for i, cmd in enumerate(self.commanders):
            if self.targets[i] is None:
                continue
            cmd.publish_heartbeat()
            if self.auto_arm and not self._armed[i] and cmd.ready_to_arm():
                cmd.engage_offboard()
                cmd.arm()
                self._armed[i] = True
                self.get_logger().info(f"[{self.namespaces[i]}] offboard + armed")

    def _errors(self):
        errs = []
        for i in range(self.n):
            if self.poses[i] is None or self.targets[i] is None:
                errs.append(float("inf"))
            else:
                errs.append(_dist(self.poses[i], self.targets[i]))
        return errs

    def _assign(self, targets):
        """Assign each drone the nearest still-free target (greedy, no crossings).

        Falls back to index order if we don't yet have a pose for every drone.
        """
        if any(p is None for p in self.poses):
            return list(targets[: self.n])
        result = [None] * self.n
        used = set()
        for i in range(self.n):
            best, best_d = None, None
            for t in range(len(targets)):
                if t in used:
                    continue
                d = _dist(self.poses[i], targets[t])
                if best_d is None or d < best_d:
                    best, best_d = t, d
            used.add(best)
            result[i] = targets[best]
        return result

    # ------------------------------ action callbacks ---------------------------
    def _goal_cb(self, goal_request):
        if self._active:
            self.get_logger().warn("rejecting goal: a formation is already running")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        self._active = True
        try:
            return self._run_goal(goal_handle)
        finally:
            self._active = False

    def _run_goal(self, goal_handle):
        req = goal_handle.request
        result = GoToFormation.Result()

        formation = req.formation.lower().strip()
        spacing = req.spacing if req.spacing > 0 else 1.5
        altitude = req.altitude if req.altitude > 0 else 1.5
        center = (req.center.x, req.center.y)

        if formation == CUSTOM:
            if len(req.custom_offsets) != self.n:
                goal_handle.abort()
                result.success = False
                result.message = (
                    f"custom formation needs exactly {self.n} offsets, "
                    f"got {len(req.custom_offsets)}")
                return result
            raw = transform_offsets(
                [(p.x, p.y, p.z) for p in req.custom_offsets],
                center, altitude, req.yaw)
        elif formation in FORMATIONS:
            raw = compute_targets(
                formation, self.n, spacing, center, altitude, req.yaw)
        else:
            goal_handle.abort()
            result.success = False
            result.message = (
                f"unknown formation '{req.formation}'; "
                f"options: {list(FORMATIONS)} or '{CUSTOM}' with offsets")
            return result
        targets = self._assign(raw)
        for i in range(self.n):
            self.targets[i] = targets[i]
            self.commanders[i].set_target_enu(*targets[i], yaw=req.yaw)

        self.get_logger().info(
            f"forming '{formation}': spacing={spacing} m, alt={altitude} m")

        rate = self.create_rate(5.0)          # feedback / check rate
        settle = 0
        start = self.get_clock().now()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "canceled by operator"
                return result

            errs = self._errors()
            finite = [e for e in errs if e != float("inf")]
            in_pos = sum(1 for e in errs if e <= self.tolerance)
            max_err = max(finite) if finite else float("inf")

            fb = GoToFormation.Feedback()
            fb.drones_total = self.n
            fb.drones_in_position = in_pos
            fb.progress = in_pos / self.n if self.n else 0.0
            fb.max_error = float(max_err) if max_err != float("inf") else -1.0
            goal_handle.publish_feedback(fb)

            if in_pos == self.n:
                settle += 1
                if settle >= self.settle_cycles:
                    goal_handle.succeed()
                    result.success = True
                    result.message = (
                        f"'{formation}' achieved (worst error {max_err:.2f} m)")
                    return result
            else:
                settle = 0

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > self.goal_timeout:
                goal_handle.abort()
                result.success = False
                result.message = (
                    f"timeout after {self.goal_timeout:.0f} s "
                    f"({in_pos}/{self.n} in position)")
                return result

            rate.sleep()

        goal_handle.abort()
        result.success = False
        result.message = "node shutting down"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FormationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
