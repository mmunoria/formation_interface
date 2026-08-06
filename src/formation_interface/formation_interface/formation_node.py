import math

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from formation_interfaces.action import GoToFormation

from formation_interface.drone_commander import make_commander
from formation_interface.formations import (
    CUSTOM,
    FORMATIONS,
    compute_targets,
    transform_offsets,
)
from formation_interface.planning import Arena, PlanningError, plan_paths

MAX_DRONES = 8

# Late subscribers (GUI, rviz) should still receive the last published path.
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


class FormationNode(Node):
    def __init__(self):
        super().__init__("formation_node")

       
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
        # Rectangular indoor arena (map frame, metres) + planner settings.
        self.declare_parameter("arena_min_x", -4.0)
        self.declare_parameter("arena_max_x", 5.0)
        self.declare_parameter("arena_min_y", -2.0)
        self.declare_parameter("arena_max_y", 2.0)
        self.declare_parameter("plan_cell_size", 0.1)              # m
        self.declare_parameter("safety_radius", 0.4)               # m
        self.declare_parameter("waypoint_tolerance", 0.25)         # m

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
        self.arena = Arena(
            float(g("arena_min_x").value), float(g("arena_max_x").value),
            float(g("arena_min_y").value), float(g("arena_max_y").value))
        self.plan_cell = float(g("plan_cell_size").value)
        self.safety_radius = float(g("safety_radius").value)
        self.waypoint_tol = float(g("waypoint_tolerance").value)

        if len(self.namespaces) > MAX_DRONES:
            self.get_logger().warn(
                f"{len(self.namespaces)} drones configured; "
                f"max is {MAX_DRONES} - ignoring the extras")
            self.namespaces = self.namespaces[:MAX_DRONES]
        self.n = len(self.namespaces)
        self._cb = ReentrantCallbackGroup()

        
        self.poses = [None] * self.n       # latest (x, y, z) ENU per drone
        self.targets = [None] * self.n     # active (x, y, z) ENU target or None
        self._armed = [False] * self.n
        self._active = False               # a goal is currently executing
        self._paths = [None] * self.n      # planned [(x, y), ...] per drone
        self._wp = [0] * self.n            # index of the waypoint being flown to
        self._current_wp = [None] * self.n  # (x, y, z) ENU waypoint being flown *now*

        self.commanders = []
        self._path_pubs = []
        self._current_target_pubs = []
        for i, ns in enumerate(self.namespaces):
            sysid = self.system_ids[i] if i < len(self.system_ids) else i + 2
            topic = self.pose_topics[i] if i < len(self.pose_topics) else f"/{ns}/pose"
            self.commanders.append(
                make_commander(self.backend, self, ns, sysid, topic, i + 1))
            self._make_pose_sub(i, topic)
            # Full planned path, published once (latched) per goal - for
            # visualization (gui_node) only, not a live setpoint feed.
            self._path_pubs.append(
                self.create_publisher(Path, f"/drone{i + 1}/path", LATCHED_QOS))
            # The single waypoint currently being flown to, republished every
            # control tick - for onboard/companion-computer consumers (e.g.
            # from_drone/mocap_update.py) that need a live target, not the
            # whole plan.
            self._current_target_pubs.append(
                self.create_publisher(
                    PoseStamped, f"/drone{i + 1}/current_target", 10))

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
        
        for i, cmd in enumerate(self.commanders):
            cmd.publish_heartbeat()
            self._publish_current_target(i)
            if self.targets[i] is None:
                continue
            if self.auto_arm and not self._armed[i] and cmd.ready_to_arm():
                cmd.engage_offboard()
                cmd.arm()
                self._armed[i] = True
                self.get_logger().info(f"[{self.namespaces[i]}] offboard + armed")

    """ some new stuff don not touch until test usabata"""
    def _publish_current_target(self, i):
        
        wp = self._current_wp[i]
        if wp is None:
            return
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(wp[0])
        msg.pose.position.y = float(wp[1])
        msg.pose.position.z = float(wp[2])
        msg.pose.orientation.w = 1.0
        self._current_target_pubs[i].publish(msg)

    def _errors(self, indices=None):
        if indices is None:
            indices = range(self.n)
        errs = []
        for i in indices:
            if self.poses[i] is None or self.targets[i] is None:
                errs.append(float("inf"))
            else:
                errs.append(_dist(self.poses[i], self.targets[i]))
        return errs

    def _target_indices(self, req):
        
        if not req.drone_ids:
            return list(range(self.n))
        ids = list(req.drone_ids)
        if len(set(ids)) != len(ids):
            raise ValueError(f"drone_ids has duplicates: {ids}")
        bad = [d for d in ids if d < 1 or d > self.n]
        if bad:
            raise ValueError(f"drone_ids out of range (1..{self.n}): {bad}")
        return [d - 1 for d in ids]

    def _assign(self, targets, indices):
        
        if any(self.poses[i] is None for i in indices):
            return list(targets[: len(indices)])
        result = [None] * len(indices)
        used = set()
        for k, i in enumerate(indices):
            best, best_d = None, None
            for t in range(len(targets)):
                if t in used:
                    continue
                d = _dist(self.poses[i], targets[t])
                if best_d is None or d < best_d:
                    best, best_d = t, d
            used.add(best)
            result[k] = targets[best]
        return result

    # ------------------------------ path following -----------------------------
    def _advance_waypoints(self, altitude, yaw):
    
        for i in range(self.n):
            path = self._paths[i]
            if path is None or self._wp[i] >= len(path) - 1:
                continue
            pose = self.poses[i]
            if pose is None:
                continue
            wx, wy = path[self._wp[i]]
            if math.hypot(pose[0] - wx, pose[1] - wy) <= self.waypoint_tol:
                self._wp[i] += 1
                nx, ny = path[self._wp[i]]
                self.commanders[i].set_target_enu(nx, ny, altitude, yaw=yaw)
                self._current_wp[i] = (nx, ny, altitude)

    def _publish_path(self, i, path, altitude):
        
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y) in path:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = float(altitude)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._path_pubs[i].publish(msg)

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

        try:
            idx = self._target_indices(req)
        except ValueError as exc:
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            return result
        count = len(idx)

        if formation == CUSTOM:
            if len(req.custom_offsets) != count:
                goal_handle.abort()
                result.success = False
                result.message = (
                    f"custom formation needs exactly {count} offsets, "
                    f"got {len(req.custom_offsets)}")
                return result
            raw = transform_offsets(
                [(p.x, p.y, p.z) for p in req.custom_offsets],
                center, altitude, req.yaw)
        elif formation in FORMATIONS:
            raw = compute_targets(
                formation, count, spacing, center, altitude, req.yaw)
        else:
            goal_handle.abort()
            result.success = False
            result.message = (
                f"unknown formation '{req.formation}'; "
                f"options: {list(FORMATIONS)} or '{CUSTOM}' with offsets")
            return result
        targets = self._assign(raw, idx)

        # ---- arena bounds check (R3) ----
        outside = [idx[k] + 1 for k, t in enumerate(targets)
                   if not self.arena.contains(t[0], t[1])]
        if outside:
            goal_handle.abort()
            result.success = False
            result.message = (
                f"targets outside the arena for drone(s) {outside} - "
                f"shrink spacing or move the formation centre")
            return result

        # ---- path planning, drones as mutual obstacles (R4) ----
        missing = [i + 1 for i in idx if self.poses[i] is None]
        if missing:
            goal_handle.abort()
            result.success = False
            result.message = (
                f"no pose received for drone(s) {missing} - "
                f"check the OptiTrack topics")
            return result

        # Drones not in this goal but with a known pose (e.g. holding a
        # target from an earlier goal) are static obstacles, not planned for.
        static_obstacles = [
            (self.poses[i][0], self.poses[i][1])
            for i in range(self.n) if i not in idx and self.poses[i] is not None]

        try:
            paths, crossings = plan_paths(
                [(self.poses[i][0], self.poses[i][1]) for i in idx],
                [(t[0], t[1]) for t in targets],
                self.arena, self.plan_cell, self.safety_radius,
                static_obstacles=static_obstacles)
        except PlanningError as exc:
            goal_handle.abort()
            result.success = False
            result.message = f"path planning failed: {exc}"
            return result
        if crossings:
            self.get_logger().warn(
                f"drone(s) {[idx[c - 1] + 1 for c in crossings]} cross "
                f"other planned paths (unavoidable for this formation)")

        for k, i in enumerate(idx):
            self._paths[i] = paths[k]
            self._wp[i] = 1 if len(paths[k]) > 1 else 0
            self._publish_path(i, paths[k], altitude)
            self.targets[i] = targets[k]
            wx, wy = paths[k][self._wp[i]]
            self.commanders[i].set_target_enu(wx, wy, altitude, yaw=req.yaw)
            self._current_wp[i] = (wx, wy, altitude)

        self.get_logger().info(
            f"forming '{formation}' for drone(s) {[i + 1 for i in idx]}: "
            f"spacing={spacing} m, alt={altitude} m, "
            f"paths planned ({sum(len(p) for p in paths)} waypoints total)")

        rate = self.create_rate(5.0)          # feedback / check rate
        settle = 0
        start = self.get_clock().now()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "canceled by operator"
                return result

            self._advance_waypoints(altitude, req.yaw)

            errs = self._errors(idx)
            finite = [e for e in errs if e != float("inf")]
            in_pos = sum(1 for e in errs if e <= self.tolerance)
            max_err = max(finite) if finite else float("inf")

            fb = GoToFormation.Feedback()
            fb.drones_total = count
            fb.drones_in_position = in_pos
            fb.progress = in_pos / count if count else 0.0
            fb.max_error = float(max_err) if max_err != float("inf") else -1.0
            goal_handle.publish_feedback(fb)

            if in_pos == count:
                settle += 1
                if settle >= self.settle_cycles:
                    goal_handle.succeed()
                    result.success = True
                    result.message = (
                        f"'{formation}' achieved for drone(s) "
                        f"{[i + 1 for i in idx]} (worst error {max_err:.2f} m)")
                    return result
            else:
                settle = 0

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > self.goal_timeout:
                goal_handle.abort()
                result.success = False
                result.message = (
                    f"timeout after {self.goal_timeout:.0f} s "
                    f"({in_pos}/{count} in position)")
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
