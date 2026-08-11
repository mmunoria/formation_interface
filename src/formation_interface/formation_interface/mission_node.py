"""ExecuteFlight action server + TerminateFlight service: the mission
manager.

Starts/interrupts/terminates ONE drone's flight independently of every other
drone (multiple concurrent ``ExecuteFlight`` goals, one per ``drone_name``),
unlike ``formation_node``'s single-goal-for-the-whole-swarm model - every
drone here is independently idle/streaming/holding. A separate, independent
layer on top of the formation-flying stack (same relationship
``monitor_gui.py``/``active_tracker.py`` have to ``formation_node``): it
reuses ``drone_commander.py``'s PX4/Mock commanders (via the ``local``
deploy backend) but does not run alongside or coordinate with
``formation_node`` itself.

Cancelling a goal is the gentle Interrupt: it freezes the drone's last
commanded setpoint (``deployer.interrupt(hard=False)``) and does not land or
retrieve logs. Landing + log retrieval is the separate, deliberate
``TerminateFlight`` service call.
"""

import math
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from formation_interfaces.action import ExecuteFlight
from formation_interfaces.msg import DroneTelemetry
from formation_interfaces.srv import TerminateFlight

from formation_interface.deploy.backends import make_deployer
from formation_interface.drone_profiles import drone_supports, list_drone_profiles
from formation_interface.flight_profiles import list_flight_profiles, sample_trajectory
from formation_interface.mission_store import load_mission, new_run_dir
from formation_interface.telemetry import PX4TelemetryWatcher, TelemetrySample


def _dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_node")

        self.declare_parameter("missions_root", "missions")
        self.declare_parameter("drone_profiles_dir", "config/drone_profiles")
        self.declare_parameter("flight_profiles_dir", "flight_profiles")
        self.declare_parameter("default_backend", "mock_ssh")
        self.declare_parameter("telemetry_rate", 5.0)          # Hz
        self.declare_parameter("telemetry_stale_timeout", 3.0)  # s (reserved for future staleness use)
        self.declare_parameter("goal_timeout", 60.0)             # s - time allowed to reach 'streaming'

        g = self.get_parameter
        # missions_root is workspace-relative by default - resolved against
        # the current working directory, i.e. `ros2 launch` is expected to
        # run from the repo root. Pass an absolute path to override.
        missions_root = str(g("missions_root").value)
        root_path = Path(missions_root)
        self.missions_root = root_path if root_path.is_absolute() else Path.cwd() / root_path
        self.drone_profiles_dir = Path(str(g("drone_profiles_dir").value))
        self.flight_profiles_dir = Path(str(g("flight_profiles_dir").value))
        self.default_backend = g("default_backend").value
        self.telemetry_rate = float(g("telemetry_rate").value)
        self.telemetry_stale_timeout = float(g("telemetry_stale_timeout").value)
        self.goal_timeout = float(g("goal_timeout").value)

        self.drone_profiles = list_drone_profiles(self.drone_profiles_dir)
        if not self.drone_profiles:
            self.get_logger().warn(
                f"no drone profiles found under {self.drone_profiles_dir} - "
                f"every ExecuteFlight goal will be rejected until some exist")
        self.flight_profiles = list_flight_profiles(self.flight_profiles_dir)

        self._cb = ReentrantCallbackGroup()

        self.poses = {}           # drone_name -> (x, y, z) ENU, latest known
        self._active_goals = {}    # drone_name -> ServerGoalHandle (goal bookkeeping only)
        self._deployers = {}       # drone_name -> deploy backend instance; persists past
                                    # goal success (that's what keeps a drone "holding")
        self._flight_ctx = {}      # drone_name -> (FlightProfile, start_time, mission_name, base_xyz)
        self._watchers = {}        # drone_name -> PX4TelemetryWatcher or None

        for name, profile in self.drone_profiles.items():
            self._make_pose_sub(name, profile)
            self._watchers[name] = self._make_watcher(name, profile)

        self._telemetry_pub = self.create_publisher(
            DroneTelemetry, "/mission/drone_telemetry", 10)
        self.create_timer(
            1.0 / self.telemetry_rate, self._publish_all_telemetry, callback_group=self._cb)

        self._server = ActionServer(
            self, ExecuteFlight, "execute_flight",
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb)
        self._terminate_srv = self.create_service(
            TerminateFlight, "terminate_flight", self._on_terminate,
            callback_group=self._cb)

        self.get_logger().info(
            f"mission_node ready: {len(self.drone_profiles)} drone profile(s), "
            f"{len(self.flight_profiles)} flight profile(s), "
            f"default_backend='{self.default_backend}', missions_root={self.missions_root}")

    # ------------------------------ pose intake --------------------------------
    def _make_pose_sub(self, name, profile):
        if profile.pose_topic_type == "Odometry":
            self.create_subscription(
                Odometry, profile.pose_topic,
                lambda m, n=name: self._store_pose(n, m.pose.pose.position),
                10, callback_group=self._cb)
        else:
            self.create_subscription(
                PoseStamped, profile.pose_topic,
                lambda m, n=name: self._store_pose(n, m.pose.position),
                10, callback_group=self._cb)

    def _store_pose(self, name, position):
        self.poses[name] = (position.x, position.y, position.z)

    def _make_watcher(self, name, profile):
        try:
            return PX4TelemetryWatcher(self, profile.namespace)
        except ImportError:
            self.get_logger().warn(
                f"px4_msgs not available - '{name}' reports unknown "
                f"nav_state/battery unless its deploy backend supplies mock telemetry")
            return None

    # ------------------------------ telemetry publish ---------------------------
    def _publish_all_telemetry(self):
        for name in self.drone_profiles:
            sample = self._sample_telemetry(name)
            deployer = self._deployers.get(name)
            ctx = self._flight_ctx.get(name)

            msg = DroneTelemetry()
            msg.drone_name = name
            msg.nav_state = int(sample.nav_state)
            msg.arming_state = int(sample.arming_state)
            msg.pre_flight_checks_pass = bool(sample.pre_flight_checks_pass)
            msg.gcs_connection_lost = bool(sample.gcs_connection_lost)
            msg.failsafe_active = bool(sample.failsafe_active)
            msg.link_ok = self._link_ok(name)
            msg.battery_remaining = float(sample.battery_remaining)
            msg.battery_voltage_v = float(sample.battery_voltage_v)
            msg.tracking_error = float(self._tracking_error(name))
            msg.mission_name = ctx[2] if ctx else ""
            msg.mission_phase = getattr(deployer, "phase", "idle") if deployer else "idle"
            msg.error_message = ""
            self._telemetry_pub.publish(msg)

    def _sample_telemetry(self, name):
        deployer = self._deployers.get(name)
        if deployer is not None and hasattr(deployer, "telemetry_sample"):
            sample = deployer.telemetry_sample()
            if sample is not None:
                return sample
        watcher = self._watchers.get(name)
        if watcher is not None:
            return watcher.sample()
        return TelemetrySample()

    def _link_ok(self, name):
        # Best-effort liveness: a drone we've heard a pose from is "linked".
        # Richer per-backend link checks (ssh reachability, DDS liveliness)
        # are a documented follow-up - see CLAUDE.md-style phasing notes.
        return name in self.poses

    def _tracking_error(self, name):
        ctx = self._flight_ctx.get(name)
        pose = self.poses.get(name)
        if ctx is None or pose is None:
            return -1.0
        flight_profile, start, _mission_name, base = ctx
        t = (self.get_clock().now() - start).nanoseconds / 1e9
        try:
            tx, ty, tz, _yaw = sample_trajectory(flight_profile.trajectory, t, base)
        except ValueError:
            return -1.0
        return _dist(pose, (tx, ty, tz))

    # ------------------------------ action callbacks ---------------------------
    def _goal_cb(self, goal_request):
        name = goal_request.drone_name
        if name not in self.drone_profiles:
            self.get_logger().warn(f"rejecting goal: unknown drone '{name}'")
            return GoalResponse.REJECT
        if name in self._active_goals:
            self.get_logger().warn(f"rejecting goal: '{name}' already has an active goal")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        name = goal_handle.request.drone_name
        self._active_goals[name] = goal_handle
        try:
            return self._run_goal(goal_handle)
        finally:
            self._active_goals.pop(name, None)

    def _run_goal(self, goal_handle):
        req = goal_handle.request
        name = req.drone_name
        profile = self.drone_profiles[name]
        result = ExecuteFlight.Result()

        try:
            flight_profile = self._resolve_flight_profile(req)
        except (KeyError, ValueError) as exc:
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            result.phase = "error"
            return result

        if not drone_supports(profile, flight_profile.required_capabilities):
            goal_handle.abort()
            result.success = False
            result.message = (
                f"'{name}' lacks required capabilities "
                f"{flight_profile.required_capabilities} (has {profile.capabilities})")
            result.phase = "error"
            return result

        backend = req.backend or profile.backend or self.default_backend
        mission_dir = self.missions_root / (req.mission_name or "_adhoc")
        run_dir = new_run_dir(mission_dir)
        deployer = make_deployer(backend, self, profile, req.mission_name, run_dir)

        ok, msg = deployer.precheck()
        if not ok:
            goal_handle.abort()
            result.success = False
            result.message = f"precheck failed: {msg}"
            result.phase = "error"
            return result

        ok, msg = deployer.deploy(flight_profile)
        if not ok:
            goal_handle.abort()
            result.success = False
            result.message = f"deploy failed: {msg}"
            result.phase = "error"
            return result

        # From here the deployer outlives this goal - it's what keeps the
        # drone streaming/holding after the goal succeeds, until Terminate.
        self._deployers[name] = deployer
        start = self.get_clock().now()
        base = self.poses.get(name, (0.0, 0.0, 0.0))
        self._flight_ctx[name] = (flight_profile, start, req.mission_name, base)

        rate = self.create_rate(5.0)
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                ok, msg = deployer.interrupt(hard=False)
                goal_handle.canceled()
                result.success = False
                result.message = f"interrupted (gentle hold): {msg}"
                result.phase = "held"
                return result

            phase = getattr(deployer, "phase", "streaming")
            fb = ExecuteFlight.Feedback()
            fb.phase = phase
            sample = self._sample_telemetry(name)
            fb.nav_state = int(sample.nav_state)
            fb.arming_state = int(sample.arming_state)
            fb.pre_flight_checks_pass = bool(sample.pre_flight_checks_pass)
            fb.gcs_connection_lost = bool(sample.gcs_connection_lost)
            fb.battery_remaining = float(sample.battery_remaining)
            fb.tracking_error = float(self._tracking_error(name))
            goal_handle.publish_feedback(fb)

            if phase == "done":
                # Terminate was called (from the GUI/service, not a cancel
                # on THIS goal) while this goal was still open - conclude it
                # cleanly instead of letting it spin or misreport a timeout.
                goal_handle.succeed()
                result.success = True
                result.message = f"'{name}' terminated"
                result.phase = "landed"
                return result
            if phase == "error":
                goal_handle.abort()
                result.success = False
                result.message = f"'{name}' deploy backend reported an error"
                result.phase = "error"
                self._deployers.pop(name, None)
                self._flight_ctx.pop(name, None)
                return result

            if phase not in ("streaming", "holding"):
                # Only the deploy transition (deploying/copying/starting) is
                # timed - once holding, the goal stays open indefinitely;
                # it only ends via Interrupt (cancel, above) or Terminate
                # ('done', above).
                elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
                if elapsed > self.goal_timeout:
                    goal_handle.abort()
                    result.success = False
                    result.message = (
                        f"deploy timed out after {self.goal_timeout:.0f} s "
                        f"(stuck at '{phase}')")
                    result.phase = "error"
                    self._deployers.pop(name, None)
                    self._flight_ctx.pop(name, None)
                    return result

            rate.sleep()

        goal_handle.abort()
        result.success = False
        result.message = "node shutting down"
        result.phase = "error"
        return result

    def _resolve_flight_profile(self, req):
        name = req.flight_profile
        if not name:
            if not req.mission_name:
                raise ValueError(
                    "no flight_profile given and no mission_name to resolve one from")
            spec = load_mission(self.missions_root, req.mission_name)
            name = spec.assignment.get(req.drone_name)
            if not name:
                raise ValueError(
                    f"mission '{req.mission_name}' has no flight profile "
                    f"assigned to '{req.drone_name}'")
        if name not in self.flight_profiles:
            raise KeyError(f"unknown flight profile '{name}'")
        return self.flight_profiles[name]

    # ------------------------------ terminate service ---------------------------
    def _on_terminate(self, request, response):
        names = []
        for name, deployer in self._deployers.items():
            if request.drone_name and name != request.drone_name:
                continue
            ctx = self._flight_ctx.get(name)
            mission_name = ctx[2] if ctx else ""
            if request.mission_name and mission_name != request.mission_name:
                continue
            names.append(name)

        if not names:
            response.success = False
            response.message = "no matching active drone(s) to terminate"
            response.log_paths = []
            return response

        log_paths = []
        messages = []
        overall_ok = True
        for name in names:
            deployer = self._deployers[name]
            if request.land_first:
                self._maybe_land(name, deployer)
            ok, msg, log_path = deployer.terminate()
            overall_ok = overall_ok and ok
            messages.append(f"'{name}': {msg}")
            if log_path:
                log_paths.append(log_path)
            self._deployers.pop(name, None)
            self._flight_ctx.pop(name, None)

        response.success = overall_ok
        response.message = "; ".join(messages)
        response.log_paths = log_paths
        return response

    def _maybe_land(self, name, deployer):
        """Best-effort AUTO_LAND before pulling logs. Only wired for the
        'local' backend today (its PX4Commander can send the command
        directly) - ssh/mock_ssh backends land as part of their own
        terminate() flow once real hardware validates that path (see
        deploy/backends.py's SSHDeployer docstring)."""
        commander = getattr(deployer, "commander", None)
        if commander is not None and hasattr(commander, "land"):
            commander.land()
            self.get_logger().info(f"'{name}': sent AUTO_LAND command")
        else:
            self.get_logger().info(
                f"'{name}': land_first requested but this backend doesn't "
                f"expose a direct land command - proceeding to log retrieval")


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
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
