"""Companion-computer-side node: reads one drone's rendered params file
(``deploy/launch_gen.py``'s ``render_single_drone_params`` +
``write_params_yaml``, copied here by ``SSHDeployer``) and flies that flight
profile for real, using the same ``drone_commander.PX4Commander`` this
workspace's ``formation_node`` uses - just for one drone, driven by a flight
profile's time-parametric trajectory (``flight_profiles.sample_trajectory``)
instead of a formation target.

This is the companion-computer side of the 'ssh' deploy backend
(``deploy/backends.py:SSHDeployer``), launched remotely via
``single_drone_flight.launch.py`` - not part of the host-side
``mission_node`` process. Coded now but **not yet validated against real
hardware** (no companion computers available yet - see ``SSHDeployer``'s
docstring); ``MockSSHDeployer`` exercises the equivalent flow with no
companion computer needed.

Known v1 limitation: ROS 2 parameters can't carry a nested list-of-lists, so
a 'waypoints'-type trajectory (whose ``points`` field is a list of
``[x, y, z, t]`` tuples) can't currently flow through the rendered params
YAML into this node's parameters. 'static_point' and 'circle' trajectories
(flat scalar/array fields only) work; 'waypoints' is a follow-up once this
path is validated against real hardware.

SIGINT (sent by ``SSHDeployer.interrupt(hard=False)``) freezes the last
commanded setpoint and keeps streaming it - the same free "hold in place"
behavior ``formation_node`` gets from an un-refreshed target - rather than
exiting, since PX4 offboard needs a continuous setpoint stream or it drops
out. This intentionally replaces rclpy's default SIGINT handler, so use
SIGTERM (or ``SSHDeployer.interrupt(hard=True)``) to actually stop the
process.
"""

import signal

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from formation_interface.drone_commander import make_commander
from formation_interface.flight_profiles import sample_trajectory


class FlightExecutorNode(Node):
    def __init__(self):
        # Flight profile params/trajectory are a variable-shaped dict
        # (depends on trajectory 'type'), so this node auto-declares
        # whatever the rendered params YAML actually contains instead of a
        # fixed declare_parameter list.
        super().__init__(
            "flight_executor_node",
            automatically_declare_parameters_from_overrides=True)

        def _get(name, default):
            try:
                return self.get_parameter(name).value
            except Exception:
                return default

        self.drone_name = _get("drone_name", "drone")
        self.namespace = _get("namespace", "")
        self.system_id = int(_get("system_id", 1))
        self.pose_topic = _get("pose_topic", "")
        self.pose_topic_type = _get("pose_topic_type", "PoseStamped")
        self.control_scheme = _get("control_scheme", "position")
        control_rate = float(_get("control_rate", 20.0))

        self.trajectory = {
            suffix: param.value
            for suffix, param in self.get_parameters_by_prefix("trajectory").items()
        }
        self.trajectory.setdefault("type", "static_point")

        self.pose = None
        self._base = None    # set on first pose received
        self._holding = False
        self._start = self.get_clock().now()

        self._make_pose_sub()
        self.commander = make_commander(
            "px4", self, self.namespace, self.system_id, self.pose_topic, 0,
            control_scheme=self.control_scheme)

        self.create_timer(1.0 / control_rate, self._control_loop)
        signal.signal(signal.SIGINT, self._on_sigint)

        self.get_logger().info(
            f"flight_executor_node ready: drone='{self.drone_name}', "
            f"ns='{self.namespace}', control_scheme={self.control_scheme}, "
            f"trajectory={self.trajectory.get('type')}")

    def _make_pose_sub(self):
        if self.pose_topic_type == "Odometry":
            self.create_subscription(
                Odometry, self.pose_topic,
                lambda m: self._store_pose(m.pose.pose.position), 10)
        else:
            self.create_subscription(
                PoseStamped, self.pose_topic,
                lambda m: self._store_pose(m.pose.position), 10)

    def _store_pose(self, position):
        self.pose = (position.x, position.y, position.z)
        if self._base is None:
            self._base = self.pose

    def _control_loop(self):
        # Must run every tick regardless of hold state - PX4 drops out of
        # offboard without a continuous setpoint stream (same principle as
        # formation_node's control-loop timer).
        self.commander.publish_heartbeat()
        if self._holding or self._base is None:
            return
        t = (self.get_clock().now() - self._start).nanoseconds / 1e9
        try:
            x, y, z, yaw = sample_trajectory(self.trajectory, t, self._base)
        except ValueError as exc:
            self.get_logger().error(f"trajectory sampling failed: {exc}")
            return
        self.commander.set_target_enu(x, y, z, yaw=yaw)

    def _on_sigint(self, signum, frame):
        self._holding = True
        self.get_logger().info("SIGINT received - holding last setpoint")


def main(args=None):
    rclpy.init(args=args)
    node = FlightExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
