
import math

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


def make_commander(backend, node, namespace, system_id, pose_topic, drone_id):
    """Factory: return the commander matching ``backend`` ('px4' or 'sim').

    ``drone_id`` is the display ID (index + 1) used for per-drone topics like
    ``/drone<ID>/active``. Real drones publish their active flag onboard, so
    only the sim backend uses it here.
    """
    if backend == "sim":
        return MockCommander(node, namespace, pose_topic, drone_id)
    return PX4Commander(node, namespace, system_id)


def _px4_qos() -> QoSProfile:
    # Matches the QoS PX4's uXRCE-DDS bridge expects on the /fmu/in topics.
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class PX4Commander:
    """Streams PX4 offboard position setpoints to one vehicle instance."""

    def __init__(self, node, namespace, system_id):
        # Lazy import so 'sim' backend / tests don't require px4_msgs.
        from px4_msgs.msg import (
            OffboardControlMode,
            TrajectorySetpoint,
            VehicleCommand,
        )

        self._OffboardControlMode = OffboardControlMode
        self._TrajectorySetpoint = TrajectorySetpoint
        self._VehicleCommand = VehicleCommand

        self.node = node
        self.ns = namespace
        self.system_id = int(system_id)

        qos = _px4_qos()
        base = f"/{namespace}/fmu/in" if namespace else "/fmu/in"
        self._ocm_pub = node.create_publisher(
            OffboardControlMode, f"{base}/offboard_control_mode", qos)
        self._sp_pub = node.create_publisher(
            TrajectorySetpoint, f"{base}/trajectory_setpoint", qos)
        self._cmd_pub = node.create_publisher(
            VehicleCommand, f"{base}/vehicle_command", qos)

        self._target_ned = None
        self._yaw_ned = 0.0
        self._ticks = 0
        self.armed = False

    def set_target_enu(self, x, y, z, yaw=0.0):
        # World ENU (x=east, y=north, z=up) -> PX4 NED (x=north, y=east, z=down).
        self._target_ned = (float(y), float(x), float(-z))
        # ENU yaw (CCW from east) -> NED yaw (CW from north).
        self._yaw_ned = float(math.pi / 2.0 - yaw)

    def publish_heartbeat(self):
        """Publish one offboard-mode + setpoint pair. Call at >= 2 Hz (we use the
        node's control rate). PX4 drops out of offboard if this stops."""
        if self._target_ned is None:
            return
        now = self._now_us()

        ocm = self._OffboardControlMode()
        ocm.timestamp = now
        ocm.position = True
        ocm.velocity = False
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        self._ocm_pub.publish(ocm)

        sp = self._TrajectorySetpoint()
        sp.timestamp = now
        sp.position = [self._target_ned[0], self._target_ned[1], self._target_ned[2]]
        sp.yaw = self._yaw_ned
        self._sp_pub.publish(sp)

        self._ticks += 1

    def ready_to_arm(self):
        """PX4 needs a stream of setpoints before it will accept offboard."""
        return self._ticks >= 10

    def engage_offboard(self):
        vc = self._VehicleCommand
        # base_mode custom (1), PX4 main mode OFFBOARD (6).
        self._send(vc.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def arm(self):
        vc = self._VehicleCommand
        self._send(vc.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.armed = True

    def disarm(self):
        vc = self._VehicleCommand
        self._send(vc.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.armed = False

    def _send(self, command, param1=0.0, param2=0.0):
        msg = self._VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = int(command)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = self.system_id
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._cmd_pub.publish(msg)

    def _now_us(self):
        return int(self.node.get_clock().now().nanoseconds / 1000)


class MockCommander:
    """Simulated drone: integrates toward the target, publishes a fake pose.

    Publishes ``PoseStamped`` on the same topic the formation node subscribes to,
    closing the loop so convergence feedback works end-to-end without hardware.
    """

    def __init__(self, node, namespace, pose_topic, drone_id, speed=1.5):
        self.node = node
        self.ns = namespace
        self.speed = float(speed)          # m/s
        self._pose = [0.0, 0.0, 0.0]
        self._target = None
        self._last = node.get_clock().now()
        self.armed = True
        self._pub = node.create_publisher(PoseStamped, pose_topic, 10)
        # Liveness heartbeat - a real drone publishes this onboard.
        self._active_pub = node.create_publisher(
            Bool, f"/drone{int(drone_id)}/active", 10)

    def set_target_enu(self, x, y, z, yaw=0.0):
        self._target = [float(x), float(y), float(z)]

    def publish_heartbeat(self):
        now = self.node.get_clock().now()
        dt = (now - self._last).nanoseconds / 1e9
        self._last = now
        if self._target is not None and dt > 0:
            step = self.speed * dt
            for k in range(3):
                delta = self._target[k] - self._pose[k]
                if abs(delta) <= step:
                    self._pose[k] = self._target[k]
                else:
                    self._pose[k] += math.copysign(step, delta)

        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = self._pose[0]
        msg.pose.position.y = self._pose[1]
        msg.pose.position.z = self._pose[2]
        msg.pose.orientation.w = 1.0
        self._pub.publish(msg)

        active = Bool()
        active.data = True
        self._active_pub.publish(active)

    def ready_to_arm(self):
        return True

    def engage_offboard(self):
        pass

    def arm(self):
        self.armed = True

    def disarm(self):
        self.armed = False
