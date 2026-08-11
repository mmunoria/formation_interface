
import math

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


def make_commander(backend, node, namespace, system_id, pose_topic, drone_id,
                    control_scheme="position"):
    """Factory: return the commander matching ``backend`` ('px4' or 'sim').

    ``drone_id`` is the display ID (index + 1) used for per-drone topics like
    ``/drone<ID>/active``. Real drones publish their active flag onboard, so
    only the sim backend uses it here.

    ``control_scheme`` ('position' | 'velocity' | 'trajectory') selects which
    PX4 offboard setpoint type ``PX4Commander`` streams (see its
    ``set_target_*`` methods). Ignored by the sim backend - ``MockCommander``
    only ever integrates a point mass toward a position target. Defaults to
    'position' so every existing call site (formation_node.py) is unaffected.
    """
    if backend == "sim":
        return MockCommander(node, namespace, pose_topic, drone_id)
    return PX4Commander(node, namespace, system_id, control_scheme=control_scheme)


def _px4_qos() -> QoSProfile:
    # Matches the QoS PX4's uXRCE-DDS bridge expects on the /fmu/in topics.
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class PX4Commander:
    """Streams PX4 offboard setpoints to one vehicle instance.

    ``control_scheme`` selects which ``TrajectorySetpoint`` fields are
    commanded (and which ``OffboardControlMode`` booleans are raised to
    match): 'position' (default, the only mode formation_node.py uses) sets
    only position via :meth:`set_target_enu`; 'velocity' sets only velocity
    via :meth:`set_target_velocity_enu`; 'trajectory' sets position plus
    optional velocity/acceleration feedforward via
    :meth:`set_target_trajectory_enu`.
    """

    def __init__(self, node, namespace, system_id, control_scheme="position"):
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
        self.control_scheme = control_scheme

        qos = _px4_qos()
        base = f"/{namespace}/fmu/in" if namespace else "/fmu/in"
        self._ocm_pub = node.create_publisher(
            OffboardControlMode, f"{base}/offboard_control_mode", qos)
        self._sp_pub = node.create_publisher(
            TrajectorySetpoint, f"{base}/trajectory_setpoint", qos)
        self._cmd_pub = node.create_publisher(
            VehicleCommand, f"{base}/vehicle_command", qos)

        self._target_ned = None
        self._velocity_ned = None
        self._accel_ned = None
        self._yaw_ned = 0.0
        self._yawspeed_ned = 0.0
        self._ticks = 0
        self.armed = False

    def set_target_enu(self, x, y, z, yaw=0.0):
        """Position control scheme (also the position component of
        'trajectory'). World ENU (x=east, y=north, z=up) -> PX4 NED
        (x=north, y=east, z=down)."""
        self._target_ned = (float(y), float(x), float(-z))
        # ENU yaw (CCW from east) -> NED yaw (CW from north).
        self._yaw_ned = float(math.pi / 2.0 - yaw)

    def set_target_velocity_enu(self, vx, vy, vz, yaw_rate=0.0):
        """Velocity control scheme: velocity in world ENU m/s, converted to
        PX4 NED the same way :meth:`set_target_enu` converts position."""
        self._velocity_ned = (float(vy), float(vx), float(-vz))
        self._yawspeed_ned = float(-yaw_rate)

    def set_target_trajectory_enu(self, pos, vel=None, accel=None, yaw=0.0):
        """Trajectory control scheme: required position plus optional
        velocity/acceleration feedforward, all in world ENU."""
        self.set_target_enu(pos[0], pos[1], pos[2], yaw=yaw)
        self._velocity_ned = (
            (float(vel[1]), float(vel[0]), float(-vel[2])) if vel is not None else None)
        self._accel_ned = (
            (float(accel[1]), float(accel[0]), float(-accel[2])) if accel is not None else None)

    def publish_heartbeat(self):
        """Publish one offboard-mode + setpoint pair. Call at >= 2 Hz (we use the
        node's control rate). PX4 drops out of offboard if this stops."""
        if self._target_ned is None and self._velocity_ned is None:
            return
        now = self._now_us()
        scheme = self.control_scheme

        ocm = self._OffboardControlMode()
        ocm.timestamp = now
        ocm.position = scheme in ("position", "trajectory") and self._target_ned is not None
        ocm.velocity = scheme in ("velocity", "trajectory") and self._velocity_ned is not None
        ocm.acceleration = scheme == "trajectory" and self._accel_ned is not None
        ocm.attitude = False
        ocm.body_rate = False
        self._ocm_pub.publish(ocm)

        # NaN means "don't control this field" (TrajectorySetpoint.msg).
        nan = float("nan")
        sp = self._TrajectorySetpoint()
        sp.timestamp = now
        sp.position = list(self._target_ned) if ocm.position else [nan, nan, nan]
        sp.velocity = list(self._velocity_ned) if ocm.velocity else [nan, nan, nan]
        sp.acceleration = list(self._accel_ned) if ocm.acceleration else [nan, nan, nan]
        sp.yaw = self._yaw_ned
        sp.yawspeed = self._yawspeed_ned
        self._sp_pub.publish(sp)

        self._ticks += 1

    def ready_to_arm(self):
        """PX4 needs a stream of setpoints before it will accept offboard."""
        return self._ticks >= 10

    def engage_offboard(self):
        vc = self._VehicleCommand
        # base_mode custom (1), PX4 main mode OFFBOARD (6).
        self._send(vc.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def land(self):
        """Command PX4's own AUTO_LAND, used by mission_node's Terminate
        (land_first) handling for drones running the 'local' deploy backend."""
        self._send(self._VehicleCommand.VEHICLE_CMD_NAV_LAND)

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
