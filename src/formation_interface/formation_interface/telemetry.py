"""Live per-drone telemetry: battery, error flags, mode, sensor handshake,
remote-link status - the "important live states" from the mission
requirements.

``TelemetrySample`` is a plain dataclass with no ROS dependency, so
``MockSSHDeployer`` (``deploy/backends.py``) can construct one without
importing px4_msgs. ``PX4TelemetryWatcher`` is the real backend: it lazily
imports px4_msgs inside ``__init__``, exactly mirroring ``PX4Commander``'s
pattern (``drone_commander.py``), so sim/mock_ssh-only setups and unit tests
never need px4_msgs installed.
"""

from dataclasses import dataclass

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

# Mirrors the constants declared in formation_interfaces/msg/DroneTelemetry.msg.
NAV_STATE_UNKNOWN = 255
ARMING_STATE_UNKNOWN = 0


@dataclass
class TelemetrySample:
    nav_state: int = NAV_STATE_UNKNOWN
    arming_state: int = ARMING_STATE_UNKNOWN
    pre_flight_checks_pass: bool = False
    gcs_connection_lost: bool = True
    failsafe_active: bool = False
    battery_remaining: float = -1.0     # 0..1, -1 if unknown
    battery_voltage_v: float = -1.0     # -1 if unknown


def _px4_out_qos() -> QoSProfile:
    # Matches PX4's uXRCE-DDS bridge QoS on /fmu/out topics (best-effort,
    # volatile, shallow history) - distinct from drone_commander._px4_qos(),
    # which is for the /fmu/in setpoint stream we publish, not subscribe to.
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


class PX4TelemetryWatcher:
    """Subscribes to one drone's VehicleStatus/BatteryStatus, independent of
    any action-goal lifecycle - construct one per configured drone at node
    startup so idle drones still report live telemetry (same principle as
    formation_node's control-loop timer: must never stall regardless of
    goal state)."""

    def __init__(self, node, namespace):
        # Lazy import so sim/mock-only setups and tests don't require px4_msgs.
        from px4_msgs.msg import BatteryStatus, VehicleStatus

        self._latest_status = None
        self._latest_battery = None

        qos = _px4_out_qos()
        base = f"/{namespace}/fmu/out" if namespace else "/fmu/out"
        node.create_subscription(
            VehicleStatus, f"{base}/vehicle_status", self._on_status, qos)
        node.create_subscription(
            BatteryStatus, f"{base}/battery_status", self._on_battery, qos)

    def _on_status(self, msg):
        self._latest_status = msg

    def _on_battery(self, msg):
        self._latest_battery = msg

    def sample(self) -> TelemetrySample:
        s = self._latest_status
        b = self._latest_battery
        return TelemetrySample(
            nav_state=int(s.nav_state) if s is not None else NAV_STATE_UNKNOWN,
            arming_state=int(s.arming_state) if s is not None else ARMING_STATE_UNKNOWN,
            pre_flight_checks_pass=bool(s.pre_flight_checks_pass) if s is not None else False,
            gcs_connection_lost=bool(s.gcs_connection_lost) if s is not None else True,
            failsafe_active=bool(s.failsafe) if s is not None else False,
            battery_remaining=float(b.remaining) if b is not None and b.connected else -1.0,
            battery_voltage_v=float(b.voltage_v) if b is not None and b.connected else -1.0,
        )
