"""Unit tests for deploy/backends.py.

Only LocalDeployer's/MockSSHDeployer's lightweight contract is tested here.
LocalDeployer.deploy() (which lazily imports drone_commander -> px4_msgs)
and every SSHDeployer method that shells out to real ssh/scp/rsync are
intentionally NOT exercised - no ROS workspace sourced / no real network in
CI, consistent with this repo not directly unit-testing formation_node.py's
rclpy internals either.
"""

from pathlib import Path

from formation_interface.deploy.backends import (
    LocalDeployer,
    MockSSHDeployer,
    SSHDeployer,
    make_deployer,
)
from formation_interface.drone_profiles import DroneProfile
from formation_interface.flight_profiles import FlightProfile


class _FakeTimer:
    """Stand-in for the rclpy Timer MockSSHDeployer gets back from
    node.create_timer() - fired manually instead of by a real event loop."""

    def __init__(self, callback):
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True

    def fire(self):
        if not self.cancelled:
            self.callback()


class _FakeNode:
    """Minimal stand-in for an rclpy Node: MockSSHDeployer only needs
    create_timer(period, callback)."""

    def __init__(self):
        self.timers = []

    def create_timer(self, period, callback, callback_group=None):
        timer = _FakeTimer(callback)
        self.timers.append(timer)
        return timer


def _drone(**overrides):
    kwargs = dict(
        name="drone1", namespace="px4_1", system_id=2,
        pose_topic="/vrpn_mocap/drone1/pose", capabilities=["gps"])
    kwargs.update(overrides)
    return DroneProfile(**kwargs)


def _flight():
    return FlightProfile(
        name="hover_low", description="hold", state="OFFBOARD",
        control_scheme="position", params={"altitude": 1.5},
        trajectory={"type": "static_point", "offset": [0.0, 0.0, 1.5]})


def test_make_deployer_dispatches_on_backend_string(tmp_path):
    node = _FakeNode()
    assert isinstance(
        make_deployer("local", node, _drone(), "m", tmp_path), LocalDeployer)
    assert isinstance(
        make_deployer("ssh", node, _drone(), "m", tmp_path), SSHDeployer)
    assert isinstance(
        make_deployer("mock_ssh", node, _drone(), "m", tmp_path), MockSSHDeployer)
    # Anything unrecognised (including "") falls back to the safe mock default.
    assert isinstance(
        make_deployer("", node, _drone(), "m", tmp_path), MockSSHDeployer)


def test_mock_ssh_deployer_full_happy_path(tmp_path):
    node = _FakeNode()
    deployer = make_deployer("mock_ssh", node, _drone(), "test_mission", tmp_path)

    ok, _msg = deployer.precheck()
    assert ok

    ok, _msg = deployer.deploy(_flight())
    assert ok
    assert (tmp_path / "params" / "drone1_params.yaml").exists()

    # Walk the fake timer through every phase to reach 'holding'.
    timer = node.timers[-1]
    for _ in range(10):
        if deployer.phase == "holding":
            break
        timer.fire()
    assert deployer.phase == "holding"

    sample = deployer.telemetry_sample()
    assert sample is not None
    assert sample.nav_state == 14   # OFFBOARD, once streaming/holding

    ok, _msg = deployer.interrupt(hard=False)
    assert ok
    assert deployer.phase == "holding"

    ok, _msg, log_path = deployer.terminate()
    assert ok
    assert log_path is not None
    assert (Path(log_path) / "mock.ulog").exists()
    assert timer.cancelled


def test_local_deployer_contract(tmp_path):
    node = _FakeNode()
    deployer = LocalDeployer(node, _drone(), "m", tmp_path)

    ok, _msg = deployer.precheck()
    assert ok
    assert deployer.telemetry_sample() is None

    ok, _msg = deployer.interrupt(hard=False)
    assert ok
    assert deployer.phase == "holding"

    ok, _msg, log_path = deployer.terminate()
    assert ok
    assert log_path is None
