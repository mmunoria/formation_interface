"""Deploy backends: how a flight profile actually gets onto a drone and
starts running. Mirrors ``drone_commander.make_commander``'s factory +
lazy-import pattern exactly - every backend exposes the same duck-typed
interface (``precheck``, ``deploy``, ``interrupt``, ``terminate``, and the
mock's ``telemetry_sample``), so ``mission_node`` never branches on backend.
"""

from pathlib import Path

from formation_interface.deploy import launch_gen


def make_deployer(backend, node, drone_profile, mission_name, run_dir):
    """Factory: return the deployer matching ``backend``.

    ``backend`` is ``'local'`` | ``'ssh'`` | anything else (including ``''``
    or ``'mock_ssh'``), which falls through to :class:`MockSSHDeployer` - the
    safe, no-hardware-required default.
    """
    if backend == "local":
        return LocalDeployer(node, drone_profile, mission_name, run_dir)
    if backend == "ssh":
        return SSHDeployer(node, drone_profile, mission_name, run_dir)
    return MockSSHDeployer(node, drone_profile, mission_name, run_dir)


class LocalDeployer:
    """backend='local' - no SSH, no companion computer: ``mission_node``'s
    own control loop streams setpoints in-process via
    ``drone_commander.make_commander()``, exactly like ``formation_node``
    does today."""

    def __init__(self, node, drone_profile, mission_name, run_dir):
        self.node = node
        self.drone_profile = drone_profile
        self.mission_name = mission_name
        self.run_dir = Path(run_dir)
        self.commander = None
        self._phase = "idle"

    @property
    def phase(self):
        return self._phase

    def precheck(self):
        return True, "in-process (no remote link to check)"

    def deploy(self, flight_profile):
        # Lazy import: drone_commander.py imports rclpy/geometry_msgs at
        # module scope, so keeping this here (not a module-level import in
        # backends.py) means MockSSHDeployer/SSHDeployer - and this class's
        # own precheck()/telemetry_sample()/terminate() - stay importable
        # and unit-testable with no ROS workspace sourced.
        from formation_interface.drone_commander import make_commander

        self.commander = make_commander(
            "px4", self.node, self.drone_profile.namespace,
            self.drone_profile.system_id, self.drone_profile.pose_topic, 0,
            control_scheme=flight_profile.control_scheme)
        self._phase = "streaming"
        return True, "commander ready"

    def telemetry_sample(self):
        return None   # relies on mission_node's PX4TelemetryWatcher instead

    def interrupt(self, hard=False):
        self._phase = "holding"
        return True, "holding (setpoint frozen by caller)"

    def terminate(self):
        self._phase = "done"
        return True, "no companion computer to pull logs from", None


class SSHDeployer:
    """backend='ssh': real per-drone companion-computer deploy over the
    system's own ssh/scp/rsync binaries via ``subprocess`` (no new pip
    dependency). Companion computers today run loose scripts (see
    ``from_drone/mocap_update.py``), not a full colcon workspace, so what
    gets copied is one small rendered params file, not a package install.

    Not validated against real hardware yet - :class:`MockSSHDeployer`
    satisfies the identical interface for dry runs; switching a drone
    profile's ``backend:`` from ``mock_ssh`` to ``ssh`` is the only change
    needed once a companion computer is reachable.
    """

    def __init__(self, node, drone_profile, mission_name, run_dir):
        import subprocess   # stdlib; isolated here for parity with the
        self._subprocess = subprocess   # lazy-import convention used
                                         # elsewhere (PX4Commander/px4_msgs),
                                         # and so a future paramiko swap is one spot
        self.node = node
        self.drone_profile = drone_profile
        self.mission_name = mission_name
        self.run_dir = Path(run_dir)
        self._remote_pid = None
        self._phase = "idle"

    @property
    def phase(self):
        return self._phase

    def telemetry_sample(self):
        return None   # relies on mission_node's PX4TelemetryWatcher instead

    def _ssh_target(self):
        p = self.drone_profile
        return f"{p.ssh_user}@{p.ssh_host}" if p.ssh_user else p.ssh_host

    def _ssh_base_cmd(self):
        p = self.drone_profile
        cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5"]
        if p.ssh_key_path:
            cmd += ["-i", str(Path(p.ssh_key_path).expanduser())]
        cmd.append(self._ssh_target())
        return cmd

    def _run(self, cmd, timeout):
        try:
            return True, self._subprocess.run(cmd, capture_output=True, timeout=timeout)
        except Exception as exc:
            return False, exc

    def precheck(self):
        if not self.drone_profile.ssh_host:
            return False, "drone profile has no ssh_host configured"
        ok, result = self._run(self._ssh_base_cmd() + ["true"], timeout=8)
        if not ok:
            return False, f"ssh precheck failed: {result}"
        if result.returncode != 0:
            return False, f"ssh precheck failed: {result.stderr.decode(errors='replace').strip()}"
        return True, "ssh reachable"

    def deploy(self, flight_profile):
        p = self.drone_profile
        self._phase = "deploying"
        params = launch_gen.render_single_drone_params(
            p, flight_profile, self.mission_name, self.run_dir.name)
        local_path = launch_gen.write_params_yaml(
            params, self.run_dir / "params" / f"{p.name}_params.yaml")
        remote_path = f"{p.remote_workspace_path.rstrip('/')}/{local_path.name}"

        self._phase = "copying"
        scp_cmd = ["scp"]
        if p.ssh_key_path:
            scp_cmd += ["-i", str(Path(p.ssh_key_path).expanduser())]
        scp_cmd += [str(local_path), f"{self._ssh_target()}:{remote_path}"]
        ok, result = self._run(scp_cmd, timeout=15)
        if not ok:
            return False, f"scp failed: {result}"
        if result.returncode != 0:
            return False, f"scp failed: {result.stderr.decode(errors='replace').strip()}"

        self._phase = "starting"
        remote_log = f"{p.remote_log_dir.rstrip('/')}/{p.name}_{self.run_dir.name}.log"
        start_cmd = (
            "nohup ros2 launch formation_interface single_drone_flight.launch.py "
            f"params_file:={remote_path} > {remote_log} 2>&1 & echo $!")
        ok, result = self._run(self._ssh_base_cmd() + [start_cmd], timeout=15)
        if not ok:
            self._phase = "error"
            return False, f"remote start failed: {result}"
        if result.returncode != 0:
            self._phase = "error"
            return False, f"remote start failed: {result.stderr.decode(errors='replace').strip()}"
        self._remote_pid = result.stdout.decode(errors="replace").strip()
        self._phase = "streaming"
        return True, f"started remotely (pid {self._remote_pid})"

    def interrupt(self, hard=False):
        if not self._remote_pid:
            return False, "no tracked remote process"
        sig = "SIGKILL" if hard else "SIGINT"
        ok, result = self._run(
            self._ssh_base_cmd() + [f"kill -{sig} {self._remote_pid}"], timeout=10)
        if not ok:
            return False, f"remote interrupt failed: {result}"
        if result.returncode != 0:
            return False, f"remote interrupt failed: {result.stderr.decode(errors='replace').strip()}"
        self._phase = "holding"
        return True, f"sent {sig} to remote pid {self._remote_pid}"

    def terminate(self):
        p = self.drone_profile
        if self._remote_pid:
            self.interrupt(hard=False)
        self._phase = "terminating"
        local_log_dir = self.run_dir / "logs" / p.name
        local_log_dir.mkdir(parents=True, exist_ok=True)
        rsync_cmd = ["rsync", "-az"]
        if p.ssh_key_path:
            rsync_cmd += ["-e", f"ssh -i {Path(p.ssh_key_path).expanduser()}"]
        rsync_cmd += [
            f"{self._ssh_target()}:{p.remote_log_dir.rstrip('/')}/",
            str(local_log_dir) + "/",
        ]
        ok, result = self._run(rsync_cmd, timeout=60)
        if not ok:
            return False, f"log retrieval failed: {result}", None
        if result.returncode != 0:
            return False, f"log retrieval failed: {result.stderr.decode(errors='replace').strip()}", None
        self._phase = "done"
        return True, "logs retrieved", str(local_log_dir)


class MockSSHDeployer:
    """backend='mock_ssh' - the safe default. Zero real network: same
    "headless node + declare_parameter config + internal simulated state +
    create_timer callbacks" shape as ``mock_optitrack.py``, but embedded as a
    plain class driven by a timer on the *passed-in* ``mission_node`` (one
    instance per active goal, not a separate process/executable), so the
    full Start -> deploy -> telemetry -> interrupt -> terminate -> log-pull
    flow is demoable and testable with no hardware.
    """

    _PHASES = ["deploying", "copying", "starting", "streaming"]

    def __init__(self, node, drone_profile, mission_name, run_dir, step_period=0.5):
        self.node = node
        self.drone_profile = drone_profile
        self.mission_name = mission_name
        self.run_dir = Path(run_dir)
        self._phase_idx = -1
        self._phase = "idle"
        self._holding = False
        self._timer = node.create_timer(step_period, self._advance_phase)

    def precheck(self):
        return True, "mock: reachable"

    def deploy(self, flight_profile):
        p = self.drone_profile
        params = launch_gen.render_single_drone_params(
            p, flight_profile, self.mission_name, self.run_dir.name)
        launch_gen.write_params_yaml(
            params, self.run_dir / "params" / f"{p.name}_params.yaml")
        self._phase_idx = 0
        self._phase = self._PHASES[0]
        return True, "mock deploy started"

    def _advance_phase(self):
        if self._holding or self._phase_idx < 0:
            return
        if self._phase_idx < len(self._PHASES) - 1:
            self._phase_idx += 1
            self._phase = self._PHASES[self._phase_idx]
        else:
            self._holding = True
            self._phase = "holding"

    @property
    def phase(self):
        return self._phase

    def telemetry_sample(self):
        """A fake VehicleStatus-shaped sample, bypassing px4_msgs entirely."""
        from formation_interface.telemetry import TelemetrySample
        if self._phase == "idle":
            return TelemetrySample()
        streaming = self._phase in ("streaming", "holding")
        return TelemetrySample(
            nav_state=14 if streaming else 255,    # OFFBOARD once streaming
            arming_state=2 if streaming else 1,      # ARMED once streaming
            pre_flight_checks_pass=True,
            gcs_connection_lost=False,
            failsafe_active=False,
            battery_remaining=0.85,
            battery_voltage_v=15.8,
        )

    def interrupt(self, hard=False):
        self._holding = True
        self._phase = "holding"
        return True, "mock: holding"

    def terminate(self):
        p = self.drone_profile
        log_dir = self.run_dir / "logs" / p.name
        log_dir.mkdir(parents=True, exist_ok=True)
        (log_dir / "mock.ulog").write_text(
            f"mock log for {p.name}, mission {self.mission_name}\n")
        if self._timer is not None:
            self._timer.cancel()
        self._phase = "done"
        return True, "mock: logs retrieved", str(log_dir)
