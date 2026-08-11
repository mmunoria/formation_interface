# Mission Manager Subsystem

Reference doc for the mission-manager code added on top of the existing
formation-flying stack (`formation_node.py` / `drone_commander.py` /
`formations.py`). It's a second, independent layer — same relationship the
monitor subsystem (`active_tracker.py` / `monitor_gui.py`) has to
`formation_node` — added to satisfy a set of flight-operations requirements:
define a flight profile independent of any one drone, define a drone's own
parameters independent of any one mission, assign drones to flights via a
GUI, start/interrupt (gentle)/terminate one or all drones, monitor live
swarm health, and deploy/retrieve logs from each drone.

The implementation plan this was built from is
`/home/progress/.claude/plans/validated-growing-orbit.md` — this doc is the
as-built reference; consult the plan for the original design rationale if
something here seems terse.

## Requirement → implementation map

| Requirement | File(s) |
|---|---|
| Flight profile (state / control scheme / shared params / trajectory), drone-agnostic | `flight_profiles/*.yaml`, `flight_profiles.py` |
| Drone parameters, drone-specific, not mission-specific | `config/drone_profiles/*.yaml`, `drone_profiles.py` |
| Assign drones to a flight, GUI, quickly varying | `mission_gui.py` (assignment table) |
| Start (one/all) | `mission_gui.py` buttons → `ExecuteFlight` action goals |
| Interrupt (one/all), gentle | `ExecuteFlight` goal cancel → `deployer.interrupt(hard=False)` |
| Live telemetry: battery, error flags, mode, tracking error, sensor handshake, remote link, capability | `DroneTelemetry.msg`, `telemetry.py`, `mission_node.py`'s publisher, `mission_gui.py`'s table |
| Mission name/description, README, no accidental overwrites | `mission_store.py` |
| Feature toggles | `mission_gui.py` checkboxes → `mission.yaml`'s `features` |
| Generate config → copy to drone → execute | `deploy/launch_gen.py` + `deploy/backends.py:SSHDeployer` |
| Terminate → copy logs back | `TerminateFlight.srv`, `deploy/backends.py`'s `terminate()` methods |

## New/changed files

### `formation_interfaces` package (interfaces)

| File | What it does |
|---|---|
| `action/ExecuteFlight.action` | Per-drone action (not per-swarm like `GoToFormation`). Goal: `mission_name`, `drone_name`, optional `flight_profile`/`backend` overrides. Feedback: deploy phase, PX4 mode/arming, sensor/link flags, battery, tracking error. Result: success, message, final phase. |
| `srv/TerminateFlight.srv` | Request: `mission_name`, optional `drone_name` (empty = every active drone under that mission), `land_first`. Response: success, message, `log_paths[]`. Deliberately separate from cancelling `ExecuteFlight` — cancel only holds; this lands (best-effort) and always pulls logs. |
| `msg/DroneTelemetry.msg` | One drone's live status, published on a shared topic tagged by `drone_name` (string, not the index-based `drone_id` used by `DronePose`/`DroneActive`) — battery, PX4 nav/arming state (local enum copies, no px4_msgs dependency), sensor-checks-pass, gcs-connection-lost, link_ok, tracking_error, mission_name/phase. |
| `CMakeLists.txt` (edited) | Registers the three files above alongside the existing `GoToFormation`/`DronePose`/`DroneActive`. |

### `formation_interface` package — ROS-free core (unit-tested directly, no ROS sourcing needed)

| File | What it does |
|---|---|
| `flight_profiles.py` | `FlightProfile` dataclass + `load_flight_profile()`/`list_flight_profiles()` (validate `state` against a hand-kept copy of PX4's `NAVIGATION_STATE_*` enum, `control_scheme` against `position`/`velocity`/`trajectory`). `sample_trajectory(spec, t, base)` sample a reference trajectory at time `t`: `static_point`, `circle`, or `waypoints`. |
| `drone_profiles.py` | `DroneProfile` dataclass (namespace, system_id, pose topic, `capabilities`, speed/accel limits, deploy `backend`, SSH connection fields) + `load_drone_profile()`/`list_drone_profiles()`/`drone_supports()`. One file per physical drone, unlike `drones.yaml`'s parallel-index-array shape — see the file's own docstring for why. |
| `mission_store.py` | `MissionSpec` dataclass + `create_mission()` (the anti-overwrite guard — a plain directory-existence check, raises `MissionExistsError` unless `force=True`), `load_mission()`/`save_mission()` (never touches `README.md`), `list_missions()`, `new_run_dir()` (one fresh timestamped dir per Start, retried on same-second collision). |

### `formation_interface` package — deploy backends

| File | What it does |
|---|---|
| `deploy/launch_gen.py` | Pure functions, no ROS/side effects beyond the explicit write: `render_single_drone_params()` builds the flat param dict a companion computer needs; `write_params_yaml()` writes it in the `drones.yaml`-style `/**: ros__parameters:` wildcard shape (this file, unlike the profile/mission YAMLs, actually gets loaded via `declare_parameter`). |
| `deploy/backends.py` | `make_deployer(backend, ...)` factory (mirrors `drone_commander.make_commander`'s pattern) returning one of: **`LocalDeployer`** (`backend: local` — no SSH, streams setpoints in-process via `drone_commander.make_commander()`, like `formation_node` does); **`SSHDeployer`** (`backend: ssh` — real `subprocess`+ssh/scp/rsync to a companion computer; **not yet validated against real hardware**); **`MockSSHDeployer`** (`backend: mock_ssh`, the default — zero network, walks itself through `deploying→copying→starting→streaming` on a timer, fakes telemetry, writes a fake log file on terminate). All three expose the same `precheck()`/`deploy()`/`interrupt()`/`terminate()`/`phase` contract. |

### `formation_interface` package — ROS-dependent nodes/GUI

| File | What it does |
|---|---|
| `telemetry.py` | `TelemetrySample` dataclass (no ROS dependency) + `PX4TelemetryWatcher` (subscribes to one drone's `VehicleStatus`/`BatteryStatus` on `/<namespace>/fmu/out/...`, lazily imports `px4_msgs` exactly like `PX4Commander` does). |
| `mission_node.py` | The action server + service. Accepts **one concurrent `ExecuteFlight` goal per `drone_name`** (unlike `formation_node`'s single-goal-for-the-whole-swarm gate) — every drone independently idle/deploying/streaming/holding. Runs a `PX4TelemetryWatcher` per configured drone regardless of goal state, publishes `DroneTelemetry` at `telemetry_rate` Hz. A goal stays open for as long as the drone is streaming/holding — it only ends via cancel (gentle interrupt) or an external `TerminateFlight` call; it does **not** self-complete once flying starts. Resolves a drone's flight profile from the goal (explicit override) or the mission's `assignment` map, checks `drone_supports()` before deploying. |
| `mission_gui.py` | Tkinter control panel, same two-thread model as `gui_node.py`/`monitor_gui.py` (rclpy on a daemon thread, Tk on the main thread, `queue.Queue` + `after()` poll for action/feedback events). Panels: Mission (create with an overwrite-confirm dialog, pick an existing mission), Assignment (one row per drone profile, flight-profile dropdown filtered by capability, per-row Start/Interrupt/Terminate), All-drones controls, Features, live Telemetry table. |
| `flight_executor_node.py` | **Companion-computer-side**, launched remotely by `SSHDeployer` via `single_drone_flight.launch.py` — reads one drone's rendered params file, drives `drone_commander.PX4Commander` through a flight profile's trajectory in real time. **Not yet validated against real hardware.** SIGINT freezes the last setpoint (gentle hold) instead of exiting — replaces rclpy's default SIGINT handler, so use SIGTERM to actually stop the process. |
| `drone_commander.py` (edited) | Additive only — existing call sites (`formation_node.py`) are unaffected. `make_commander()`/`PX4Commander.__init__` gained a `control_scheme="position"` kwarg; added `set_target_velocity_enu()`, `set_target_trajectory_enu()`, and `land()`; `publish_heartbeat()` now dispatches which `TrajectorySetpoint`/`OffboardControlMode` fields are commanded based on `control_scheme`, using NaN for uncommanded fields per PX4's documented convention. |

### Launch files & config

| File | What it does |
|---|---|
| `launch/mission.launch.py` | Starts `mission_node`. `backend:=` arg (default `mock_ssh`) overrides every drone profile's default deploy backend for the run. |
| `launch/mission_gui.launch.py` | Starts `mission_gui`. |
| `launch/single_drone_flight.launch.py` | Static, checked-in launch file `SSHDeployer` runs remotely; takes a `params_file:=` arg (the file `launch_gen.write_params_yaml()` rendered and scp'd over). |
| `config/mission_manager.yaml` | Shared `/**:` params for `mission_node`/`mission_gui`: `missions_root`, `drone_profiles_dir`, `flight_profiles_dir`, `default_backend`, `telemetry_rate`, `goal_timeout`. |
| `config/drone_profiles/drone1.yaml`, `drone2.yaml` (+ `README.md`) | Example drone profiles, namespace/system_id/pose_topic matching the real entries in `config/drones.yaml`. |
| `flight_profiles/hover_low.yaml`, `circle_survey.yaml` (+ `README.md`) | Example flight profiles (`static_point` and `circle` trajectories). |

### Tests (`test/`)

`test_flight_profiles.py`, `test_drone_profiles.py`, `test_mission_store.py`, `test_launch_gen.py`, `test_deploy_backends.py` — 32 tests total, plain-pytest style matching `test_formations.py`/`test_planning.py` (no ROS sourcing needed; `test_deploy_backends.py` uses a fake `Node` stand-in so `MockSSHDeployer`/`LocalDeployer` are exercised without rclpy).

### Other edits

- `setup.py` — new `console_scripts` (`mission_node`, `mission_gui`, `flight_executor_node`) and `data_files` globs for `config/drone_profiles/` and `flight_profiles/`.
- `package.xml` — added `python3-yaml` (the new profile/mission loaders use `yaml.safe_load`).
- `.gitignore` — `missions/*/runs/` ignored (per-run params/logs); `mission.yaml`/`README.md` stay tracked.

## Runtime data layout (`missions/`, repo root)

```
missions/<mission_name>/
  mission.yaml        # assignment + features, overwritable (never silently)
  README.md            # written once, never regenerated
  runs/<UTC-timestamp>/
    params/<drone_name>_params.yaml   # what was rendered/copied to the drone
    logs/<drone_name>/...              # pulled back on Terminate
```

## Verified working (mock_ssh, no hardware)

Build, full test suite, and a live `mission_node` run were used to confirm: mission create/anti-overwrite, capability-mismatch rejection, the full `deploying→copying→starting→streaming` phase walk with live telemetry, gentle Interrupt correctly holding a **streaming** drone without ending its goal, and Terminate landing (best-effort) + pulling a log file into the right run directory. See the plan doc's "Verification" section for the exact commands.

One bug was caught and fixed during this verification: the action was originally completing (`goal_handle.succeed()`) as soon as a drone reached `streaming`, which meant a drone that had settled into `holding` had no open goal left to cancel — Interrupt would silently no-op on exactly the drones an operator would want to interrupt. Fixed so the goal stays open for the entire streaming/holding duration and only concludes via cancel (interrupted) or an external Terminate call (landed).

## What needs to change in the future

**Before flying on real hardware:**
- `SSHDeployer` (real `ssh`/`scp`/`rsync` calls) and `flight_executor_node.py` are written but **not validated against a real companion computer** — there wasn't one available. Test against one real drone before trusting the `ssh` backend.
- `flight_executor_node.py` can't currently receive a `waypoints`-type trajectory: ROS 2 parameters can't carry a nested list-of-lists, so `trajectory.points` (a list of `[x, y, z, t]` tuples) doesn't survive the rendered-params-YAML → node-parameters path. `static_point` and `circle` (flat scalar/array fields) work. Needs either a different transport for that one field (e.g. ship it as a JSON string parameter and parse it) or restructuring the points list into parallel flat arrays.
- `_maybe_land()` in `mission_node.py` (the `land_first` handling in Terminate) only sends a real `AUTO_LAND` command for the `local` backend, which is the only one with a live `PX4Commander` on the host. For `ssh`, landing should happen on the companion computer as part of its own terminate flow (`flight_executor_node.py` doesn't currently do this — it just holds on SIGINT). For `mock_ssh`, there's nothing to land.
- `NAV_STATE_NAMES` in `flight_profiles.py` and the enum constants in `DroneTelemetry.msg` are hand-copied from `px4_msgs/msg/VehicleStatus.msg`. If the vendored `px4_msgs` is ever updated to a PX4 version with different enum values, both need updating by hand — nothing enforces they stay in sync.

**Design/scope gaps to revisit:**
- `_link_ok()` in `mission_node.py` is a placeholder — "have we ever heard a pose for this drone" — not a real per-backend liveness check (SSH reachability, DDS liveliness). Fine for now, not a real signal of a mid-flight dropped connection.
- Hard/emergency interrupt (`interrupt(hard=True)`, which `SSHDeployer` already supports via `SIGKILL`) has no path exposed from the GUI or the `ExecuteFlight` action goal — only gentle interrupt is wired up end-to-end, matching the requirement as given, but worth adding if an E-stop is ever needed.
- `config/drone_profiles/*.yaml` and the pre-existing `config/drones.yaml` are **not kept in sync automatically** — both describe overlapping facts (namespace, system_id, pose topic) for the formation stack vs. the mission stack respectively. A namespace change today means editing both by hand; consider generating one from the other, or merging the two config surfaces, next time either changes.
- `features.record_bag` exists as a toggle in `mission.yaml` but nothing reads it yet — no `ros2 bag` integration.
- `FailsafeFlags`-level per-field error detail (beyond `failsafe_active`/`gcs_connection_lost`/`pre_flight_checks_pass`) was scoped out of v1; `DroneTelemetry.error_message` exists as a free-text field for this but nothing populates it yet.
- `mission_gui.py` has no map view and no in-GUI editing of drone/flight profile YAML (by design, to keep v1 scope down) — profiles are hand-edited and picked up via the "reload profiles" button. Revisit if that friction becomes a problem.
- `missions_root` defaults to a `missions` path resolved relative to the current working directory `ros2 launch` was run from, not an absolute path — documented in `config/mission_manager.yaml` with a `>>> CHANGE <<<` comment, same convention as `drones.yaml`'s topic list, but easy to get wrong if `ros2 launch` isn't run from the repo root.

**Not a regression, but worth knowing:** `colcon test`'s flake8/pep257 checks fail on this package both before and after this work (verified against a clean baseline) — it's a pre-existing, repo-wide condition (`gui_node.py`, `formation_node.py`, etc. all fail the same way), not something introduced here. The new files match the existing code's actual style; fixing the underlying lint config mismatch is a separate, unrelated cleanup task.
