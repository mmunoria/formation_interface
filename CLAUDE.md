# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A ROS 2 colcon workspace for commanding a swarm of PX4 drones into formations,
using OptiTrack (via `vrpn_mocap` or similar) for position feedback. An
operator (terminal menu or Tkinter GUI) sends a `GoToFormation` action goal to
a formation node running near the drones; the formation node computes
per-drone targets, assigns them, and streams PX4 offboard setpoints until the
swarm converges.

```
HOST PC                                         DRONES (PX4)
┌──────────────────────────┐                    ┌────────────────────────────┐
│ interface_node / gui_node│  GoToFormation     │ formation_node             │
│   menu/GUI → action goal─┼──► action ────────►│  ActionServer              │
│   ◄── feedback / result  │                    │  • compute per-drone target│
│                          │   /<ns>/fmu/in/... │  • assign (nearest-first)  │
│ OptiTrack (Motive)       │  ◄─ setpoints ─────┼──• stream PX4 offboard ───►│ autopilot
│   → mocap bridge         │                    │  • watch poses → feedback  │
│   → /.../droneN/pose ────┼──► poses ─────────►│                            │
└──────────────────────────┘                    └────────────────────────────┘
```

Only `src/formation_interface` and `src/formation_interfaces` are this
project's own code. `src/px4_msgs` and `src/px4_ros_com` are vendored
third-party ROS packages (each its own git repo) — treat them as read-only
dependencies, not code to modify. `build/`, `install/`, and `log/` are colcon
build artifacts (gitignored); never hand-edit anything under them.

## Packages

| Package                | Build type    | Contents                                                          |
|-------------------------|--------------|--------------------------------------------------------------------|
| `formation_interfaces` | `ament_cmake` | `GoToFormation.action`, `DronePose.msg`, `DroneActive.msg`         |
| `formation_interface`  | `ament_python`| `interface_node`, `gui_node`, `formation_node`, formation math, PX4 backend, live monitor + planner |

Custom ROS 2 actions **must** live in an `ament_cmake` package, which is why
the action definition is split into its own package from the Python nodes.

## Build / run / test

Workspace root is this directory (has `src/`). Build the interfaces package
first since `formation_interface` depends on the generated action types:

```bash
colcon build --packages-select formation_interfaces
colcon build --packages-select formation_interface
source install/setup.bash
```

`px4_msgs` (and `px4_ros_com` if used) must also be built in this workspace —
they already exist under `src/`.

Run, real hardware (edit `src/formation_interface/config/drones.yaml` first):
```bash
ros2 launch formation_interface formation.launch.py     # terminal 1
ros2 launch formation_interface gui.launch.py            # terminal 2, GUI ...
ros2 run formation_interface interface_node               # ... OR terminal menu
```

Dry run with no PX4 / no OptiTrack (mock drones simulate motion and publish
their own poses, closing the loop end-to-end):
```bash
ros2 launch formation_interface formation.launch.py backend:=sim
```

The live multi-drone monitor + planner (see Architecture below) is a
separate, independent launch — not part of the formation-flying stack above:
```bash
ros2 launch formation_interface monitor.launch.py               # real OptiTrack
ros2 launch formation_interface monitor.launch.py use_mock:=true # no hardware, simulated drones
```

Tests (ament pytest — flake8, pep257, copyright header, and the formation-math
unit tests) live in `src/formation_interface/test/`:
```bash
colcon test --packages-select formation_interface
colcon test-result --verbose

# a single test file, without colcon:
python3 -m pytest src/formation_interface/test/test_formations.py -v
```

`formations.py` has zero ROS dependencies specifically so it can be pytest'd
directly like this, without sourcing the workspace.

## Architecture

**`formations.py`** (`src/formation_interface/formation_interface/formations.py`)
— pure geometry, no ROS imports. Built-in generators (`line`, `v`, `circle`)
live in `_GENERATORS` and return an `(n, 2)` array of local XY offsets around
the formation origin; `FORMATIONS` (the public name list used by menus) is
derived from that dict's keys, so adding a formation is: write a `_foo(n,
spacing)` generator, add it to `_GENERATORS`, done. `transform_offsets()` maps
*any* per-drone offset list (built-in or arbitrary/custom) into the world ENU
frame — rotate by yaw, translate to center, stamp altitude (+ per-drone z for
3-D formations). `compute_targets()` is the built-in-formation convenience
wrapper around the same transform. Both the interface node and the formation
node import this module — it's the single source of truth for formation
geometry.

**`formation_node.py`** — the `GoToFormation` action server, meant to run
near/on the drones. On a goal: compute raw targets (via `formations.py`),
`_assign()` them to drones nearest-first (greedy, to minimize crossing paths;
falls back to index order until every drone has a pose), then hand each
target to that drone's *commander*. A free-running timer (`control_rate`, independent
of action-goal lifecycle) calls `publish_heartbeat()` on every commander —
PX4 offboard mode requires a continuous setpoint stream or it drops out,
so this loop must never stall regardless of goal state. The action's
execute loop polls `_errors()` (per-drone distance to target) at 5 Hz,
publishes feedback, and succeeds once every drone is within
`position_tolerance` for `settle_cycles` consecutive checks (debounce against
transient dips), or aborts on `goal_timeout` / cancel.

**`drone_commander.py`** — one commander per drone, chosen by the `backend`
param via `make_commander()`:
  - `PX4Commander` (`backend: px4`) streams `OffboardControlMode` +
    `TrajectorySetpoint` on `/<namespace>/fmu/in/...` (uXRCE-DDS), and sends
    `VehicleCommand`s for offboard-engage/arm/disarm. Converts world ENU
    targets to PX4's NED frame in `set_target_enu()`. All `px4_msgs` imports
    are lazy (inside `__init__`), so the `sim` backend and unit tests work
    without `px4_msgs` installed.
  - `MockCommander` (`backend: sim`) integrates a point mass toward the
    target at a fixed speed and publishes `PoseStamped` on the same topic the
    formation node subscribes to for that drone — this is what closes the
    loop for `backend:=sim` dry runs with no hardware or OptiTrack.

**`interface_node.py`** — terminal front-end. `send_formation()` is the one
method that talks to the action server (builds the goal, blocks on
`threading.Event` until result); everything else (menu text, custom-formation
prompts) is a thin caller of it. rclpy spins on a background thread so the
blocking `input()`-driven menu on the main thread doesn't block callbacks.
Custom/arbitrary formations (any offset per drone, not just the three
built-ins) are sent with `formation: "custom"` and a `custom_offsets` list on
the goal; they can be saved as named presets to `~/.formation_presets.json`,
shared between the terminal and GUI front-ends.

**`gui_node.py`** — Tkinter control panel (`GuiBackend` wraps the same
action-client pattern as `interface_node.py`; `App` is the Tk UI). Same
`GoToFormation` action, so it and `interface_node` are interchangeable, thin
clients — `formation_node` doesn't know or care which one sent a goal. Adds a
live top-down map from the pose topics and click-to-design: toggle design
mode, click one target per drone, then fly or save as a preset (same
`~/.formation_presets.json` presets file as the terminal interface).

## Live monitor + path planner (`active_tracker.py`, `path_planner.py`, `monitor_gui.py`)

A second, independent subsystem layered on top of the formation-flying stack
above — visualizes whichever drones are currently online and plans a route
for one drone around the others. It does **not** share code or state with
`formation_node`/`drone_commander.py`, and doesn't send anything to PX4; it's
visualize-and-plan only.

Unlike `config/drones.yaml`'s fixed, index-aligned drone list, this side is
fully dynamic: drones self-identify. `DronePose` (`drone_id` +
`geometry_msgs/PoseStamped`) and `DroneActive` (`drone_id` + `bool`) are each
published on one **shared** topic (`/optitrack/drone_pose`,
`/optitrack/drone_active`) by every drone, rather than one pose topic per
drone — so the drone count needs no config changes.

- **`active_tracker.py`** — ROS-free, like `formations.py`. `Drone` is a
  dataclass (`id`, `position`, `orientation`, `active`, `planned_path`,
  `goal`); `DroneRegistry` creates/updates one on first sighting and keeps
  its last known state even once inactive (`active_drones()` filters those
  out, but a reconnect doesn't lose history). `prune_stale()` is a safety net
  that force-deactivates a drone if its `DroneActive` heartbeat goes silent
  for `active_timeout` seconds, independent of an explicit `active: false`.
- **`path_planner.py`** — ROS-free 8-connected grid A* (`GridAStarPlanner`),
  walled-rectangle bounds, obstacles are inflated by `safety_radius`. Called
  with an empty obstacle list (the one-active-drone case) it degenerates to
  routing straight across open ground — that requirement falls out of the
  algorithm rather than needing a special case.
- **`monitor_gui.py`** — same two-class ROS-node + Tkinter pattern as
  `gui_node.py` (`MonitorBackend` / `MonitorApp`), but deliberately a
  separate file/executable (`monitor_gui`) rather than merged into
  `gui_node.py`, since the data model is different (dynamic ID-based drones
  vs. `gui_node`'s fixed config-driven count). `MonitorBackend` feeds a
  `DroneRegistry` from the shared topics and exposes `plan_for(drone_id,
  goal)`, which is a **synchronous, pure computation** (registry lookup +
  `GridAStarPlanner.plan()`) — safe to call directly from a Tk click handler,
  no action/future round-trip needed. `MonitorApp` renders the boundary
  rectangle, one marker + ID + heading tick per active drone, and a
  select-drone / click-to-set-goal flow that draws the returned path.
- **`mock_optitrack.py`** (`mock_optitrack_node`) — parallel to
  `drone_commander.MockCommander`'s `backend:=sim`: publishes simulated
  `DronePose`/`DroneActive` traffic for a configurable number of
  randomly-wandering drones, periodically toggling one's active state, so
  the whole pipeline is exercisable with no OptiTrack hardware
  (`monitor.launch.py use_mock:=true`).
- Config: `config/monitor.yaml` (map bounds, `cell_size`, `safety_radius`,
  `active_timeout`, plus `mock_optitrack_node`-only params).

## Frames

Formation math (`formations.py`, `formation_node.py`) works entirely in world
**ENU** (x=east, y=north, z=up). Only `PX4Commander.set_target_enu()` converts
to PX4's **NED** convention, right at the boundary to the autopilot. This
assumes OptiTrack is fused into PX4's EKF so both share an origin — verify
with a single drone before flying multiple.

## Config — `src/formation_interface/config/drones.yaml`

Loaded by both `formation_node` and `gui_node` (`/**` wildcard param
namespace). Lists are index-aligned: entry *i* in every list is the same
physical drone. Must be set per-deployment: `drone_namespaces` (PX4
uXRCE-DDS namespace per vehicle), `system_ids` (MAVLink system id, used for
arm/mode `VehicleCommand`s), `pose_topics` + `pose_topic_type` (your
OptiTrack bridge's per-drone topics). `auto_arm` must stay `false` until the
setup is trusted — when `true`, `formation_node` automatically switches each
drone to OFFBOARD and arms it as soon as its setpoint stream is warm
(`ready_to_arm()`, ≥10 published setpoints); otherwise arm manually via
QGroundControl.

## Known drift

The top-level `README.md` describes formations `line, column, v, circle,
grid, diamond` and a "roadmap" PySide6 GUI. Neither matches current code:
only `line`, `v`, `circle` are implemented (see `FORMATIONS` in
`formations.py`), and the GUI (`gui_node.py`, Tkinter) already exists and is
wired up via `gui.launch.py`. `src/formation_interface/README.md` is the
accurate, up-to-date description of the same project — prefer it over the
top-level one until the top-level file is reconciled.
