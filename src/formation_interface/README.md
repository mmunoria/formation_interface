# Drone Formation Control (ROS 2 + PX4 + OptiTrack)

Two packages that let an operator pick a formation on the host PC and have five
PX4 drones fly it, using OptiTrack for position feedback.

```
HOST PC                                         DRONES (PX4)
┌──────────────────────────┐                    ┌────────────────────────────┐
│ interface_node (terminal)│  GoToFormation     │ formation_node             │
│   menu → action goal ────┼──► action ────────►│  ActionServer              │
│   ◄── feedback / result  │                    │  • compute per-drone target│
│                          │   /<ns>/fmu/in/... │  • assign (nearest-first)  │
│ OptiTrack (Motive)       │  ◄─ setpoints ─────┼──• stream PX4 offboard ───►│ autopilot
│   → mocap bridge         │                    │  • watch poses → feedback  │
│   → /.../droneN/pose ────┼──► poses ─────────►│                            │
└──────────────────────────┘                    └────────────────────────────┘
```

## Packages

| Package                | Build type   | Contents                                            |
|------------------------|--------------|-----------------------------------------------------|
| `formation_interfaces` | `ament_cmake`| `GoToFormation.action` (custom action definition)   |
| `formation_interface`  | `ament_python`| `interface_node`, `formation_node`, formation math, PX4 backend |

Custom actions **must** live in an `ament_cmake` package — that's why the
interface definition is split out from the Python nodes.

## Formations

`line`, `column`, `v`, `circle`, `grid`, `diamond` — all defined in
[`formations.py`](formation_interface/formations.py) as pure geometry (no ROS),
so they're unit-tested and easy to extend. Add a generator function and register
it in `_GENERATORS`; it shows up in the menu automatically.

## Build

```bash
cd ~/.../formation_interface        # colcon workspace root (has src/)
colcon build --packages-select formation_interfaces   # interfaces first
colcon build --packages-select formation_interface
source install/setup.bash
```

`px4_msgs` must be built/sourced in the same workspace (clone it into `src/`).

## Run

**Real hardware:**
```bash
# terminal 1 — formation node (edit config/drones.yaml first!)
ros2 launch formation_interface formation.launch.py

# terminal 2 — operator menu
ros2 run formation_interface interface_node
```

**Dry run, no PX4 / no OptiTrack** (mock drones simulate + publish their poses):
```bash
ros2 launch formation_interface formation.launch.py backend:=sim
ros2 run formation_interface interface_node
```

## Configure — `config/drones.yaml`

Index-aligned lists (entry *i* = same drone). **You must set:**

- `drone_namespaces` — PX4 uXRCE-DDS namespaces (`px4_1` … `px4_5`).
- `pose_topics` — your OptiTrack bridge's per-drone pose topics.
- `pose_topic_type` — `PoseStamped` or `Odometry`.
- `auto_arm` — **keep `false`** until you trust the setup; arm manually in
  QGroundControl. When `true`, the node switches each drone to OFFBOARD and arms
  it automatically once setpoints are streaming.

## Frames

Formation math is in world **ENU** (x=east, y=north, z=up). The PX4 backend
converts to **NED** for `TrajectorySetpoint`. This assumes OptiTrack is fused
into PX4's EKF so both share an origin — verify with a single drone first.

## Assumptions to verify for your setup

- `px4_msgs` message field names match your PX4 version (`TrajectorySetpoint`,
  `OffboardControlMode`, `VehicleCommand`).
- `system_ids` match your vehicles' MAVLink system IDs.
- The uXRCE-DDS topic namespacing (`/px4_1/fmu/in/...`) matches how your agent
  is launched.

## Roadmap: GUI

`interface_node` keeps all ROS logic behind `send_formation()`. A PySide6/Qt GUI
(formation buttons, per-drone status lights, a live top-down map from the pose
topics) can wrap the same call without touching the formation node.
