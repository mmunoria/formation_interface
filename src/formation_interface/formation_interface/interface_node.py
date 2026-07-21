"""Interface node - runs on the host PC.

A menu-driven terminal front-end that sends ``GoToFormation`` goals to the
formation node and prints live feedback.

Three built-in formations (line, v, circle) are always offered, and the
operator can *create any formation* by entering one offset per drone; those
custom formations can be saved as named presets, which persist to
``~/.formation_presets.json`` and reappear in the menu on the next run.

rclpy spins in a background thread while the blocking menu runs on the main
thread, so the two never fight.  This same node is what a future Qt/GUI
front-end will wrap - the GUI just calls ``send_formation`` instead of the
text menu.
"""

import json
import math
import threading
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Point

from formation_interfaces.action import GoToFormation

from formation_interface.formations import CUSTOM, FORMATIONS

MENU = list(FORMATIONS)
PRESET_FILE = Path.home() / ".formation_presets.json"


class InterfaceNode(Node):
    def __init__(self):
        super().__init__("interface_node")
        self._client = ActionClient(self, GoToFormation, "go_to_formation")
        self._done = threading.Event()
        self._goal_handle = None

    # ------------------------------ public API ---------------------------------
    def send_formation(self, formation, spacing, altitude, yaw=0.0, offsets=None,
                        center=(0.0, 0.0), drone_ids=None):
        """Send one formation goal and block until it finishes (result printed).

        Args:
            formation:  a built-in name, or anything when ``offsets`` is given
                        (the goal is then sent as a custom formation).
            offsets:    optional list of per-drone (x, y[, z]) offsets from the
                        formation centre - this is how arbitrary formations are
                        commanded.
            drone_ids:  optional 1-indexed subset of drones to command (matches
                        drones.yaml order). Omitted/empty = every configured
                        drone (the usual whole-swarm goal).
        """
        if not self._client.wait_for_server(timeout_sec=5.0):
            print("  !! formation_node action server not available. Is it running?")
            return

        goal = GoToFormation.Goal()
        goal.spacing = float(spacing)
        goal.altitude = float(altitude)
        goal.center = Point(x=float(center[0]), y=float(center[1]), z=0.0)
        goal.yaw = float(yaw)
        if drone_ids:
            goal.drone_ids = [int(d) for d in drone_ids]

        if offsets is not None:
            goal.formation = CUSTOM
            for off in offsets:
                z = float(off[2]) if len(off) > 2 else 0.0
                goal.custom_offsets.append(
                    Point(x=float(off[0]), y=float(off[1]), z=z))
            label = f"custom '{formation}'" if formation != CUSTOM else "custom"
        else:
            goal.formation = formation
            label = f"'{formation}'"

        self._done.clear()
        print(f"  -> sending {label} (altitude={altitude} m)")
        future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_response)
        self._done.wait()

    # ------------------------------ action callbacks ---------------------------
    def _on_response(self, future):
        handle = future.result()
        if not handle.accepted:
            print("  !! goal rejected by formation_node")
            self._done.set()
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, msg):
        f = msg.feedback
        err = "n/a" if f.max_error < 0 else f"{f.max_error:.2f} m"
        print(f"     [{f.drones_in_position}/{f.drones_total}] "
              f"{f.progress * 100:5.1f}%   worst error: {err}")

    def _on_result(self, future):
        res = future.result().result
        tag = "OK  " if res.success else "FAIL"
        print(f"  == {tag}: {res.message}")
        self._done.set()


# ------------------------------ preset storage ---------------------------------
def load_presets():
    try:
        with open(PRESET_FILE, "r", encoding="utf-8") as fh:
            data = json.load(fh)
        return {str(k): [tuple(map(float, o)) for o in v] for k, v in data.items()}
    except (OSError, ValueError):
        return {}


def save_presets(presets):
    try:
        with open(PRESET_FILE, "w", encoding="utf-8") as fh:
            json.dump({k: [list(o) for o in v] for k, v in presets.items()},
                      fh, indent=2)
    except OSError as exc:
        print(f"  (could not save presets: {exc})")


# ------------------------------ terminal menu ----------------------------------
def _ask_float(prompt, default):
    raw = input(f"{prompt} [{default}]: ").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        print("  (not a number, using default)")
        return default


def _parse_offset(raw):
    parts = [p for p in raw.replace(",", " ").split() if p]
    if len(parts) not in (2, 3):
        raise ValueError(raw)
    x, y = float(parts[0]), float(parts[1])
    z = float(parts[2]) if len(parts) == 3 else 0.0
    return (x, y, z)


def create_custom(presets):
    """Interactively define an arbitrary formation, one offset per drone.

    Returns (name, offsets) or None if aborted.
    """
    print("\n--- Create a formation ---")
    print("Enter one offset per drone, relative to the formation centre, in metres.")
    print("Format: 'x y' or 'x y z' (z is added to the base altitude).")
    print("Example V for 3 drones:  '0 0', '-1 1', '-1 -1'.  Blank line aborts.")

    n = int(_ask_float("number of drones", 5))
    offsets = []
    for i in range(n):
        while True:
            raw = input(f"  drone {i + 1} offset: ").strip()
            if not raw:
                print("  (aborted)")
                return None
            try:
                offsets.append(_parse_offset(raw))
                break
            except ValueError:
                print("    (couldn't parse - use 'x y' or 'x y z')")

    name = input("save as preset name (blank = use once): ").strip().lower()
    if name:
        if name in MENU or name == CUSTOM:
            print(f"  ('{name}' is reserved, not saving)")
            name = CUSTOM
        else:
            presets[name] = offsets
            save_presets(presets)
            print(f"  (saved - '{name}' will appear in the menu)")
    else:
        name = CUSTOM
    return name, offsets


def create_waypoint():
    """Interactively define a single-drone waypoint goal.

    Unlike ``create_custom`` (one offset per *every* drone), this flies just
    one drone - useful when the rest of the swarm isn't hooked up yet, or you
    just want to reposition one drone without disturbing the others.

    Returns ``(drone_id, x, y, z, yaw_rad)`` or ``None`` if aborted.
    """
    print("\n--- Fly a single drone ---")
    raw = input("  drone id: ").strip()
    if not raw:
        print("  (aborted)")
        return None
    try:
        drone_id = int(raw)
    except ValueError:
        print("  (not a number)")
        return None
    raw = input("  target 'x y z' (metres, z is altitude): ").strip()
    if not raw:
        print("  (aborted)")
        return None
    try:
        x, y, z = _parse_offset(raw)
    except ValueError:
        print("    (couldn't parse - use 'x y' or 'x y z')")
        return None
    yaw_deg = _ask_float("  yaw (deg)", 0.0)
    return drone_id, x, y, z, math.radians(yaw_deg)


def run_menu(node):
    presets = load_presets()
    while rclpy.ok():
        preset_names = sorted(presets)
        print("\n=== Drone Formation Control ===")
        for i, name in enumerate(MENU, 1):
            print(f"  {i}) {name}")
        for j, name in enumerate(preset_names, len(MENU) + 1):
            print(f"  {j}) {name} (saved, {len(presets[name])} drones)")
        print("  c) create a new formation")
        print("  w) fly a single drone to a waypoint")
        print("  0) quit")

        choice = input("select formation: ").strip().lower()
        if choice in ("0", "q", "quit", "exit"):
            break

        if choice in ("c", "create", "custom"):
            created = create_custom(presets)
            if created is None:
                continue
            name, offsets = created
            altitude = _ask_float("  altitude (m)", 1.5)
            node.send_formation(name, 0.0, altitude, offsets=offsets)
            continue

        if choice in ("w", "waypoint"):
            created = create_waypoint()
            if created is None:
                continue
            drone_id, x, y, z, yaw = created
            node.send_formation(CUSTOM, 0.0, z, yaw=yaw, offsets=[(0.0, 0.0, 0.0)],
                                center=(x, y), drone_ids=[drone_id])
            continue

        if not choice.isdigit():
            print("  (invalid choice)")
            continue
        idx = int(choice)

        if 1 <= idx <= len(MENU):
            formation = MENU[idx - 1]
            spacing = _ask_float("  spacing (m)", 1.5)
            altitude = _ask_float("  altitude (m)", 1.5)
            node.send_formation(formation, spacing, altitude)
        elif len(MENU) < idx <= len(MENU) + len(preset_names):
            name = preset_names[idx - len(MENU) - 1]
            altitude = _ask_float("  altitude (m)", 1.5)
            node.send_formation(name, 0.0, altitude, offsets=presets[name])
        else:
            print("  (invalid choice)")


def main(args=None):
    rclpy.init(args=args)
    node = InterfaceNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        run_menu(node)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
