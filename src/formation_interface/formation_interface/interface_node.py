"""Interface node - runs on the host PC.

A menu-driven terminal front-end that sends ``GoToFormation`` goals to the
formation node and prints live feedback.

The old version called ``input()`` inside a timer, which blocked the ROS
executor.  Here rclpy spins in a background thread while the blocking menu runs
on the main thread, so the two never fight.  This same node is what a future
Qt/GUI front-end will wrap - the GUI just calls ``send_formation`` instead of
the text menu.
"""

import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Point

from formation_interfaces.action import GoToFormation

from formation_interface.formations import FORMATIONS

MENU = list(FORMATIONS)


class InterfaceNode(Node):
    def __init__(self):
        super().__init__("interface_node")
        self._client = ActionClient(self, GoToFormation, "go_to_formation")
        self._done = threading.Event()
        self._goal_handle = None

    # ------------------------------ public API ---------------------------------
    def send_formation(self, formation, spacing, altitude, yaw=0.0):
        """Send one formation goal and block until it finishes (result printed)."""
        if not self._client.wait_for_server(timeout_sec=5.0):
            print("  !! formation_node action server not available. Is it running?")
            return

        goal = GoToFormation.Goal()
        goal.formation = formation
        goal.spacing = float(spacing)
        goal.altitude = float(altitude)
        goal.center = Point(x=0.0, y=0.0, z=0.0)
        goal.yaw = float(yaw)

        self._done.clear()
        print(f"  -> sending '{formation}' (spacing={spacing} m, altitude={altitude} m)")
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


def _ask_float(prompt, default):
    raw = input(f"{prompt} [{default}]: ").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        print("  (not a number, using default)")
        return default


def run_menu(node):
    while rclpy.ok():
        print("\n=== Drone Formation Control ===")
        for i, name in enumerate(MENU, 1):
            print(f"  {i}) {name}")
        print("  0) quit")

        choice = input("select formation: ").strip().lower()
        if choice in ("0", "q", "quit", "exit"):
            break
        if not choice.isdigit() or not (1 <= int(choice) <= len(MENU)):
            print("  (invalid choice)")
            continue

        formation = MENU[int(choice) - 1]
        spacing = _ask_float("  spacing (m)", 1.5)
        altitude = _ask_float("  altitude (m)", 1.5)
        node.send_formation(formation, spacing, altitude)


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
