"""Live multi-drone monitor + path planner (host PC).

Tkinter + rclpy, same threading model as :mod:`gui_node`: rclpy spins on a
daemon thread, Tk owns the main thread. Unlike ``gui_node`` there is no
action client here and no cross-thread futures, so the two sides only share
plain data (the :class:`~formation_interface.active_tracker.DroneRegistry`) -
subscription callbacks write to it on the spin thread, the GUI reads it every
poll tick on the Tk thread. That's safe under the GIL for the same reason
``gui_node``'s live pose table is (see its module docstring); nothing here is
flight-safety-critical.

Unlike the formation-flying GUI, the drone count here isn't fixed by
``config/drones.yaml`` - drones are discovered dynamically from
``DronePose``/``DroneActive`` messages, each carrying its own ID, and only
currently-active ones are shown.

Features:
  * live top-down map of every *active* drone (ID label + heading tick),
    inside the configured rectangular flight area
  * a list of active drones that grows/shrinks as they come online or go
    inactive (explicit ``active: false``, or a stale heartbeat)
  * select a drone, click the map to set its goal - the drone's path is
    planned around every other active drone and drawn as a dashed line

This is visualization + planning only: no path is sent to any drone.
"""

import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node

from formation_interfaces.msg import DroneActive, DronePose

from formation_interface.active_tracker import DroneRegistry
from formation_interface.path_planner import GridAStarPlanner

DRONE_COLORS = ["#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4",
                "#42d4f4", "#f032e6", "#9a6324"]


class MonitorBackend(Node):
    """ROS side: OptiTrack pose/active intake, the drone registry, planning."""

    def __init__(self):
        super().__init__("formation_monitor")
        self.declare_parameter("map_xmin", -5.0)
        self.declare_parameter("map_xmax", 5.0)
        self.declare_parameter("map_ymin", -5.0)
        self.declare_parameter("map_ymax", 5.0)
        self.declare_parameter("cell_size", 0.25)
        self.declare_parameter("safety_radius", 0.5)
        self.declare_parameter("active_timeout", 2.0)

        g = self.get_parameter
        bounds = (float(g("map_xmin").value), float(g("map_xmax").value),
                  float(g("map_ymin").value), float(g("map_ymax").value))
        self.planner = GridAStarPlanner(
            bounds,
            cell_size=float(g("cell_size").value),
            safety_radius=float(g("safety_radius").value))
        self.registry = DroneRegistry(stale_timeout=float(g("active_timeout").value))

        self.create_subscription(
            DronePose, "/optitrack/drone_pose", self._on_pose, 20)
        self.create_subscription(
            DroneActive, "/optitrack/drone_active", self._on_active, 20)
        self.create_timer(0.2, self._prune)

        self.get_logger().info(
            f"formation_monitor ready: bounds={bounds}, "
            f"cell_size={self.planner.cell_size} m, "
            f"safety_radius={self.planner.safety_radius} m")

    # ------------------------------ optitrack intake ---------------------------
    def _on_pose(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.registry.update_pose(msg.drone_id, p.x, p.y, (o.x, o.y, o.z, o.w))

    def _on_active(self, msg):
        self.registry.update_active(msg.drone_id, msg.active, self._now())

    def _prune(self):
        self.registry.prune_stale(self._now())

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------ planning ------------------------------------
    def plan_for(self, drone_id, goal):
        """Plan a path for one active drone to ``goal``, around every other
        active drone. Returns ``(success, message)``; stores the result (or
        clears it, on failure) on that drone's registry entry."""
        drone = self.registry.get(drone_id)
        if drone is None or not drone.active:
            return False, f"drone {drone_id} is not active"

        obstacles = [d.position for d in self.registry.active_drones()
                     if d.id != drone_id]
        try:
            path = self.planner.plan(drone.position, goal, obstacles)
        except ValueError as exc:
            return False, str(exc)

        if path is None:
            self.registry.set_goal(drone_id, None)
            self.registry.set_path(drone_id, None)
            return False, "no path found"

        self.registry.set_goal(drone_id, goal)
        self.registry.set_path(drone_id, path)
        return True, f"planned {len(path)} waypoints"


class MonitorApp(tk.Tk):
    CANVAS = 640          # px
    MARGIN = 24           # px, so the boundary isn't flush with the window edge

    def __init__(self, node: MonitorBackend):
        super().__init__()
        self.node = node
        self.title("Drone Swarm Monitor")
        self.resizable(False, False)

        self.selected_drone = None
        self._listbox_ids = []

        self._build_left_panel()
        self._build_map()
        self._draw_boundary()
        self.after(100, self._poll)

    # ------------------------------ layout -------------------------------------
    def _build_left_panel(self):
        panel = ttk.Frame(self, padding=10)
        panel.grid(row=0, column=0, sticky="ns")

        ttk.Label(panel, text="Active drones", font=("", 11, "bold")).pack(anchor="w")
        self.listbox = tk.Listbox(panel, height=12, width=18, exportselection=False)
        self.listbox.pack(fill="x", pady=(2, 8))
        self.listbox.bind("<<ListboxSelect>>", self._on_select)

        ttk.Separator(panel).pack(fill="x", pady=8)
        ttk.Label(panel, wraplength=200, foreground="#555",
                  text="Select a drone above, then click the map to plan a "
                       "path for it around every other active drone.").pack(anchor="w")

        self.status = ttk.Label(panel, text="waiting for drones...", wraplength=200)
        self.status.pack(anchor="w", pady=(10, 0))

    def _build_map(self):
        self.canvas = tk.Canvas(self, width=self.CANVAS, height=self.CANVAS,
                                bg="#fafafa", highlightthickness=1,
                                highlightbackground="#ccc")
        self.canvas.grid(row=0, column=1, padx=(0, 10), pady=10)
        self.canvas.bind("<Button-1>", self._on_map_click)

    # ------------------------------ map helpers --------------------------------
    def _w2p(self, x, y):
        b = self.node.planner
        s = (self.CANVAS - 2 * self.MARGIN) / max(b.xmax - b.xmin, b.ymax - b.ymin)
        px = self.MARGIN + (x - b.xmin) * s
        py = self.CANVAS - self.MARGIN - (y - b.ymin) * s     # +y is "up" on screen
        return px, py

    def _p2w(self, px, py):
        b = self.node.planner
        s = (self.CANVAS - 2 * self.MARGIN) / max(b.xmax - b.xmin, b.ymax - b.ymin)
        x = b.xmin + (px - self.MARGIN) / s
        y = b.ymin + (self.CANVAS - self.MARGIN - py) / s
        return x, y

    def _draw_boundary(self):
        b = self.node.planner
        left, bottom = self._w2p(b.xmin, b.ymin)
        right, top = self._w2p(b.xmax, b.ymax)
        self.canvas.create_rectangle(left, top, right, bottom, outline="#333", width=2)

        step = 1.0
        x = math.ceil(b.xmin / step) * step
        while x <= b.xmax + 1e-9:
            px, _ = self._w2p(x, b.ymin)
            self.canvas.create_line(px, top, px, bottom, fill="#eee")
            x += step
        y = math.ceil(b.ymin / step) * step
        while y <= b.ymax + 1e-9:
            _, py = self._w2p(b.xmin, y)
            self.canvas.create_line(left, py, right, py, fill="#eee")
            y += step

    def _redraw_dynamic(self):
        self.canvas.delete("dyn")
        for drone in self.node.registry.active_drones():
            x, y = drone.position
            px, py = self._w2p(x, y)
            color = DRONE_COLORS[drone.id % len(DRONE_COLORS)]

            if len(drone.planned_path) >= 2:
                pts = []
                for wx, wy in drone.planned_path:
                    wpx, wpy = self._w2p(wx, wy)
                    pts.extend((wpx, wpy))
                self.canvas.create_line(*pts, fill=color, dash=(4, 2), tags="dyn")

            if drone.goal is not None:
                gx, gy = self._w2p(*drone.goal)
                self.canvas.create_line(gx - 6, gy - 6, gx + 6, gy + 6,
                                        fill=color, width=2, tags="dyn")
                self.canvas.create_line(gx - 6, gy + 6, gx + 6, gy - 6,
                                        fill=color, width=2, tags="dyn")

            hx, hy = self._w2p(x + 0.3 * math.cos(drone.yaw),
                               y + 0.3 * math.sin(drone.yaw))
            self.canvas.create_line(px, py, hx, hy, fill=color, width=2, tags="dyn")
            self.canvas.create_oval(px - 8, py - 8, px + 8, py + 8,
                                    fill=color, outline="white", tags="dyn")
            self.canvas.create_text(px, py, text=str(drone.id), fill="white",
                                    font=("", 8, "bold"), tags="dyn")

    def _refresh_listbox(self):
        ids = sorted(d.id for d in self.node.registry.active_drones())
        if ids == self._listbox_ids:
            return
        selected_id = self.selected_drone
        self.listbox.delete(0, tk.END)
        for drone_id in ids:
            self.listbox.insert(tk.END, f"drone {drone_id}")
        self._listbox_ids = ids
        if selected_id in ids:
            self.listbox.selection_set(ids.index(selected_id))
        else:
            self.selected_drone = None

    # ------------------------------ interaction ---------------------------------
    def _on_select(self, _event):
        sel = self.listbox.curselection()
        if not sel:
            return
        self.selected_drone = self._listbox_ids[sel[0]]
        self.status.config(
            text=f"drone {self.selected_drone} selected - click the map to set its goal")

    def _on_map_click(self, event):
        if self.selected_drone is None:
            self.status.config(text="select a drone from the list first")
            return
        goal = self._p2w(event.x, event.y)
        ok, msg = self.node.plan_for(self.selected_drone, goal)
        tag = "OK" if ok else "FAIL"
        self.status.config(text=f"drone {self.selected_drone}: {tag} - {msg}")

    # ------------------------------ event pump -----------------------------------
    def _poll(self):
        self._refresh_listbox()
        self._redraw_dynamic()
        self.after(100, self._poll)


def main(args=None):
    rclpy.init(args=args)
    node = MonitorBackend()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = MonitorApp(node)
    try:
        app.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
