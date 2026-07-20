"""Graphical control panel for the drone formation system (host PC).

Tkinter + rclpy. Tkinter ships with Python (``sudo apt install python3-tk`` on
a bare Ubuntu), so there is nothing extra to install on the lab machine.

Threading model:
  * rclpy spins in a daemon thread; the Tk main loop owns the main thread.
  * ROS callbacks never touch Tk directly - they drop events on a
    ``queue.Queue`` that the GUI drains from a 10 Hz ``after()`` poll. The
    live pose table is read directly (atomic tuple swaps under the GIL).

Features:
  * one-click built-in formations (line / v / circle) + spacing / altitude / yaw
  * live top-down map of the OptiTrack poses, ghost markers for the targets
  * click-to-design mode: place one target per drone on the map, fly it,
    optionally save it as a named preset (same ``~/.formation_presets.json``
    the terminal interface uses)
  * live action feedback: progress bar, in-position count, worst error, cancel
"""

import math
import queue
import threading
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry

from formation_interfaces.action import GoToFormation

from formation_interface.formations import (
    CUSTOM,
    FORMATIONS,
    compute_targets,
    transform_offsets,
)
from formation_interface.interface_node import load_presets, save_presets

DRONE_COLORS = ["#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4",
                "#42d4f4", "#f032e6", "#9a6324"]


class GuiBackend(Node):
    """ROS side of the GUI: pose intake + GoToFormation action client."""

    def __init__(self, events: queue.Queue):
        super().__init__("formation_gui")
        self.declare_parameter("pose_topics", [
            "/vrpn_mocap/drone1/pose", "/vrpn_mocap/drone2/pose",
            "/vrpn_mocap/drone3/pose", "/vrpn_mocap/drone4/pose",
            "/vrpn_mocap/drone5/pose"])
        self.declare_parameter("pose_topic_type", "PoseStamped")

        self.events = events
        topics = list(self.get_parameter("pose_topics").value)
        ptype = self.get_parameter("pose_topic_type").value
        self.n = len(topics)
        self.poses = [None] * self.n            # latest (x, y, z) per drone

        for i, topic in enumerate(topics):
            if ptype == "Odometry":
                self.create_subscription(
                    Odometry, topic,
                    lambda m, i=i: self._store(i, m.pose.pose.position), 10)
            else:
                self.create_subscription(
                    PoseStamped, topic,
                    lambda m, i=i: self._store(i, m.pose.position), 10)

        self._client = ActionClient(self, GoToFormation, "go_to_formation")
        self._goal_handle = None

    def _store(self, idx, p):
        self.poses[idx] = (p.x, p.y, p.z)

    def server_ready(self):
        return self._client.server_is_ready()

    # ------------------------------ sending ------------------------------------
    def send(self, formation, spacing, altitude, yaw,
             center=(0.0, 0.0), offsets=None):
        if not self._client.server_is_ready():
            self.events.put(("result", False, "formation_node not available"))
            return

        goal = GoToFormation.Goal()
        goal.spacing = float(spacing)
        goal.altitude = float(altitude)
        goal.center = Point(x=float(center[0]), y=float(center[1]), z=0.0)
        goal.yaw = float(yaw)
        if offsets is not None:
            goal.formation = CUSTOM
            for off in offsets:
                z = float(off[2]) if len(off) > 2 else 0.0
                goal.custom_offsets.append(
                    Point(x=float(off[0]), y=float(off[1]), z=z))
        else:
            goal.formation = formation

        self.events.put(("log", f"sending '{formation}' ..."))
        fut = self._client.send_goal_async(goal, feedback_callback=self._on_fb)
        fut.add_done_callback(self._on_resp)

    def cancel(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self.events.put(("log", "cancel requested"))

    # ------------------------------ action callbacks ---------------------------
    def _on_resp(self, future):
        handle = future.result()
        if not handle.accepted:
            self.events.put(("result", False, "goal rejected (already running?)"))
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_fb(self, msg):
        f = msg.feedback
        self.events.put(
            ("feedback", f.drones_in_position, f.drones_total,
             f.progress, f.max_error))

    def _on_result(self, future):
        res = future.result().result
        self._goal_handle = None
        self.events.put(("result", res.success, res.message))


class App(tk.Tk):
    CANVAS = 520          # px
    MAP_RANGE = 5.0       # metres shown from centre to edge

    def __init__(self, node: GuiBackend, events: queue.Queue):
        super().__init__()
        self.node = node
        self.events = events
        self.title("Drone Formation Control")
        self.resizable(False, False)

        self.presets = load_presets()
        self.targets = []            # ghost markers currently displayed
        self.design_points = []      # world (x, y) clicks in design mode
        self.design_mode = False
        self.busy = False

        self._build_left_panel()
        self._build_map()
        self._draw_grid()
        self.after(100, self._poll)

    # ------------------------------ layout -------------------------------------
    def _build_left_panel(self):
        panel = ttk.Frame(self, padding=10)
        panel.grid(row=0, column=0, sticky="ns")

        ttk.Label(panel, text="Formations", font=("", 11, "bold")).pack(anchor="w")
        row = ttk.Frame(panel)
        row.pack(fill="x", pady=(2, 8))
        self.formation_btns = []
        for name in FORMATIONS:
            b = ttk.Button(row, text=name, width=7,
                           command=lambda n=name: self._fly_builtin(n))
            b.pack(side="left", padx=2)
            self.formation_btns.append(b)

        # parameters
        grid = ttk.Frame(panel)
        grid.pack(fill="x", pady=4)
        self.spacing = self._spin(grid, 0, "spacing (m)", 1.5, 0.3, 5.0, 0.1)
        self.altitude = self._spin(grid, 1, "altitude (m)", 1.5, 0.5, 3.0, 0.1)
        self.yaw_deg = self._spin(grid, 2, "yaw (deg)", 0.0, -180.0, 180.0, 15.0)

        ttk.Separator(panel).pack(fill="x", pady=8)

        # presets
        ttk.Label(panel, text="Saved formations", font=("", 11, "bold")).pack(anchor="w")
        prow = ttk.Frame(panel)
        prow.pack(fill="x", pady=2)
        self.preset_box = ttk.Combobox(
            prow, state="readonly", width=16, values=sorted(self.presets))
        self.preset_box.pack(side="left", padx=(0, 4))
        self.preset_fly_btn = ttk.Button(prow, text="fly", width=5,
                                         command=self._fly_preset)
        self.preset_fly_btn.pack(side="left")

        ttk.Separator(panel).pack(fill="x", pady=8)

        # design mode
        ttk.Label(panel, text="Design a formation",
                  font=("", 11, "bold")).pack(anchor="w")
        ttk.Label(panel, wraplength=210, foreground="#555",
                  text="Toggle design mode, then click one target per drone "
                       "on the map.").pack(anchor="w")
        drow = ttk.Frame(panel)
        drow.pack(fill="x", pady=4)
        self.design_btn = ttk.Button(drow, text="design: off", width=10,
                                     command=self._toggle_design)
        self.design_btn.pack(side="left", padx=2)
        ttk.Button(drow, text="clear", width=6,
                   command=self._clear_design).pack(side="left", padx=2)
        drow2 = ttk.Frame(panel)
        drow2.pack(fill="x", pady=2)
        self.fly_design_btn = ttk.Button(
            drow2, text="fly designed", width=12, state="disabled",
            command=self._fly_designed)
        self.fly_design_btn.pack(side="left", padx=2)
        self.save_design_btn = ttk.Button(
            drow2, text="save preset", width=11, state="disabled",
            command=self._save_designed)
        self.save_design_btn.pack(side="left", padx=2)

        ttk.Separator(panel).pack(fill="x", pady=8)

        # status
        self.cancel_btn = ttk.Button(panel, text="CANCEL", state="disabled",
                                     command=self.node.cancel)
        self.cancel_btn.pack(fill="x", pady=(0, 6))
        self.progress = ttk.Progressbar(panel, maximum=100.0)
        self.progress.pack(fill="x")
        self.status = ttk.Label(panel, text="idle", wraplength=210)
        self.status.pack(anchor="w", pady=2)
        self.server_lbl = ttk.Label(panel, text="formation_node: ...",
                                    foreground="#888")
        self.server_lbl.pack(anchor="w", pady=(6, 0))

    def _spin(self, parent, r, label, val, lo, hi, step):
        ttk.Label(parent, text=label).grid(row=r, column=0, sticky="w")
        var = tk.DoubleVar(value=val)
        tk.Spinbox(parent, textvariable=var, from_=lo, to=hi, increment=step,
                   width=7).grid(row=r, column=1, sticky="e", padx=4, pady=1)
        return var

    def _build_map(self):
        self.canvas = tk.Canvas(self, width=self.CANVAS, height=self.CANVAS,
                                bg="#fafafa", highlightthickness=1,
                                highlightbackground="#ccc")
        self.canvas.grid(row=0, column=1, padx=(0, 10), pady=10)
        self.canvas.bind("<Button-1>", self._on_map_click)

    # ------------------------------ map helpers --------------------------------
    def _w2p(self, x, y):
        s = self.CANVAS / (2.0 * self.MAP_RANGE)
        return self.CANVAS / 2.0 + x * s, self.CANVAS / 2.0 - y * s

    def _p2w(self, px, py):
        s = self.CANVAS / (2.0 * self.MAP_RANGE)
        return (px - self.CANVAS / 2.0) / s, (self.CANVAS / 2.0 - py) / s

    def _draw_grid(self):
        for m in range(-int(self.MAP_RANGE), int(self.MAP_RANGE) + 1):
            px, _ = self._w2p(m, 0)
            _, py = self._w2p(0, m)
            colour = "#bbb" if m == 0 else "#e8e8e8"
            self.canvas.create_line(px, 0, px, self.CANVAS, fill=colour)
            self.canvas.create_line(0, py, self.CANVAS, py, fill=colour)
            if m and m % 2 == 0:
                self.canvas.create_text(px + 2, self.CANVAS / 2 + 10,
                                        text=f"{m}", fill="#999", anchor="w")
                self.canvas.create_text(self.CANVAS / 2 + 4, py,
                                        text=f"{m}", fill="#999", anchor="w")
        self.canvas.create_text(self.CANVAS - 30, self.CANVAS / 2 - 10,
                                text="x (E)", fill="#888")
        self.canvas.create_text(self.CANVAS / 2 + 20, 12, text="y (N)",
                                fill="#888")

    def _redraw_dynamic(self):
        self.canvas.delete("dyn")
        # ghost targets
        for i, (x, y, _z) in enumerate(self.targets):
            px, py = self._w2p(x, y)
            self.canvas.create_oval(px - 9, py - 9, px + 9, py + 9,
                                    outline="#999", dash=(3, 2), tags="dyn")
        # design clicks
        for i, (x, y) in enumerate(self.design_points):
            px, py = self._w2p(x, y)
            self.canvas.create_rectangle(px - 6, py - 6, px + 6, py + 6,
                                         outline="#d40", width=2, tags="dyn")
            self.canvas.create_text(px, py - 14, text=str(i + 1),
                                    fill="#d40", tags="dyn")
        # live drones
        for i, pose in enumerate(self.node.poses):
            if pose is None:
                continue
            px, py = self._w2p(pose[0], pose[1])
            c = DRONE_COLORS[i % len(DRONE_COLORS)]
            self.canvas.create_oval(px - 7, py - 7, px + 7, py + 7,
                                    fill=c, outline="white", tags="dyn")
            self.canvas.create_text(px, py, text=str(i + 1), fill="white",
                                    font=("", 8, "bold"), tags="dyn")

    # ------------------------------ actions ------------------------------------
    def _yaw_rad(self):
        return math.radians(self.yaw_deg.get())

    def _fly_builtin(self, name):
        if self.busy:
            return
        spacing, alt, yaw = self.spacing.get(), self.altitude.get(), self._yaw_rad()
        self.targets = compute_targets(name, self.node.n, spacing,
                                       (0.0, 0.0), alt, yaw)
        self._set_busy(True, f"flying '{name}'")
        self.node.send(name, spacing, alt, yaw)

    def _fly_preset(self):
        name = self.preset_box.get()
        if self.busy or not name:
            return
        offsets = self.presets[name]
        if len(offsets) != self.node.n:
            messagebox.showwarning(
                "Preset mismatch",
                f"'{name}' has {len(offsets)} drones; system has {self.node.n}.")
            return
        alt, yaw = self.altitude.get(), self._yaw_rad()
        self.targets = transform_offsets(offsets, (0.0, 0.0), alt, yaw)
        self._set_busy(True, f"flying preset '{name}'")
        self.node.send(name, 0.0, alt, yaw, offsets=offsets)

    def _toggle_design(self):
        self.design_mode = not self.design_mode
        self.design_btn.config(
            text="design: ON" if self.design_mode else "design: off")

    def _clear_design(self):
        self.design_points = []
        self.fly_design_btn.config(state="disabled")
        self.save_design_btn.config(state="disabled")

    def _on_map_click(self, ev):
        if not self.design_mode or len(self.design_points) >= self.node.n:
            return
        self.design_points.append(self._p2w(ev.x, ev.y))
        if len(self.design_points) == self.node.n:
            self.fly_design_btn.config(state="normal")
            self.save_design_btn.config(state="normal")
            self.status.config(text=f"{self.node.n} targets placed - ready")

    def _designed_center_offsets(self):
        cx = sum(p[0] for p in self.design_points) / len(self.design_points)
        cy = sum(p[1] for p in self.design_points) / len(self.design_points)
        offsets = [(x - cx, y - cy, 0.0) for x, y in self.design_points]
        return (cx, cy), offsets

    def _fly_designed(self):
        if self.busy or len(self.design_points) != self.node.n:
            return
        center, offsets = self._designed_center_offsets()
        alt = self.altitude.get()
        self.targets = transform_offsets(offsets, center, alt)
        self._set_busy(True, "flying designed formation")
        self.node.send("designed", 0.0, alt, 0.0, center=center, offsets=offsets)

    def _save_designed(self):
        if len(self.design_points) != self.node.n:
            return
        name = simpledialog.askstring("Save preset", "Preset name:", parent=self)
        if not name:
            return
        name = name.strip().lower()
        if name in FORMATIONS or name == CUSTOM:
            messagebox.showwarning("Reserved", f"'{name}' is reserved.")
            return
        _center, offsets = self._designed_center_offsets()
        self.presets[name] = offsets
        save_presets(self.presets)
        self.preset_box.config(values=sorted(self.presets))
        self.status.config(text=f"saved preset '{name}'")

    # ------------------------------ event pump ---------------------------------
    def _set_busy(self, busy, text=None):
        self.busy = busy
        state = "disabled" if busy else "normal"
        for b in self.formation_btns:
            b.config(state=state)
        self.preset_fly_btn.config(state=state)
        self.cancel_btn.config(state="normal" if busy else "disabled")
        if text:
            self.status.config(text=text)

    def _poll(self):
        self._redraw_dynamic()
        self.server_lbl.config(
            text="formation_node: connected" if self.node.server_ready()
            else "formation_node: NOT FOUND",
            foreground="#2a2" if self.node.server_ready() else "#c22")
        try:
            while True:
                ev = self.events.get_nowait()
                kind = ev[0]
                if kind == "feedback":
                    _, in_pos, total, prog, err = ev
                    self.progress["value"] = prog * 100.0
                    err_s = "n/a" if err < 0 else f"{err:.2f} m"
                    self.status.config(
                        text=f"{in_pos}/{total} in position - worst {err_s}")
                elif kind == "result":
                    _, ok, msg = ev
                    self.progress["value"] = 100.0 if ok else 0.0
                    self.status.config(text=("OK: " if ok else "FAIL: ") + msg)
                    self._set_busy(False)
                elif kind == "log":
                    self.status.config(text=ev[1])
        except queue.Empty:
            pass
        self.after(100, self._poll)


def main(args=None):
    rclpy.init(args=args)
    events = queue.Queue()
    node = GuiBackend(events)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = App(node, events)
    try:
        app.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
