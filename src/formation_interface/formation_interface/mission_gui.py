"""Graphical mission control panel for the mission manager subsystem (host PC).

Same two-thread model as ``gui_node.py``/``monitor_gui.py``: rclpy spins in a
daemon thread, Tk owns the main thread. ROS callbacks never touch Tk
directly - they drop events on a ``queue.Queue`` drained by a 10 Hz
``after()`` poll; the live telemetry table is read directly (atomic
assignment under the GIL, same rationale as ``gui_node``'s live pose table).

Independent of ``gui_node.py``/``formation_node`` - this talks to
``mission_node`` over ``ExecuteFlight`` (action, one concurrent goal per
``drone_name``), ``TerminateFlight`` (service), and ``DroneTelemetry``
(topic), not ``GoToFormation``.

Panels, top to bottom: Mission (name/description, create with an
overwrite-confirm guard, pick an existing mission), Assignment (one row per
drone profile, a flight-profile dropdown filtered to that drone's
capabilities, per-row Start/Interrupt/Terminate), All-drones controls
(Start/Interrupt/Terminate ALL + "land first"), Features (toggles saved into
the mission), and a live Telemetry table (mode, armed, battery, tracking
error, sensor handshake, link, mission phase).
"""

import queue
import threading
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from formation_interfaces.action import ExecuteFlight
from formation_interfaces.msg import DroneTelemetry
from formation_interfaces.srv import TerminateFlight

from formation_interface import mission_store
from formation_interface.drone_profiles import drone_supports, list_drone_profiles
from formation_interface.flight_profiles import list_flight_profiles

NAV_STATE_LABELS = {
    255: "unknown", 0: "MANUAL", 4: "AUTO_LOITER", 5: "AUTO_RTL",
    14: "OFFBOARD", 18: "AUTO_LAND",
}
ARMING_STATE_LABELS = {0: "unknown", 1: "disarmed", 2: "ARMED"}

OK_COLOR = "#2a2"
BAD_COLOR = "#c22"
MUTED_COLOR = "#888"


class MissionBackend(Node):
    """ROS side of the mission GUI: profiles, live telemetry, action/service
    clients. Mission-file operations (create/load/save/list) are fast local
    disk I/O, called straight through with no queue - same "safe to call
    directly from a Tk handler" precedent as monitor_gui.MonitorBackend.plan_for."""

    def __init__(self, events: queue.Queue):
        super().__init__("mission_gui")
        self.declare_parameter("missions_root", "missions")
        self.declare_parameter("drone_profiles_dir", "config/drone_profiles")
        self.declare_parameter("flight_profiles_dir", "flight_profiles")

        self.events = events
        g = self.get_parameter
        root_path = Path(str(g("missions_root").value))
        self.missions_root = root_path if root_path.is_absolute() else Path.cwd() / root_path
        self.drone_profiles_dir = Path(str(g("drone_profiles_dir").value))
        self.flight_profiles_dir = Path(str(g("flight_profiles_dir").value))

        self.drone_profiles = list_drone_profiles(self.drone_profiles_dir)
        self.flight_profiles = list_flight_profiles(self.flight_profiles_dir)
        self.telemetry = {}   # drone_name -> DroneTelemetry, read directly by Tk

        self.create_subscription(
            DroneTelemetry, "/mission/drone_telemetry", self._on_telemetry, 10)

        self._client = ActionClient(self, ExecuteFlight, "execute_flight")
        self._terminate_client = self.create_client(TerminateFlight, "terminate_flight")
        self._goal_handles = {}   # drone_name -> ClientGoalHandle

    def _on_telemetry(self, msg):
        self.telemetry[msg.drone_name] = msg

    def server_ready(self):
        return self._client.server_is_ready()

    def reload_profiles(self):
        self.drone_profiles = list_drone_profiles(self.drone_profiles_dir)
        self.flight_profiles = list_flight_profiles(self.flight_profiles_dir)

    # ------------------------------ mission file ops (fast/synchronous) --------
    def create_mission(self, name, description, assignment, features, force=False):
        return mission_store.create_mission(
            self.missions_root, name, description=description, author="",
            assignment=assignment, features=features, force=force)

    def load_mission(self, name):
        return mission_store.load_mission(self.missions_root, name)

    def save_mission(self, spec):
        mission_store.save_mission(self.missions_root, spec)

    def list_missions(self):
        return mission_store.list_missions(self.missions_root)

    # ------------------------------ start / interrupt / terminate --------------
    def start_one(self, mission_name, drone_name, flight_profile=""):
        if drone_name in self._goal_handles:
            self.events.put(("log", f"'{drone_name}' already has an active goal"))
            return
        if not self._client.server_is_ready():
            self.events.put(("result", drone_name, False, "mission_node not available"))
            return
        goal = ExecuteFlight.Goal()
        goal.mission_name = mission_name
        goal.drone_name = drone_name
        goal.flight_profile = flight_profile
        self.events.put(("log", f"starting '{drone_name}' ..."))
        fut = self._client.send_goal_async(
            goal, feedback_callback=lambda msg, n=drone_name: self._on_fb(n, msg))
        fut.add_done_callback(lambda f, n=drone_name: self._on_resp(n, f))

    def start_all(self, mission_name):
        spec = self.load_mission(mission_name)
        for drone_name in spec.assignment:
            self.start_one(mission_name, drone_name)

    def interrupt_one(self, drone_name):
        handle = self._goal_handles.get(drone_name)
        if handle is None:
            self.events.put(("log", f"'{drone_name}' has no active goal to interrupt"))
            return
        handle.cancel_goal_async()
        self.events.put(("log", f"interrupt (gentle) requested for '{drone_name}'"))

    def interrupt_all(self):
        for drone_name in list(self._goal_handles):
            self.interrupt_one(drone_name)

    def terminate(self, mission_name, drone_name, land_first):
        if not self._terminate_client.service_is_ready():
            self.events.put(("log", "terminate_flight service not available"))
            return
        req = TerminateFlight.Request()
        req.mission_name = mission_name
        req.drone_name = drone_name
        req.land_first = land_first
        fut = self._terminate_client.call_async(req)
        fut.add_done_callback(self._on_terminate_resp)
        self.events.put(("log", f"terminate requested ({drone_name or 'ALL'})"))

    def _on_terminate_resp(self, future):
        res = future.result()
        self.events.put(
            ("terminate_result", res.success, res.message, list(res.log_paths)))

    # ------------------------------ action callbacks ---------------------------
    def _on_resp(self, drone_name, future):
        handle = future.result()
        if not handle.accepted:
            self.events.put(
                ("result", drone_name, False, "goal rejected (already active?)"))
            return
        self._goal_handles[drone_name] = handle
        handle.get_result_async().add_done_callback(
            lambda f, n=drone_name: self._on_result(n, f))

    def _on_fb(self, drone_name, msg):
        f = msg.feedback
        self.events.put(("feedback", drone_name, f.phase, f.tracking_error))

    def _on_result(self, drone_name, future):
        res = future.result().result
        self._goal_handles.pop(drone_name, None)
        self.events.put(("result", drone_name, res.success, res.message))


class MissionApp(tk.Tk):
    def __init__(self, node: MissionBackend, events: queue.Queue):
        super().__init__()
        self.node = node
        self.events = events
        self.title("Drone Mission Control")
        self.current_mission = None
        self._row_widgets = {}

        self._build_mission_panel()
        self._build_assignment_panel()
        self._build_control_panel()
        self._build_features_panel()
        self._build_telemetry_panel()
        self._refresh_assignment_rows()
        self._refresh_mission_list()

        self.after(100, self._poll)

    # ------------------------------ layout -------------------------------------
    def _build_mission_panel(self):
        panel = ttk.LabelFrame(self, text="Mission", padding=8)
        panel.pack(fill="x", padx=8, pady=4)

        row = ttk.Frame(panel)
        row.pack(fill="x")
        ttk.Label(row, text="name").pack(side="left")
        self.mission_name_var = tk.StringVar()
        ttk.Entry(row, textvariable=self.mission_name_var, width=20).pack(
            side="left", padx=4)
        ttk.Label(row, text="description").pack(side="left", padx=(8, 0))
        self.mission_desc_var = tk.StringVar()
        ttk.Entry(row, textvariable=self.mission_desc_var, width=30).pack(
            side="left", padx=4)
        ttk.Button(row, text="Create Mission",
                   command=self._on_create_mission).pack(side="left", padx=4)

        row2 = ttk.Frame(panel)
        row2.pack(fill="x", pady=(4, 0))
        ttk.Label(row2, text="active mission").pack(side="left")
        self.mission_box = ttk.Combobox(row2, state="readonly", width=20)
        self.mission_box.pack(side="left", padx=4)
        self.mission_box.bind("<<ComboboxSelected>>", lambda e: self._on_select_mission())
        ttk.Button(row2, text="reload profiles",
                   command=self._on_reload_profiles).pack(side="left", padx=4)

        self.mission_status = ttk.Label(
            panel, text="no mission selected", foreground=MUTED_COLOR)
        self.mission_status.pack(anchor="w", pady=(4, 0))

    def _build_assignment_panel(self):
        panel = ttk.LabelFrame(
            self, text="Assignment (drone -> flight profile)", padding=8)
        panel.pack(fill="x", padx=8, pady=4)
        self.assignment_frame = ttk.Frame(panel)
        self.assignment_frame.pack(fill="x")
        self.assignment_vars = {}   # drone_name -> StringVar
        ttk.Button(panel, text="Save assignment",
                   command=self._on_save_assignment).pack(anchor="e", pady=(4, 0))

    def _refresh_assignment_rows(self):
        for w in self.assignment_frame.winfo_children():
            w.destroy()
        self.assignment_vars = {}
        self._row_widgets = {}
        for row_i, (name, profile) in enumerate(sorted(self.node.drone_profiles.items())):
            ttk.Label(self.assignment_frame, text=name, width=10).grid(
                row=row_i, column=0, sticky="w")
            ttk.Label(self.assignment_frame, text=", ".join(profile.capabilities) or "-",
                      foreground=MUTED_COLOR, width=20).grid(
                row=row_i, column=1, sticky="w")

            options = [
                fp.name for fp in self.node.flight_profiles.values()
                if drone_supports(profile, fp.required_capabilities)]
            var = tk.StringVar(value=options[0] if options else "")
            self.assignment_vars[name] = var
            ttk.Combobox(self.assignment_frame, state="readonly", width=16,
                         textvariable=var, values=options).grid(
                row=row_i, column=2, sticky="w", padx=4)

            row_btns = ttk.Frame(self.assignment_frame)
            row_btns.grid(row=row_i, column=3, sticky="w")
            start_btn = ttk.Button(row_btns, text="Start", width=6,
                                    command=lambda n=name: self._on_start_one(n))
            start_btn.pack(side="left", padx=1)
            interrupt_btn = ttk.Button(row_btns, text="Interrupt", width=8,
                                        command=lambda n=name: self.node.interrupt_one(n))
            interrupt_btn.pack(side="left", padx=1)
            terminate_btn = ttk.Button(row_btns, text="Terminate", width=9,
                                        command=lambda n=name: self._on_terminate_one(n))
            terminate_btn.pack(side="left", padx=1)
            self._row_widgets[name] = (start_btn, interrupt_btn, terminate_btn)

    def _build_control_panel(self):
        panel = ttk.LabelFrame(self, text="All drones", padding=8)
        panel.pack(fill="x", padx=8, pady=4)
        row = ttk.Frame(panel)
        row.pack(fill="x")
        ttk.Button(row, text="Start ALL", command=self._on_start_all).pack(
            side="left", padx=2)
        ttk.Button(row, text="INTERRUPT ALL (gentle)",
                   command=self.node.interrupt_all).pack(side="left", padx=2)
        self.land_first_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(row, text="land first", variable=self.land_first_var).pack(
            side="left", padx=(12, 2))
        ttk.Button(row, text="Terminate & Retrieve Logs (ALL)",
                   command=self._on_terminate_all).pack(side="left", padx=2)

        self.control_status = ttk.Label(panel, text="", wraplength=680)
        self.control_status.pack(anchor="w", pady=(4, 0))
        self.server_lbl = ttk.Label(panel, text="mission_node: ...", foreground=MUTED_COLOR)
        self.server_lbl.pack(anchor="w")

    def _build_features_panel(self):
        panel = ttk.LabelFrame(self, text="Features", padding=8)
        panel.pack(fill="x", padx=8, pady=4)
        self.feature_vars = {}
        for key, default in mission_store.DEFAULT_FEATURES.items():
            var = tk.BooleanVar(value=default)
            self.feature_vars[key] = var
            ttk.Checkbutton(panel, text=key, variable=var,
                            command=self._on_toggle_feature).pack(side="left", padx=6)

    def _build_telemetry_panel(self):
        panel = ttk.LabelFrame(self, text="Live telemetry", padding=8)
        panel.pack(fill="both", expand=True, padx=8, pady=4)
        cols = ("drone", "phase", "mode", "armed", "battery", "track_err",
                "sensors", "link", "mission")
        headers = {
            "drone": "Drone", "phase": "Phase", "mode": "Mode", "armed": "Armed",
            "battery": "Battery", "track_err": "Track err (m)", "sensors": "Sensors",
            "link": "Link", "mission": "Mission",
        }
        self.tree = ttk.Treeview(panel, columns=cols, show="headings", height=8)
        for c in cols:
            self.tree.heading(c, text=headers[c])
            self.tree.column(c, width=90, anchor="center")
        self.tree.pack(fill="both", expand=True)

    # ------------------------------ mission actions -----------------------------
    def _on_create_mission(self):
        name = self.mission_name_var.get().strip()
        if not name:
            messagebox.showwarning("Mission name required", "Enter a mission name first.")
            return
        desc = self.mission_desc_var.get().strip()
        try:
            self.node.create_mission(
                name, desc, assignment={}, features=dict(mission_store.DEFAULT_FEATURES))
        except mission_store.MissionExistsError:
            if not messagebox.askyesno(
                    "Mission exists",
                    f"Mission '{name}' already exists. Overwrite its mission.yaml? "
                    f"(README.md and past runs are kept.)"):
                return
            self.node.create_mission(
                name, desc, assignment={}, features=dict(mission_store.DEFAULT_FEATURES),
                force=True)
        self._refresh_mission_list()
        self.mission_box.set(name)
        self._on_select_mission()

    def _refresh_mission_list(self):
        names = self.node.list_missions()
        self.mission_box.config(values=names)

    def _on_select_mission(self):
        name = self.mission_box.get()
        if not name:
            return
        spec = self.node.load_mission(name)
        self.current_mission = spec
        for drone_name, var in self.assignment_vars.items():
            if drone_name in spec.assignment:
                var.set(spec.assignment[drone_name])
        for key, var in self.feature_vars.items():
            var.set(spec.features.get(key, mission_store.DEFAULT_FEATURES.get(key, False)))
        self.mission_status.config(
            text=f"'{spec.name}': {spec.description or '(no description)'}")

    def _on_save_assignment(self):
        name = self.mission_box.get()
        if not name:
            messagebox.showwarning("No mission selected", "Create or select a mission first.")
            return
        spec = self.node.load_mission(name)
        spec.assignment = {n: v.get() for n, v in self.assignment_vars.items() if v.get()}
        spec.features = {k: v.get() for k, v in self.feature_vars.items()}
        self.node.save_mission(spec)
        self.mission_status.config(text=f"saved assignment for '{name}'")

    def _on_reload_profiles(self):
        self.node.reload_profiles()
        self._refresh_assignment_rows()

    def _on_toggle_feature(self):
        if self.mission_box.get():
            self._on_save_assignment()

    # ------------------------------ start / interrupt / terminate --------------
    def _on_start_one(self, drone_name):
        name = self.mission_box.get()
        if not name:
            messagebox.showwarning("No mission selected", "Create or select a mission first.")
            return
        var = self.assignment_vars.get(drone_name)
        flight_profile = var.get() if var else ""
        self.node.start_one(name, drone_name, flight_profile=flight_profile)

    def _on_start_all(self):
        name = self.mission_box.get()
        if not name:
            messagebox.showwarning("No mission selected", "Create or select a mission first.")
            return
        self._on_save_assignment()
        self.node.start_all(name)

    def _on_terminate_one(self, drone_name):
        self.node.terminate(self.mission_box.get(), drone_name, self.land_first_var.get())

    def _on_terminate_all(self):
        self.node.terminate(self.mission_box.get(), "", self.land_first_var.get())

    # ------------------------------ event pump ---------------------------------
    def _update_row_states(self):
        for name, (start_btn, interrupt_btn, _terminate_btn) in self._row_widgets.items():
            active = name in self.node._goal_handles
            start_btn.config(state="disabled" if active else "normal")
            interrupt_btn.config(state="normal" if active else "disabled")

    def _refresh_telemetry_table(self):
        self.tree.delete(*self.tree.get_children())
        for name in sorted(self.node.drone_profiles):
            msg = self.node.telemetry.get(name)
            if msg is None:
                self.tree.insert(
                    "", "end", values=(name, "-", "-", "-", "-", "-", "-", "-", "-"))
                continue
            mode = NAV_STATE_LABELS.get(msg.nav_state, str(msg.nav_state))
            armed = ARMING_STATE_LABELS.get(msg.arming_state, str(msg.arming_state))
            battery = "n/a" if msg.battery_remaining < 0 else f"{msg.battery_remaining * 100:.0f}%"
            track_err = "n/a" if msg.tracking_error < 0 else f"{msg.tracking_error:.2f}"
            sensors = "OK" if msg.pre_flight_checks_pass else "FAIL"
            link = "OK" if msg.link_ok and not msg.gcs_connection_lost else "LOST"
            self.tree.insert("", "end", values=(
                name, msg.mission_phase, mode, armed, battery, track_err,
                sensors, link, msg.mission_name or "-"))

    def _poll(self):
        self.server_lbl.config(
            text="mission_node: connected" if self.node.server_ready()
            else "mission_node: NOT FOUND",
            foreground=OK_COLOR if self.node.server_ready() else BAD_COLOR)
        self._refresh_telemetry_table()
        self._update_row_states()
        try:
            while True:
                ev = self.events.get_nowait()
                kind = ev[0]
                if kind == "feedback":
                    _, drone_name, phase, track_err = ev
                    self.control_status.config(
                        text=f"'{drone_name}': {phase} (track err {track_err:.2f} m)")
                elif kind == "result":
                    _, drone_name, ok, msg = ev
                    self.control_status.config(
                        text=f"'{drone_name}': {'OK' if ok else 'FAIL'} - {msg}")
                elif kind == "terminate_result":
                    _, ok, msg, log_paths = ev
                    text = f"terminate: {'OK' if ok else 'FAIL'} - {msg}"
                    if log_paths:
                        text += f" (logs: {', '.join(log_paths)})"
                    self.control_status.config(text=text)
                elif kind == "log":
                    self.control_status.config(text=ev[1])
        except queue.Empty:
            pass
        self.after(100, self._poll)


def main(args=None):
    rclpy.init(args=args)
    events = queue.Queue()
    node = MissionBackend(events)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = MissionApp(node, events)
    try:
        app.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
