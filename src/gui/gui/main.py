import math
import os
import queue
import signal
import subprocess
import threading
import time
import tkinter as tk
from tkinter import scrolledtext, ttk

import rclpy
from geometry_msgs.msg import Twist, TwistStamped

# Each entry: (button label, ros2 launch argv). Kept as plain argv lists (not
# shell strings) so no shell quoting/injection concerns launching them.
LAUNCHES = {
    "1 Robot": [
        ("Gazebo", ["ros2", "launch", "main_bot", "gz.launch.py"]),
        ("RViz", ["ros2", "launch", "main_bot", "rz.launch.py"]),
        ("SLAM", ["ros2", "launch", "main_bot", "slam.launch.py"]),
        ("Nav2", ["ros2", "launch", "main_bot", "nav2.launch.py"]),
    ],
    "2 Robot (bay dan)": [
        ("Gazebo", ["ros2", "launch", "main_bot", "multi_robot.launch.py"]),
        ("Gazebo (headless)", ["ros2", "launch", "main_bot", "multi_robot.launch.py", "headless:=true"]),
        ("Nav2", ["ros2", "launch", "main_bot", "multi_robot_nav2.launch.py"]),
        # Needs Gazebo + Nav2 (above) already active; assigns each idle robot
        # a dock->shelf->home putaway loop, forever.
        ("Fleet Dispatcher", ["ros2", "run", "fleet_dispatcher", "fleet_dispatcher"]),
        ("RViz robot1", ["ros2", "launch", "main_bot", "multi_robot_rviz.launch.py", "namespace:=robot1"]),
        ("RViz robot2", ["ros2", "launch", "main_bot", "multi_robot_rviz.launch.py", "namespace:=robot2"]),
    ],
}

# Publishing straight to each robot's final /cmd_vel (the gz-sim DiffDrive
# plugin's input topic, or ros2_control's diff_drive_controller for the
# single-robot stack) rather than through Nav2's controller_server, so the
# joystick works with or without Nav2 running. It will fight Nav2 for control
# if a nav goal is active at the same time - that's an accepted tradeoff of a
# manual override, not something to arbitrate away here.
#
# Message type differs per target: single-robot's diff_drive_controller (see
# config/controllers.yaml + config/nav2.yaml's enable_stamped_cmd_vel: true)
# only accepts TwistStamped on /cmd_vel - a plain Twist publisher on that same
# topic name never matches it (ROS2 topics are type-checked) and silently does
# nothing. Multi-robot's gz-sim DiffDrive plugin bridge is plain Twist instead
# (enable_stamped_cmd_vel: false throughout nav2_robot{1,2}.yaml).
# name -> (topic, stamped)
TELEOP_TARGETS = {
    "1 Robot": ("/cmd_vel", True),
    "Robot 1": ("/robot1/cmd_vel", False),
    "Robot 2": ("/robot2/cmd_vel", False),
}
# Capped well below FollowPath.vx_max/wz_max in config/nav2_robot{1,2}.yaml -
# manual joystick driving wants finer control, not the fastest the robot can go.
JOY_MAX_LINEAR = 0.4
JOY_MAX_ANGULAR = 1.0
JOY_PUBLISH_HZ = 10

# ---------------------------------------------------------------------------
# Palette - dark, high-contrast, a little playful (this is a robot dashboard,
# not enterprise software).
# ---------------------------------------------------------------------------
BG = "#14151f"
PANEL = "#1d1f2e"
PANEL_LIGHT = "#272a3d"
TEXT = "#eceefb"
MUTED = "#8b8fa8"
ACCENT = "#7c5cff"
ACCENT_ACTIVE = "#9478ff"
SUCCESS = "#33d17a"
DANGER = "#ff5c72"
DANGER_ACTIVE = "#ff7b8e"
WARNING = "#ffb648"
WARNING_ACTIVE = "#ffc670"
GRID_LINE = "#3a3d55"
FONT = "Ubuntu"
FONT_MONO = "DejaVu Sans Mono"


def build_style(root):
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure(".", background=BG, foreground=TEXT, font=(FONT, 10))
    style.configure("TFrame", background=BG)
    style.configure("Panel.TFrame", background=PANEL)
    style.configure("TLabel", background=BG, foreground=TEXT)
    style.configure("Panel.TLabel", background=PANEL, foreground=TEXT)
    style.configure("Header.TLabel", background=BG, foreground=TEXT, font=(FONT, 18, "bold"))
    style.configure("SubHeader.TLabel", background=BG, foreground=MUTED, font=(FONT, 10))
    style.configure("Muted.Panel.TLabel", background=PANEL, foreground=MUTED, font=(FONT, 9))
    style.configure("Readout.Panel.TLabel", background=PANEL, foreground=ACCENT, font=(FONT_MONO, 11, "bold"))

    style.configure("TNotebook", background=BG, borderwidth=0, tabmargins=(0, 8, 0, 0))
    style.configure(
        "TNotebook.Tab", background=PANEL, foreground=MUTED, padding=(18, 10), font=(FONT, 10, "bold"), borderwidth=0
    )
    style.map(
        "TNotebook.Tab",
        background=[("selected", ACCENT)],
        foreground=[("selected", "#ffffff")],
    )

    style.configure(
        "Launch.TButton", background=PANEL_LIGHT, foreground=TEXT, borderwidth=0, padding=(12, 10), font=(FONT, 10)
    )
    style.map("Launch.TButton", background=[("active", ACCENT), ("pressed", ACCENT_ACTIVE)])

    style.configure(
        "Running.TButton", background=SUCCESS, foreground="#06210f", borderwidth=0, padding=(12, 10), font=(FONT, 10, "bold")
    )
    style.map("Running.TButton", background=[("active", "#4fe092")])

    style.configure(
        "Warning.TButton", background=WARNING, foreground="#2b1900", borderwidth=0, padding=(12, 10), font=(FONT, 10, "bold")
    )
    style.map("Warning.TButton", background=[("active", WARNING_ACTIVE)])

    style.configure(
        "Danger.TButton", background=DANGER, foreground="#2b0008", borderwidth=0, padding=(12, 10), font=(FONT, 11, "bold")
    )
    style.map("Danger.TButton", background=[("active", DANGER_ACTIVE)])

    style.configure("TRadiobutton", background=PANEL, foreground=TEXT, font=(FONT, 9))
    style.map("TRadiobutton", background=[("active", PANEL)], foreground=[("active", ACCENT)])

    style.configure("TLabelframe", background=PANEL, bordercolor=GRID_LINE, darkcolor=PANEL, lightcolor=PANEL)
    style.configure("TLabelframe.Label", background=PANEL, foreground=ACCENT, font=(FONT, 11, "bold"))
    return style


class LaunchManager:
    """Starts/stops ros2 launch subprocesses and streams their output.

    Each subprocess runs in its own process group (preexec_fn=os.setsid) so
    stopping it can signal the whole group, not just the "ros2 launch"
    wrapper PID - ros2 launch fans out into many child processes (nodes,
    bridges), and killing only the parent leaves the rest running as orphans.
    Because of that same setsid, Ctrl+C in the terminal that started this GUI
    can NOT reach those child process groups - stop_all()/the Exit button are
    the only reliable way to bring everything down.
    """

    def __init__(self, log_queue):
        self.log_queue = log_queue
        self._processes = {}
        self._lock = threading.Lock()

    def is_running(self, name):
        with self._lock:
            proc = self._processes.get(name)
        return proc is not None and proc.poll() is None

    def start(self, name, cmd):
        if self.is_running(name):
            return
        self.log_queue.put((name, f"$ {' '.join(cmd)}\n"))
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            preexec_fn=os.setsid,
        )
        with self._lock:
            self._processes[name] = proc
        threading.Thread(target=self._pump_output, args=(name, proc), daemon=True).start()

    def _pump_output(self, name, proc):
        for line in proc.stdout:
            self.log_queue.put((name, line))
        self.log_queue.put((name, "--- process exited ---\n"))

    def stop(self, name):
        with self._lock:
            proc = self._processes.get(name)
        if proc is None or proc.poll() is not None:
            return
        pgid = os.getpgid(proc.pid)
        try:
            os.killpg(pgid, signal.SIGINT)
            proc.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    def stop_all(self, timeout=10):
        """Signal every running group in parallel, then wait once, not N times.

        The previous version called stop() (SIGINT, wait up to 10s, SIGKILL)
        sequentially per process - with 4-5 stacks running that was up to a
        minute of a frozen-looking window, which is exactly what pushed
        towards "just Ctrl+C the terminal" (which, per the setsid note above,
        doesn't even work). Sending SIGINT to every group up front and
        sharing one deadline makes shutdown take as long as the *slowest*
        process instead of the sum of all of them.
        """
        with self._lock:
            live = [(name, proc) for name, proc in self._processes.items() if proc.poll() is None]
        if not live:
            return
        pgids = []
        for name, proc in live:
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGINT)
                pgids.append((proc, pgid))
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + timeout
        for proc, pgid in pgids:
            try:
                proc.wait(timeout=max(0.0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                pass
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass


class Joystick(tk.Canvas):
    """Circular drag pad: any of the 360 degrees around center maps to a
    (linear, angular) pair, released always snaps back to (0, 0) so a stuck
    or crashed GUI can't leave the robot driving blind (dead-man behavior).
    """

    def __init__(self, parent, on_change, radius=76, **kwargs):
        size = radius * 2 + 28
        super().__init__(parent, width=size, height=size, bg=PANEL, highlightthickness=0, **kwargs)
        self.radius = radius
        self.handle_radius = 16
        self.center = (size / 2, size / 2)
        self.on_change = on_change

        cx, cy = self.center
        self.create_oval(
            cx - radius, cy - radius, cx + radius, cy + radius, outline=GRID_LINE, width=2, fill=PANEL_LIGHT
        )
        self.create_line(cx - radius, cy, cx + radius, cy, fill=GRID_LINE)
        self.create_line(cx, cy - radius, cx, cy + radius, fill=GRID_LINE)
        self.create_text(cx, cy - radius - 10, text="tien", fill=MUTED, font=(FONT, 8))
        self.create_text(cx, cy + radius + 10, text="lui", fill=MUTED, font=(FONT, 8))
        self.handle = self.create_oval(
            cx - self.handle_radius,
            cy - self.handle_radius,
            cx + self.handle_radius,
            cy + self.handle_radius,
            fill=ACCENT,
            outline="",
        )

        self.bind("<Button-1>", self._drag)
        self.bind("<B1-Motion>", self._drag)
        self.bind("<ButtonRelease-1>", self._release)

    def _drag(self, event):
        cx, cy = self.center
        dx, dy = event.x - cx, event.y - cy
        dist = math.hypot(dx, dy)
        if dist > self.radius:
            dx, dy = dx / dist * self.radius, dy / dist * self.radius
        self._place_handle(dx, dy)
        # normalize to [-1, 1]; canvas y grows downward, so up (forward) is -dy
        self.on_change(dx / self.radius, -dy / self.radius)

    def _release(self, _event):
        self._place_handle(0, 0)
        self.on_change(0.0, 0.0)

    def _place_handle(self, dx, dy):
        cx, cy = self.center
        r = self.handle_radius
        self.coords(self.handle, cx + dx - r, cy + dy - r, cx + dx + r, cy + dy + r)


class App:
    def __init__(self, root, ros_node):
        self.root = root
        self.ros_node = ros_node
        self.root.title("Main Bot Control Panel")
        self.root.configure(background=BG)
        build_style(root)

        self.log_queue = queue.Queue()
        self.manager = LaunchManager(self.log_queue)
        self.status_buttons = {}

        self.teleop_pubs = {
            name: (ros_node.create_publisher(TwistStamped if stamped else Twist, topic, 10), stamped)
            for name, (topic, stamped) in TELEOP_TARGETS.items()
        }
        self.teleop_target = tk.StringVar(value="1 Robot")
        self._joy_linear = 0.0
        self._joy_angular = 0.0
        self._closing = False

        self._build_layout()
        self.root.protocol("WM_DELETE_WINDOW", self._exit_app)
        self.root.after(100, self._poll_log)
        self.root.after(300, self._refresh_statuses)
        self.root.after(int(1000 / JOY_PUBLISH_HZ), self._publish_teleop)

    # -- layout ------------------------------------------------------------

    def _build_layout(self):
        outer = ttk.Frame(self.root, padding=16)
        outer.pack(fill="both", expand=True)

        left = ttk.Frame(outer)
        left.pack(side=tk.LEFT, fill="y", padx=(0, 16))

        ttk.Label(left, text="Main Bot", style="Header.TLabel").pack(anchor="w")
        ttk.Label(left, text="Control panel", style="SubHeader.TLabel").pack(anchor="w", pady=(0, 12))

        notebook = ttk.Notebook(left)
        notebook.pack(fill="x")
        for group_name, items in LAUNCHES.items():
            tab = ttk.Frame(notebook, style="Panel.TFrame", padding=12)
            notebook.add(tab, text=group_name)
            for label, cmd in items:
                self._make_launch_row(tab, f"{group_name}/{label}", label, cmd)

        self._build_teleop_panel(left)

        action_bar = ttk.Frame(left)
        action_bar.pack(fill="x", pady=(16, 0))
        ttk.Button(action_bar, text="Dung tat ca", style="Warning.TButton", command=self.manager.stop_all).pack(
            fill="x", pady=(0, 8)
        )
        ttk.Button(action_bar, text="Thoat", style="Danger.TButton", command=self._exit_app).pack(fill="x")

        right = ttk.Frame(outer)
        right.pack(side=tk.LEFT, fill="both", expand=True)
        ttk.Label(right, text="Log", style="SubHeader.TLabel").pack(anchor="w", pady=(0, 4))
        self.log = scrolledtext.ScrolledText(
            right,
            width=70,
            height=26,
            state="disabled",
            background=PANEL,
            foreground=TEXT,
            insertbackground=TEXT,
            borderwidth=0,
            font=(FONT_MONO, 9),
        )
        self.log.pack(fill="both", expand=True)

    def _make_launch_row(self, parent, name, label, cmd):
        row = ttk.Frame(parent, style="Panel.TFrame")
        row.pack(fill="x", pady=4)
        btn = ttk.Button(row, text=label, style="Launch.TButton", command=lambda: self._toggle(name, cmd))
        btn.pack(fill="x")
        self.status_buttons[name] = btn

    def _build_teleop_panel(self, parent):
        box = ttk.LabelFrame(parent, text="Dieu khien thu cong", padding=12)
        box.pack(fill="x", pady=(16, 0))

        target_row = ttk.Frame(box, style="Panel.TFrame")
        target_row.pack(fill="x", pady=(0, 10))
        for target in TELEOP_TARGETS:
            ttk.Radiobutton(
                target_row, text=target, value=target, variable=self.teleop_target, style="TRadiobutton"
            ).pack(side=tk.LEFT, padx=(0, 10))

        joy_row = ttk.Frame(box, style="Panel.TFrame")
        joy_row.pack()
        Joystick(joy_row, on_change=self._on_joy_change).pack()

        self.readout = ttk.Label(box, text="v=0.00 m/s   w=0.00 rad/s", style="Readout.Panel.TLabel")
        self.readout.pack(pady=(10, 0))

    # -- teleop --------------------------------------------------------

    def _on_joy_change(self, nx, ny):
        self._joy_linear = ny * JOY_MAX_LINEAR
        self._joy_angular = -nx * JOY_MAX_ANGULAR
        self.readout.configure(text=f"v={self._joy_linear:+.2f} m/s   w={self._joy_angular:+.2f} rad/s")

    def _publish_on(self, target_name, linear, angular):
        pub, stamped = self.teleop_pubs[target_name]
        if stamped:
            msg = TwistStamped()
            msg.header.stamp = self.ros_node.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = linear
            msg.twist.angular.z = angular
        else:
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
        pub.publish(msg)

    def _publish_teleop(self):
        if self._closing:
            return
        self._publish_on(self.teleop_target.get(), self._joy_linear, self._joy_angular)
        self.root.after(int(1000 / JOY_PUBLISH_HZ), self._publish_teleop)

    # -- launch buttons ------------------------------------------------

    def _toggle(self, name, cmd):
        if self.manager.is_running(name):
            self.manager.stop(name)
        else:
            self.manager.start(name, cmd)

    def _refresh_statuses(self):
        for name, btn in self.status_buttons.items():
            running = self.manager.is_running(name)
            btn.configure(style="Running.TButton" if running else "Launch.TButton")
        if not self._closing:
            self.root.after(300, self._refresh_statuses)

    def _poll_log(self):
        try:
            while True:
                name, line = self.log_queue.get_nowait()
                self.log.configure(state="normal")
                self.log.insert("end", f"[{name}] {line}")
                self.log.see("end")
                self.log.configure(state="disabled")
        except queue.Empty:
            pass
        if not self._closing:
            self.root.after(100, self._poll_log)

    # -- shutdown --------------------------------------------------------

    def _exit_app(self):
        """Stop every launched process group and close - the only button
        that reliably shuts everything down (see LaunchManager's docstring
        for why a terminal Ctrl+C on this GUI does not)."""
        self._closing = True
        self.manager.stop_all()
        for name in self.teleop_pubs:
            self._publish_on(name, 0.0, 0.0)
        self.root.destroy()


def main():
    rclpy.init()
    node = rclpy.create_node("control_panel_teleop")
    root = tk.Tk()
    App(root, node)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
