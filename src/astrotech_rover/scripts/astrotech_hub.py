#!/usr/bin/env python3
"""Astrotech operations hub -- a one-window launcher for the science payload.

A standalone tkinter GUI that boots every piece of the Astrotech stack with a
button: build the workspace, launch the rover node + Foxglove bridge (real or
all-mock), bring the cameras / ZED up on the Jetson over SSH, run diagnostics,
calibrate the Raman spectrometer, and shut everything down.

Three tabs:
  * Jetson       -- the rover (cmr@192.168.1.69) over SSH, where the hardware
                    lives (the DEFAULT tab): build, launch the node + Foxglove
                    bridge (real or one-actuator), cameras, ZED, diagnostics,
                    shutdown, and an SSH shell.
  * Base station -- this laptop: build + install the Foxglove panels, the Raman
                    calibration helper, and an optional all-mock launch for
                    offline GUI dev.
  * Raman calibration -- fit pixel->wavelength from known peaks (reuses
                    raman_calibrate.py) and print the YAML block to paste.

Authentication (any of, in priority order):
  1. type the Jetson password into the field in the panel header -- held in
     memory only, never written to disk;
  2. JETSON_SSH_PASSWORD in a gitignored .env at the repo root
     (cp .env.example .env), or exported in your shell;
  3. nothing -- plain ssh uses your SSH key (ssh-copy-id cmr@192.168.1.69) or
     prompts you in the terminal.
Modes 1-2 use sshpass; the password is staged to the child shell via a 0600
temp file that is read and deleted immediately (never on the command line).

Modeled on the autonomy team's scripts/jetson_launcher.py. Does not import or
modify anything in this repo. Requires:
    - Python 3 with tkinter (preinstalled on Ubuntu)
    - sshpass            (sudo apt install sshpass)   [only for the Jetson tab]
    - gnome-terminal / xterm / konsole / xfce4-terminal

Run with:
    python3 src/astrotech_rover/scripts/astrotech_hub.py
"""

import importlib.util
import math
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk


# The Raman calibration fitter lives next to this file (stdlib only). Load it by
# path so the interactive Raman tab reuses the exact same math as the CLI.
def _load_raman_calibrate():
    try:
        path = Path(__file__).resolve().parent / "raman_calibrate.py"
        spec = importlib.util.spec_from_file_location("raman_calibrate", str(path))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    except Exception:
        return None


RAMAN_CALIBRATE = _load_raman_calibrate()


# --- Configuration -----------------------------------------------------------

JETSON_USER = "cmr"
JETSON_HOST = "192.168.1.69"
JETSON_WORKSPACE = "~/cmr/terra2"

# This file: src/astrotech_rover/scripts/astrotech_hub.py -> parents[3] = repo root.
REPO_ROOT = Path(__file__).resolve().parents[3]
ENV_FILE = REPO_ROOT / ".env"
LOCAL_WORKSPACE = str(REPO_ROOT)  # colcon workspace root (this repo)
EXTENSION_DIR = str(REPO_ROOT / "gui" / "extensions" / "urc-astrotech-panels")

ROS_UNDERLAY = "/opt/ros/humble/setup.bash"

# Force a specific terminal emulator. None = auto-detect, or one of
# "xterm", "xfce4-terminal", "konsole", "gnome-terminal". If gnome-terminal
# throws DBus / "org.gnome.Terminal" errors, set this to "xterm".
PREFERRED_TERMINAL = None

# A command entry is a dict:
#   label        button text
#   cmd          shell command to run (after the cd / source below)
#   source       True -> `source install/setup.bash` first (run-time overlay)
#   cwd          working dir (local tab only; defaults to the workspace root)
#   interactive  True -> drop into an interactive shell afterwards instead of
#                a "press Enter to close" pause (used by the Raman helper)
#
# Local tab = the base-station laptop (runs Foxglove + dev/calibration tools).
# The real rover bring-up lives on the Jetson tab. Build commands source the ROS
# *underlay* explicitly so a cold checkout (no install/ yet) still builds.
LOCAL_GROUPS = [
    ("Foxglove panels  (install into Foxglove on this laptop)", [
        {
            "label": "Build + install Foxglove panels  (npm)",
            "cmd": "npm install && npm run build && npm run local-install",
            "source": False,
            "cwd": EXTENSION_DIR,
        },
    ]),
    ("Tools", [
        {
            "label": "Raman calibration helper  (interactive)",
            "cmd": "python3 src/astrotech_rover/scripts/raman_calibrate.py --help; "
                   "echo; echo 'Example:'; "
                   "echo '  python3 src/astrotech_rover/scripts/raman_calibrate.py "
                   "--laser-nm 785 --point 410:520.7:cm-1 --point 1980:1332:cm-1 "
                   "--point 3050:2900:cm-1'",
            "source": False,
            "interactive": True,
        },
    ]),
    ("Local dev / demo  (no rover -- needs ROS + a local build)", [
        {
            "label": "Build astrotech  (cmr_msgs + astrotech_rover)",
            "cmd": f"source {ROS_UNDERLAY} && "
                   "colcon build --symlink-install "
                   "--packages-select cmr_msgs astrotech_rover",
            "source": False,
        },
        {
            "label": "Launch ALL-MOCK  (demo, no hardware)",
            "cmd": "URC_AUGER_MOCK=1 URC_MIXING_SERVO_MOCK=1 "
                   "URC_RAMAN_MOCK=1 URC_ENV_MOCK=1 URC_ANALYSIS_MOCK=1 "
                   "ros2 launch astrotech_rover astrotech.launch.py",
            "source": True,
        },
    ]),
]

# Jetson tab = the rover (hardware: CAN dongle + cameras + ZED). All commands
# run over SSH via build_workspace_command(JETSON_WORKSPACE, ...).
JETSON_GROUPS = [
    ("Build", [
        {
            "label": "Build astrotech  (cmr_msgs + astrotech_rover)",
            "cmd": f"source {ROS_UNDERLAY} && "
                   "colcon build --symlink-install "
                   "--packages-select cmr_msgs astrotech_rover",
            "source": False,
        },
        {
            "label": "Colcon build  (full workspace)",
            "cmd": f"source {ROS_UNDERLAY} && colcon build",
            "source": False,
        },
    ]),
    ("Run the payload  (node + Foxglove bridge -> ws://192.168.1.69:8765)", [
        {
            "label": "Launch  (real hardware)",
            "cmd": "ros2 launch astrotech_rover astrotech.launch.py",
            "source": True,
        },
        {
            "label": "Launch  auger only  (mixing servo mocked)",
            "cmd": "URC_MIXING_SERVO_MOCK=1 "
                   "ros2 launch astrotech_rover astrotech.launch.py",
            "source": True,
        },
        {
            "label": "Launch  mixing servo only  (auger mocked)",
            "cmd": "URC_AUGER_MOCK=1 "
                   "ros2 launch astrotech_rover astrotech.launch.py",
            "source": True,
        },
    ]),
    ("Cameras", [
        {
            "label": "Launch cameras  (cmr_cams)",
            "cmd": "ros2 launch cmr_cams default.launch.py",
            "source": True,
        },
        {
            "label": "Activate all cameras",
            "cmd": "./src/cmr_cams/config/activate.sh",
            "source": True,
        },
    ]),
    ("ZED + bridge", [
        {
            "label": "ZED publisher  ->  /zed/image_left",
            "cmd": "ros2 run cmr_zed zed_publisher_node",
            "source": True,
        },
        {
            "label": "Foxglove bridge only  (if not using the launch above)",
            "cmd": "ros2 run foxglove_bridge foxglove_bridge",
            "source": True,
        },
    ]),
    ("Diagnostics", [
        {"label": "List topics", "cmd": "ros2 topic list", "source": True},
        {
            "label": "Raman rate  (hz)",
            "cmd": "ros2 topic hz /astrotech/raman/spectrum",
            "source": True,
        },
        {
            "label": "Auger cam rate  (hz)",
            "cmd": "ros2 topic hz /cam11/image_raw",
            "source": True,
        },
        {
            "label": "ZED rate  (hz)",
            "cmd": "ros2 topic hz /zed/image_left",
            "source": True,
        },
    ]),
    ("Shut down", [
        {
            "label": "Stop payload / bridge / ZED  (pkill)",
            "cmd": "pkill -9 -f astrotech_node; pkill -9 -f foxglove_bridge; "
                   "pkill -9 -f zed",
            "source": False,
        },
    ]),
]


# --- .env loading ------------------------------------------------------------

def load_env_file(path):
    """Tiny KEY=VALUE parser. No deps, no shell expansion. Returns a dict."""
    if not path.exists():
        return {}
    out = {}
    for raw in path.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        key = key.strip()
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
            value = value[1:-1]
        out[key] = value
    return out


def load_jetson_password():
    """Env var wins; .env file is the fallback. Returns '' if neither is set."""
    if os.environ.get("JETSON_SSH_PASSWORD"):
        return os.environ["JETSON_SSH_PASSWORD"]
    return load_env_file(ENV_FILE).get("JETSON_SSH_PASSWORD", "")


# --- Terminal handling -------------------------------------------------------

def find_terminal():
    """Pick a terminal emulator. Honors PREFERRED_TERMINAL; else xterm first."""
    if PREFERRED_TERMINAL:
        if shutil.which(PREFERRED_TERMINAL):
            return PREFERRED_TERMINAL
        print(
            f"Warning: PREFERRED_TERMINAL={PREFERRED_TERMINAL!r} not installed, "
            "falling back to auto-detect.",
            file=sys.stderr,
        )
    for binary in ("xterm", "xfce4-terminal", "konsole", "gnome-terminal"):
        if shutil.which(binary):
            return binary
    return None


def build_terminal_argv(terminal, title, local_bash):
    """Construct argv that opens `terminal` running `bash -c local_bash`."""
    if terminal == "gnome-terminal":
        return [terminal, "--title", title, "--", "bash", "-c", local_bash]
    if terminal == "konsole":
        return [terminal, "-p", f"tabtitle={title}", "-e", "bash", "-c", local_bash]
    if terminal == "xfce4-terminal":
        return [terminal, "-T", title, "-x", "bash", "-c", local_bash]
    return [terminal, "-T", title, "-e", "bash", "-c", local_bash]  # xterm


def build_workspace_command(workspace, cmd, source_workspace):
    """`cd <ws> [&& source install/setup.bash] && <cmd>` one-liner."""
    parts = [f"cd {workspace}"]
    if source_workspace:
        parts.append("source install/setup.bash")
    parts.append(cmd)
    return " && ".join(parts)


def with_pause(bash_cmd, message="[done -- press Enter to close]"):
    """Append a pause prompt so the terminal doesn't close immediately."""
    safe_msg = message.replace("'", "'\\''")
    return f"{bash_cmd}; echo; echo '{safe_msg}'; read"


def with_interactive(bash_cmd):
    """Run the command, then drop into an interactive shell (stays open)."""
    return f"{bash_cmd}; echo; exec bash"


def local_bash_for(entry):
    """Build the local bash one-liner for a command entry dict."""
    cwd = entry.get("cwd", LOCAL_WORKSPACE)
    parts = [f"cd {shlex.quote(cwd)}"]
    if entry.get("source"):
        parts.append("source install/setup.bash")
    parts.append(entry["cmd"])
    base = " && ".join(parts)
    return with_interactive(base) if entry.get("interactive") else with_pause(base)


# --- GUI ---------------------------------------------------------------------

class ScrollableTab(ttk.Frame):
    """A tab whose body scrolls vertically when its content overflows."""

    def __init__(self, parent):
        super().__init__(parent)
        canvas = tk.Canvas(self, highlightthickness=0)
        scrollbar = ttk.Scrollbar(self, orient="vertical", command=canvas.yview)
        self.body = ttk.Frame(canvas)
        self.body.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all")),
        )
        window = canvas.create_window((0, 0), window=self.body, anchor="nw")
        canvas.bind(
            "<Configure>", lambda e: canvas.itemconfigure(window, width=e.width)
        )
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        # Mouse-wheel scrolling (Linux: Button-4/5).
        canvas.bind_all("<Button-4>", lambda e: canvas.yview_scroll(-1, "units"))
        canvas.bind_all("<Button-5>", lambda e: canvas.yview_scroll(1, "units"))


class AstrotechHub(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Astrotech Hub")
        self.geometry("720x760")
        self.minsize(600, 640)

        self.terminal = find_terminal()
        self.password = load_jetson_password()  # from .env / shell (optional)
        self.pw_var = tk.StringVar()             # password typed into the panel

        self._build_header()

        notebook = ttk.Notebook(self)
        notebook.pack(fill="both", expand=True, padx=10, pady=(0, 8))

        # Jetson first so it's the DEFAULT tab -- that's the real rover bring-up.
        jetson_tab = ScrollableTab(notebook)
        local_tab = ScrollableTab(notebook)
        raman_tab = ScrollableTab(notebook)
        notebook.add(jetson_tab, text="  Jetson  ")
        notebook.add(local_tab, text="  Base station  ")
        notebook.add(raman_tab, text="  Raman calibration  ")

        self._populate_jetson(jetson_tab.body)
        self._populate(local_tab.body, LOCAL_GROUPS, self.launch_local)
        self._build_raman_tab(raman_tab.body)

        self.status = ttk.Label(self, text="Ready.", foreground="#0a0", anchor="w")
        self.status.pack(fill="x", padx=12, pady=(0, 8), side="bottom")

        if not self.terminal:
            messagebox.showerror(
                "No terminal emulator",
                "None of gnome-terminal / konsole / xfce4-terminal / xterm are "
                "installed.\nInstall one, e.g.:\n  sudo apt install xterm",
            )

    # --- header ---------------------------------------------------------
    def _build_header(self):
        header = ttk.Frame(self)
        header.pack(fill="x", padx=12, pady=(10, 6))
        ttk.Label(
            header, text="Astrotech operations hub",
            font=("TkDefaultFont", 13, "bold"),
        ).pack(anchor="w")
        ttk.Label(
            header,
            text=f"Jetson: {JETSON_USER}@{JETSON_HOST}   ·   "
                 f"workspace: {JETSON_WORKSPACE}   ·   "
                 f"terminal: {self.terminal or 'NONE FOUND'}",
            foreground="#555",
        ).pack(anchor="w")
        pw_row = ttk.Frame(header)
        pw_row.pack(anchor="w", fill="x", pady=(2, 0))
        ttk.Label(pw_row, text="Jetson SSH password:").pack(side="left")
        ttk.Entry(pw_row, textvariable=self.pw_var, show="•", width=22).pack(
            side="left", padx=6
        )
        ttk.Label(
            pw_row,
            text="(optional — held in memory only; blank = SSH key / prompt)",
            foreground="#888",
        ).pack(side="left")
        self._auth_label = ttk.Label(header, text="")
        self._auth_label.pack(anchor="w")
        self.pw_var.trace_add("write", lambda *_: self._update_auth_label())
        self._update_auth_label()
        ttk.Label(
            header,
            text="Real bring-up runs on the Jetson tab; operate from Foxglove on "
                 "this laptop (connect to ws://192.168.1.69:8765). Each button "
                 "opens a terminal and runs there — Ctrl+C or close it to stop.",
            foreground="#555", justify="left",
        ).pack(anchor="w", pady=(4, 0))

    def _effective_password(self):
        """Typed-in-panel password wins; .env / shell value is the fallback."""
        return self.pw_var.get().strip() or self.password

    def _update_auth_label(self):
        if self.pw_var.get().strip():
            text, color = "Jetson auth: password entered (this session)", "#0a0"
        elif self.password:
            text, color = "Jetson auth: password from .env", "#0a0"
        else:
            text, color = "Jetson auth: SSH key / interactive prompt", "#06c"
        self._auth_label.config(text=text, foreground=color)

    # --- tab population -------------------------------------------------
    def _populate(self, parent, groups, handler):
        for group_label, entries in groups:
            frame = ttk.LabelFrame(parent, text=group_label)
            frame.pack(fill="x", padx=10, pady=6)
            for entry in entries:
                ttk.Button(
                    frame, text=entry["label"],
                    command=lambda e=entry: handler(e),
                ).pack(fill="x", padx=8, pady=3)

    def _populate_jetson(self, parent):
        connect = ttk.LabelFrame(parent, text="Connect")
        connect.pack(fill="x", padx=10, pady=6)
        ttk.Button(
            connect,
            text="Open SSH shell  (workspace sourced, ready for ros2)",
            command=self.launch_jetson_shell,
        ).pack(fill="x", padx=8, pady=3)
        self._populate(parent, JETSON_GROUPS, self.launch_jetson)

    # --- Raman calibration tab ------------------------------------------
    def _build_raman_tab(self, parent):
        self._raman_rows = []
        self._raman_yaml = ""

        ttk.Label(
            parent, justify="left", foreground="#555",
            text=(
                "Turn the Raman x-axis into cm-1.\n"
                "1) In Foxglove, read the PIXEL index of each known peak (a lamp\n"
                "   line, or a Raman standard such as silicon at 520.7 cm-1).\n"
                "2) Enter your laser wavelength and the peaks below, then Fit.\n"
                "3) Paste the printed block into the Jetson's\n"
                "   config/astrotech_interfaces.yaml, then rebuild + relaunch."
            ),
        ).pack(anchor="w", padx=10, pady=(10, 6))

        if RAMAN_CALIBRATE is None:
            ttk.Label(
                parent, foreground="#a00",
                text="raman_calibrate.py not found next to the hub — fitting disabled.",
            ).pack(anchor="w", padx=10)
            return

        top = ttk.Frame(parent)
        top.pack(fill="x", padx=10, pady=4)
        ttk.Label(top, text="Laser wavelength (nm):").pack(side="left")
        self._raman_laser = tk.StringVar(value="785")
        ttk.Entry(top, textvariable=self._raman_laser, width=8).pack(side="left", padx=6)
        ttk.Label(top, text="    Fit order:").pack(side="left")
        self._raman_order = tk.StringVar(value="2")
        ttk.Combobox(
            top, textvariable=self._raman_order, values=["1", "2", "3"],
            width=4, state="readonly",
        ).pack(side="left", padx=6)

        peaks = ttk.LabelFrame(parent, text="Reference peaks")
        peaks.pack(fill="x", padx=10, pady=6)
        hdr = ttk.Frame(peaks)
        hdr.pack(fill="x", padx=8, pady=(6, 0))
        ttk.Label(hdr, text="pixel", width=12).pack(side="left", padx=(0, 6))
        ttk.Label(hdr, text="value", width=12).pack(side="left", padx=(0, 6))
        ttk.Label(hdr, text="unit", width=8).pack(side="left", padx=(0, 6))
        self._raman_rows_frame = ttk.Frame(peaks)
        self._raman_rows_frame.pack(fill="x", padx=8, pady=4)
        for _ in range(3):
            self._raman_add_row()
        ttk.Button(peaks, text="+ Add peak", command=self._raman_add_row).pack(
            anchor="w", padx=8, pady=(0, 6)
        )

        actions = ttk.Frame(parent)
        actions.pack(fill="x", padx=10, pady=4)
        ttk.Button(actions, text="Fit", command=self._raman_fit).pack(side="left")
        ttk.Button(actions, text="Copy YAML", command=self._raman_copy).pack(
            side="left", padx=6
        )

        self._raman_out = tk.Text(
            parent, height=11, wrap="none", state="disabled",
            font=("TkFixedFont", 10),
        )
        self._raman_out.pack(fill="both", expand=True, padx=10, pady=(4, 10))

    def _raman_add_row(self, pixel="", value="", unit="cm-1"):
        row = ttk.Frame(self._raman_rows_frame)
        row.pack(fill="x", pady=2)
        entry = {
            "frame": row,
            "pixel": tk.StringVar(value=pixel),
            "value": tk.StringVar(value=value),
            "unit": tk.StringVar(value=unit),
        }
        ttk.Entry(row, textvariable=entry["pixel"], width=12).pack(side="left", padx=(0, 6))
        ttk.Entry(row, textvariable=entry["value"], width=12).pack(side="left", padx=(0, 6))
        ttk.Combobox(
            row, textvariable=entry["unit"], values=["cm-1", "nm"],
            width=6, state="readonly",
        ).pack(side="left", padx=(0, 6))
        ttk.Button(
            row, text="✕", width=3,
            command=lambda: self._raman_remove_row(entry),
        ).pack(side="left")
        self._raman_rows.append(entry)

    def _raman_remove_row(self, entry):
        entry["frame"].destroy()
        if entry in self._raman_rows:
            self._raman_rows.remove(entry)

    def _raman_set_output(self, text):
        self._raman_out.config(state="normal")
        self._raman_out.delete("1.0", "end")
        self._raman_out.insert("1.0", text)
        self._raman_out.config(state="disabled")

    def _raman_fit(self):
        cal = RAMAN_CALIBRATE
        try:
            laser = float(self._raman_laser.get())
            if laser <= 0:
                raise ValueError
        except ValueError:
            self._raman_set_output("Laser wavelength must be a positive number (nm).")
            return
        try:
            order = int(self._raman_order.get())
        except ValueError:
            order = 2

        pts = []  # (pixel, value, unit, wavelength_nm)
        for r in self._raman_rows:
            ps = r["pixel"].get().strip()
            vs = r["value"].get().strip()
            if not ps and not vs:
                continue  # skip blank rows
            try:
                pix = float(ps)
                val = float(vs)
            except ValueError:
                self._raman_set_output(
                    f"Bad number in a peak row (pixel='{ps}', value='{vs}')."
                )
                return
            try:
                wl = (cal.shift_cm_inv_to_nm(laser, val)
                      if r["unit"].get().startswith("cm") else val)
            except SystemExit as exc:  # shift out of range for this laser
                self._raman_set_output(str(exc))
                return
            pts.append((pix, val, r["unit"].get(), wl))

        if len(pts) < order + 1:
            self._raman_set_output(
                f"Need at least {order + 1} peaks for an order-{order} fit "
                f"(got {len(pts)})."
            )
            return

        try:
            coeffs = cal.polyfit_ascending(
                [p[0] for p in pts], [p[3] for p in pts], order
            )
        except SystemExit as exc:
            self._raman_set_output(str(exc))
            return

        poly = ", ".join(f"{c:.10g}" for c in coeffs)
        self._raman_yaml = (
            "  real_calibration:\n"
            f"    laser_nm: {laser:g}\n"
            f"    pixel_to_wavelength_poly: [{poly}]"
        )
        res = [
            cal.nm_to_shift_cm_inv(laser, cal.polyval_ascending(coeffs, p[0]))
            - cal.nm_to_shift_cm_inv(laser, p[3])
            for p in pts
        ]
        rms = math.sqrt(sum(x * x for x in res) / len(res))
        lines = [
            "# paste under `raman:` in config/astrotech_interfaces.yaml",
            self._raman_yaml,
            "",
            f"# fit: order {order}, {len(pts)} points, RMS {rms:.2f} cm^-1",
        ]
        for (pix, _val, _unit, wl), r in zip(pts, res):
            lines.append(
                f"#   pixel {pix:8.1f} -> {wl:9.3f} nm "
                f"({cal.nm_to_shift_cm_inv(laser, wl):8.1f} cm^-1)  "
                f"residual {r:+6.2f} cm^-1"
            )
        self._raman_set_output("\n".join(lines))
        self.status.config(text="Raman fit done.", foreground="#0a0")

    def _raman_copy(self):
        if not self._raman_yaml:
            self.status.config(
                text="Nothing to copy — run Fit first.", foreground="#a00"
            )
            return
        self.clipboard_clear()
        self.clipboard_append(self._raman_yaml)
        self.status.config(
            text="Copied real_calibration block to clipboard.", foreground="#0a0"
        )

    # --- launching ------------------------------------------------------
    def _spawn(self, bash, title, kind):
        if not self.terminal:
            messagebox.showerror("No terminal", "No terminal emulator available.")
            return False
        argv = build_terminal_argv(self.terminal, title, bash)
        try:
            subprocess.Popen(argv, start_new_session=True)
            self.status.config(text=f"Launched {kind}: {title}", foreground="#0a0")
            return True
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Launch failed", str(exc))
            self.status.config(text=f"Failed: {exc}", foreground="#a00")
            return False

    def launch_local(self, entry):
        self._spawn(local_bash_for(entry), f"Local: {entry['label']}", "local")

    def launch_jetson(self, entry):
        remote = build_workspace_command(
            JETSON_WORKSPACE, entry["cmd"], entry.get("source", False)
        )
        self._launch_jetson_remote(remote, entry["label"])

    def launch_jetson_shell(self):
        remote = (
            f"cd {JETSON_WORKSPACE} && source install/setup.bash && exec bash"
        )
        self._launch_jetson_remote(remote, "SSH shell")

    def _launch_jetson_remote(self, remote_bash, label):
        """Open a terminal that SSHes into the Jetson and runs `remote_bash`.

        Auth (see `_effective_password`):
          * A password (typed in the panel, or from .env) -> sshpass. The secret
            is staged to a 0600 temp file that the child shell reads and deletes
            immediately, so it never lands on the command line / argv (ps-safe)
            and works across terminal emulators.
          * No password -> plain `ssh`: your SSH key (ssh-copy-id) if set up,
            else ssh prompts you in the terminal. Nothing is stored.
        """
        if not self.terminal:
            messagebox.showerror("No terminal", "No terminal emulator available.")
            return False

        ssh_target = f"{JETSON_USER}@{JETSON_HOST}"
        ssh_opts = "-t -o StrictHostKeyChecking=accept-new"
        tail = "echo; echo '[session ended -- press Enter to close]'; read"
        password = self._effective_password()

        if password:
            if not shutil.which("sshpass"):
                messagebox.showerror(
                    "sshpass not installed",
                    "A Jetson password is set, which needs sshpass.\n\n"
                    "Install it:\n  sudo apt install sshpass\n\n"
                    "Or clear the password and use an SSH key:\n"
                    "  ssh-copy-id cmr@192.168.1.69",
                )
                return False
            try:
                fd, pw_path = tempfile.mkstemp(prefix=".astrotech_ssh_")
                with os.fdopen(fd, "w") as fh:
                    fh.write(password)
                os.chmod(pw_path, 0o600)
            except OSError as exc:
                messagebox.showerror("Launch failed", f"Could not stage password: {exc}")
                return False
            q = shlex.quote(pw_path)
            ssh_inner = (
                f"sshpass -e ssh {ssh_opts} {ssh_target} {shlex.quote(remote_bash)}"
            )
            # Read the password into $SSHPASS and delete the temp file before
            # ssh runs; sshpass -e then reads it from the environment.
            bash = (
                f"export SSHPASS=\"$(cat {q})\"; rm -f {q}; "
                f"{ssh_inner}; {tail}"
            )
        else:
            # Key-based (or interactive password prompt) -- no stored secret.
            ssh_inner = f"ssh {ssh_opts} {ssh_target} {shlex.quote(remote_bash)}"
            bash = f"{ssh_inner}; {tail}"

        return self._spawn(bash, f"Jetson: {label}", "jetson")


def main():
    AstrotechHub().mainloop()


if __name__ == "__main__":
    main()
