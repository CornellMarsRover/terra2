#!/usr/bin/env python3
"""
Standalone tkinter GUI to launch the rover startup commands.

Two sections:
  * Local run -- runs commands in a terminal on this machine (no SSH).
                 Used to start basestation_known (sends RTK corrections).
                 Make sure basestation_known.py already has the current
                 lat/lon/alt and cmr_rtkgps has been rebuilt.
  * Jetson    -- runs commands on cmr@192.168.1.69 via SSH, each in its
                 own terminal window.

Authentication: the SSH password is read from a .env file at the repo root
(which is gitignored). Create it once:

    cp .env.example .env          # then edit .env and set JETSON_SSH_PASSWORD

The JETSON_SSH_PASSWORD env var, if already set in your shell, takes priority
over the .env file.

This script does not depend on any package in this repo and does not modify
any existing code. It only requires:
    - Python 3 with tkinter (preinstalled on Ubuntu)
    - sshpass            (sudo apt install sshpass)
    - gnome-terminal / xterm / konsole / xfce4-terminal

Run with:
    python3 scripts/jetson_launcher.py
"""

import os
import shlex
import shutil
import subprocess
import sys
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk


# --- Configuration -----------------------------------------------------------

JETSON_USER = "cmr"
JETSON_HOST = "192.168.1.69"
JETSON_WORKSPACE = "~/cmr/terra2"

REPO_ROOT = Path(__file__).resolve().parent.parent
ENV_FILE = REPO_ROOT / ".env"
LOCAL_WORKSPACE = str(REPO_ROOT)  # this same machine's terra2 workspace

# Force a specific terminal emulator. Set to None to auto-detect, or to one of
# "xterm", "xfce4-terminal", "konsole", "gnome-terminal".
# Tip: if gnome-terminal throws DBus / "org.gnome.Terminal" errors, set this to
# "xterm" (sudo apt install xterm) -- it has no client/server dance and just works.
PREFERRED_TERMINAL = None

# Local commands -- run on THIS machine. (button label, command, source setup.bash?)
LOCAL_COMMANDS = [
    (
        "Send corrections   ->  ros2 run cmr_rtkgps basestation_known",
        "ros2 run cmr_rtkgps basestation_known",
        True,
    ),
]

# Jetson commands -- run over SSH on JETSON_HOST.
# (button label, remote command, source install/setup.bash before running?)
JETSON_COMMANDS = [
    (
        "Colcon build (full workspace)",
        "colcon build",
        False,
    ),
    (
        "ZED   ->  ros2 run cmr_zed zed_autonomy",
        "ros2 run cmr_zed zed_autonomy",
        True,
    ),
    (
        "Localization   ->  ros2 launch autonomous_navigation localization_real.launch.py",
        "ros2 launch autonomous_navigation localization_real.launch.py",
        True,
    ),
    (
        "GPS rover   ->  ros2 run cmr_rtkgps gps_rover",
        "ros2 run cmr_rtkgps gps_rover",
        True,
    ),
    (
        "Live telemetry   ->  ros2 run autonomous_navigation live_telemetry_tool",
        "ros2 run autonomous_navigation live_telemetry_tool",
        True,
    ),
]

# One-click FULL AUTON (nav + detect) run. The button fires both halves:
#   * Jetson (SSH): full nav stack + on-Jetson telemetry GUI w/ overlay.
#   * Local (base laptop): the YOLO base-station detection node.
# RTK corrections (base) and GPS rover (Jetson) are prerequisites -- start them
# first using the buttons below, as in the normal workflow.
FULL_AUTON_JETSON_CMD = "ros2 launch autonomous_navigation full_auton.launch.py"
FULL_AUTON_LOCAL_CMD = "ros2 launch cmr_cams base_detection.launch.py"


# --- .env loading ------------------------------------------------------------

def load_env_file(path):
    """Tiny KEY=VALUE parser. No deps, no shell expansion. Returns a dict.
    Lines starting with '#' and blank lines are ignored. Quotes around the
    value are stripped if matched."""
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
    """Pick a terminal emulator. Honors PREFERRED_TERMINAL; otherwise tries
    xterm first (most reliable), then falls back to GUI-y ones."""
    if PREFERRED_TERMINAL:
        if shutil.which(PREFERRED_TERMINAL):
            return PREFERRED_TERMINAL
        print(
            f"Warning: PREFERRED_TERMINAL={PREFERRED_TERMINAL!r} not installed, "
            f"falling back to auto-detect.",
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
    """Wrap a ros2 command into a single bash one-liner: `cd <ws> [&& source
    install/setup.bash] && <cmd>`. No `exec bash` at the end -- the caller is
    responsible for adding a pause so the terminal stays open. We deliberately
    do NOT spawn a new interactive shell, so .bashrc on the remote does not
    reload (and any path-typos in .bashrc don't pollute the output)."""
    parts = [f"cd {workspace}"]
    if source_workspace:
        parts.append("source install/setup.bash")
    parts.append(cmd)
    return " && ".join(parts)


def with_pause(bash_cmd, message="[done -- press Enter to close]"):
    """Append a pause prompt so the terminal doesn't close immediately."""
    safe_msg = message.replace("'", "'\\''")
    return f"{bash_cmd}; echo; echo '{safe_msg}'; read"


# --- GUI ---------------------------------------------------------------------

class JetsonLauncher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Rover Launcher")
        self.geometry("680x680")
        self.minsize(560, 600)

        self.terminal = find_terminal()
        self.password = load_jetson_password()

        pad = {"padx": 12, "pady": 4}

        header = ttk.Frame(self)
        header.pack(fill="x", **pad)
        ttk.Label(
            header,
            text=f"Target:  {JETSON_USER}@{JETSON_HOST}",
            font=("TkDefaultFont", 12, "bold"),
        ).pack(anchor="w")
        ttk.Label(
            header,
            text=f"Workspace:  {JETSON_WORKSPACE}",
            foreground="#555",
        ).pack(anchor="w")
        ttk.Label(
            header,
            text=f"Terminal:  {self.terminal or 'NONE FOUND'}",
            foreground="#555",
        ).pack(anchor="w")
        pw_status = "loaded from .env" if self.password else "NOT SET (see .env.example)"
        ttk.Label(
            header,
            text=f"SSH password:  {pw_status}",
            foreground="#0a0" if self.password else "#a00",
        ).pack(anchor="w")

        ttk.Label(
            self,
            text=(
                "Each button opens a new terminal window, sources install/setup.bash,\n"
                "and runs the command. Close the terminal (or Ctrl+C inside it) to stop it."
            ),
            foreground="#555",
            justify="left",
        ).pack(**pad, anchor="w")

        auton_frame = ttk.LabelFrame(self, text="Full autonomy + detection  (one click)")
        auton_frame.pack(fill="x", **pad)
        ttk.Label(
            auton_frame,
            text=(
                "Boots the rover nav stack + telemetry GUI on the Jetson (SSH) AND the\n"
                "base-station YOLO detection node locally. Start RTK corrections (below)\n"
                "and GPS rover (Jetson, below) first, as in the normal workflow."
            ),
            foreground="#555",
            justify="left",
        ).pack(anchor="w", padx=8, pady=(4, 4))
        ttk.Button(
            auton_frame,
            text="FULL AUTON RUN   (nav + detect)",
            command=self.launch_full_auton,
        ).pack(fill="x", padx=8, pady=4)

        local_frame = ttk.LabelFrame(self, text=f"Local run  ({LOCAL_WORKSPACE})")
        local_frame.pack(fill="x", **pad)
        ttk.Label(
            local_frame,
            text=(
                "Make sure the lat/lon/alt in basestation_known.py is current\n"
                "and the cmr_rtkgps package has been rebuilt before clicking."
            ),
            foreground="#555",
            justify="left",
        ).pack(anchor="w", padx=8, pady=(4, 4))
        for label, cmd, src in LOCAL_COMMANDS:
            ttk.Button(
                local_frame,
                text=label,
                command=lambda c=cmd, s=src, l=label: self.launch_local(c, s, l),
            ).pack(fill="x", padx=8, pady=4)

        jetson_frame = ttk.LabelFrame(self, text=f"Jetson  ({JETSON_USER}@{JETSON_HOST})")
        jetson_frame.pack(fill="both", expand=True, **pad)

        ttk.Button(
            jetson_frame,
            text="Open SSH shell  (workspace sourced, ready for ros2 commands)",
            command=self.launch_jetson_shell,
        ).pack(fill="x", padx=8, pady=(8, 2))
        ttk.Separator(jetson_frame, orient="horizontal").pack(
            fill="x", padx=8, pady=(4, 2)
        )

        for label, cmd, src in JETSON_COMMANDS:
            ttk.Button(
                jetson_frame,
                text=label,
                command=lambda c=cmd, s=src, l=label: self.launch_jetson(c, s, l),
            ).pack(fill="x", padx=8, pady=4)

        self.status = ttk.Label(self, text="Ready.", foreground="#0a0")
        self.status.pack(fill="x", **pad, side="bottom")

        if not self.terminal:
            messagebox.showerror(
                "No terminal emulator",
                "None of gnome-terminal / konsole / xfce4-terminal / xterm "
                "are installed.\nInstall one, e.g.:\n  sudo apt install xterm",
            )

    def launch_local(self, cmd, source_workspace, label):
        """Open a local terminal and run the command on this machine."""
        if not self.terminal:
            messagebox.showerror("No terminal", "No terminal emulator available.")
            return
        bash = with_pause(
            build_workspace_command(LOCAL_WORKSPACE, cmd, source_workspace)
        )
        title = f"Local: {label}"
        argv = build_terminal_argv(self.terminal, title, bash)
        try:
            subprocess.Popen(argv, start_new_session=True)
            self.status.config(text=f"Launched local: {label}", foreground="#0a0")
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Launch failed", str(exc))
            self.status.config(text=f"Failed: {exc}", foreground="#a00")

    def _launch_jetson_remote(self, remote_bash, label):
        """Open a local terminal that SSHes into the Jetson with the password
        sourced from .env, and runs `remote_bash` on the Jetson via ssh -t.
        Returns True on success."""
        if not self.terminal:
            messagebox.showerror("No terminal", "No terminal emulator available.")
            return False
        if not self.password:
            messagebox.showerror(
                "Password not set",
                f"No JETSON_SSH_PASSWORD found.\n\n"
                f"Create {ENV_FILE} (copy from .env.example) and set:\n"
                f"    JETSON_SSH_PASSWORD=your_password\n\n"
                f"Or export JETSON_SSH_PASSWORD in your shell.",
            )
            return False

        # Source .env from inside the new bash so the password makes it into
        # gnome-terminal's shell (env vars set on Popen don't propagate
        # through gnome-terminal-server). sshpass -e reads $SSHPASS, .env
        # stores it as JETSON_SSH_PASSWORD, so we bridge the two names.
        ssh_inner = (
            f"sshpass -e ssh -t -o StrictHostKeyChecking=accept-new "
            f"{JETSON_USER}@{JETSON_HOST} {shlex.quote(remote_bash)}"
        )
        bash = (
            f"set -a; source {shlex.quote(str(ENV_FILE))}; set +a; "
            f"export SSHPASS=\"${{JETSON_SSH_PASSWORD:-}}\"; "
            f"if [ -z \"$SSHPASS\" ]; then "
            f"  echo 'ERROR: JETSON_SSH_PASSWORD not set in {ENV_FILE}'; "
            f"  echo 'Edit that file and put your Jetson password after the ='; "
            f"  echo; read -p 'Press Enter to close'; exit 1; "
            f"fi; "
            f"{ssh_inner}; "
            f"echo; echo '[session ended -- press Enter to close]'; read"
        )
        title = f"Jetson: {label}"
        argv = build_terminal_argv(self.terminal, title, bash)

        try:
            subprocess.Popen(argv, start_new_session=True)
            self.status.config(text=f"Launched jetson: {label}", foreground="#0a0")
            return True
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Launch failed", str(exc))
            self.status.config(text=f"Failed: {exc}", foreground="#a00")
            return False

    def launch_jetson(self, cmd, source_workspace, label):
        """Open a local terminal that SSHes into the Jetson and runs the command."""
        remote = build_workspace_command(JETSON_WORKSPACE, cmd, source_workspace)
        self._launch_jetson_remote(remote, label)

    def launch_full_auton(self):
        """One entry: Jetson nav stack + telemetry GUI (over SSH) AND the
        base-station detection node (local). Two terminals, one click."""
        remote = build_workspace_command(JETSON_WORKSPACE, FULL_AUTON_JETSON_CMD, True)
        started = self._launch_jetson_remote(remote, "FULL AUTON nav + GUI")
        if started:
            self.launch_local(FULL_AUTON_LOCAL_CMD, True, "FULL AUTON detection")

    def launch_jetson_shell(self):
        """Drop into an interactive bash on the Jetson with the workspace
        already sourced, so the user can run any ros2 command immediately.
        `exec bash` replaces the remote sh with an interactive bash; ROS env
        vars set by `source install/setup.bash` carry across via env
        inheritance. The session ends when the user types `exit` / Ctrl+D."""
        remote = (
            f"cd {JETSON_WORKSPACE} && "
            f"source install/setup.bash && "
            f"exec bash"
        )
        self._launch_jetson_remote(remote, "SSH shell")


def main():
    if not shutil.which("sshpass"):
        print(
            "Error: sshpass is not installed. Install it with:\n"
            "    sudo apt install sshpass",
            file=sys.stderr,
        )
        sys.exit(1)
    JetsonLauncher().mainloop()


if __name__ == "__main__":
    main()
