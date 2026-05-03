#!/usr/bin/env python3
"""
Standalone tkinter GUI to launch the Jetson-side rover startup commands
(ZED, localization, GPS rover, telemetry) over SSH, each in its own
terminal window.

Local-side commands (gps_basestation / basestation_known) are NOT handled
here -- run those in your local terminal as before.

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
from tkinter import messagebox, ttk


# --- Configuration -----------------------------------------------------------

JETSON_USER = "cmr"
JETSON_HOST = "192.168.1.69"
JETSON_WORKSPACE = "~/cmr/terra2"

# WARNING: stored in plaintext. Do NOT commit this file to a public remote
# with the real password filled in. You can also override at runtime by
# setting the JETSON_SSH_PASSWORD env var.
JETSON_PASSWORD = "CHANGE_ME"

# (button label, remote command, source install/setup.bash before running?)
COMMANDS = [
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


# --- Terminal handling -------------------------------------------------------

def find_terminal():
    """Pick the first available terminal emulator. Returns its name or None."""
    for binary in ("gnome-terminal", "konsole", "xfce4-terminal", "xterm"):
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


def build_remote_command(cmd, source_workspace):
    """Wrap a ros2 command into a single bash one-liner that runs on the Jetson."""
    parts = [f"cd {JETSON_WORKSPACE}"]
    if source_workspace:
        parts.append("source install/setup.bash")
    parts.append(cmd)
    pipeline = " && ".join(parts)
    # `exec bash` keeps the SSH session interactive after the command exits
    # so the user can read errors / re-run things without losing the connection.
    return f"{pipeline}; exec bash"


# --- GUI ---------------------------------------------------------------------

class JetsonLauncher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Jetson Launcher")
        self.geometry("620x520")
        self.minsize(520, 480)

        self.terminal = find_terminal()
        self.password = os.environ.get("JETSON_SSH_PASSWORD", JETSON_PASSWORD)

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

        ttk.Label(
            self,
            text=(
                "Each button opens a new terminal window, SSHes into the Jetson,\n"
                "cd's to the workspace, sources install/setup.bash, then runs the\n"
                "command. Close the terminal (or Ctrl+C inside it) to stop it."
            ),
            foreground="#555",
            justify="left",
        ).pack(**pad, anchor="w")

        button_frame = ttk.LabelFrame(self, text="Jetson commands")
        button_frame.pack(fill="both", expand=True, **pad)

        for label, cmd, src in COMMANDS:
            ttk.Button(
                button_frame,
                text=label,
                command=lambda c=cmd, s=src, l=label: self.launch(c, s, l),
            ).pack(fill="x", padx=8, pady=4)

        self.status = ttk.Label(self, text="Ready.", foreground="#0a0")
        self.status.pack(fill="x", **pad, side="bottom")

        if not self.terminal:
            messagebox.showerror(
                "No terminal emulator",
                "None of gnome-terminal / konsole / xfce4-terminal / xterm "
                "are installed.\nInstall one, e.g.:\n  sudo apt install xterm",
            )

    def launch(self, cmd, source_workspace, label):
        if not self.terminal:
            messagebox.showerror("No terminal", "No terminal emulator available.")
            return
        if not self.password or self.password == "CHANGE_ME":
            messagebox.showerror(
                "Password not set",
                "Set JETSON_PASSWORD at the top of jetson_launcher.py "
                "(or export JETSON_SSH_PASSWORD).",
            )
            return

        remote = build_remote_command(cmd, source_workspace)
        # Pass password to sshpass via env var (-e) so it never appears in argv.
        ssh_inner = (
            f"sshpass -e ssh -t -o StrictHostKeyChecking=no "
            f"{JETSON_USER}@{JETSON_HOST} {shlex.quote(remote)}"
        )
        local_bash = (
            f"{ssh_inner}; "
            f"echo; echo '[session ended -- press Enter to close]'; read"
        )
        title = f"Jetson: {label}"
        argv = build_terminal_argv(self.terminal, title, local_bash)

        env = os.environ.copy()
        env["SSHPASS"] = self.password
        try:
            subprocess.Popen(argv, env=env, start_new_session=True)
            self.status.config(text=f"Launched: {label}", foreground="#0a0")
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Launch failed", str(exc))
            self.status.config(text=f"Failed: {exc}", foreground="#a00")


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
