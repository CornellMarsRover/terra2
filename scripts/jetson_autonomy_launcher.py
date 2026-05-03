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
    - tmux  (optional)   (sudo apt install tmux)
        If tmux is installed, every button opens a new tmux *tab* in a single
        shared window instead of spawning a separate terminal each time.
        Switch tabs with Ctrl-b then a number, or Ctrl-b then n / p.
        Kill a tab with `exit` (or Ctrl-b then &).

Run with:
    python3 scripts/jetson_autonomy_launcher.py
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

# When True (and tmux is installed), all commands run as tmux tabs in a single
# shared window. When False, each command opens its own terminal window.
USE_TABS = True
TMUX_SESSION = "jetson-autonomy-launcher"

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
    """Wrap a ros2 command into a single bash one-liner.
    `exec bash` keeps the shell interactive after the command exits so the
    user can read errors / re-run things without losing the session."""
    parts = [f"cd {workspace}"]
    if source_workspace:
        parts.append("source install/setup.bash")
    parts.append(cmd)
    pipeline = " && ".join(parts)
    return f"{pipeline}; exec bash"


# --- tmux multiplexer (optional) --------------------------------------------

def tmux_available():
    return shutil.which("tmux") is not None


def tmux_session_exists(session):
    return subprocess.call(
        ["tmux", "has-session", "-t", session],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    ) == 0


def tmux_has_clients(session):
    try:
        out = subprocess.check_output(
            ["tmux", "list-clients", "-t", session],
            stderr=subprocess.DEVNULL,
        ).decode()
        return bool(out.strip())
    except subprocess.CalledProcessError:
        return False


# --- GUI ---------------------------------------------------------------------

class JetsonAutonomyLauncher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Jetson Autonomy Launcher")
        self.geometry("680x680")
        self.minsize(560, 600)

        self.terminal = find_terminal()
        self.password = load_jetson_password()
        self.use_tabs = USE_TABS and tmux_available()

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
        if self.use_tabs:
            tabs_text = f"Tabs:  tmux session '{TMUX_SESSION}' (Ctrl-b n / p to switch)"
            tabs_color = "#0a0"
        elif USE_TABS:
            tabs_text = "Tabs:  tmux not installed -- using separate windows (apt install tmux)"
            tabs_color = "#a60"
        else:
            tabs_text = "Tabs:  disabled in script (USE_TABS=False)"
            tabs_color = "#555"
        ttk.Label(header, text=tabs_text, foreground=tabs_color).pack(anchor="w")
        pw_status = "loaded from .env" if self.password else "NOT SET (see .env.example)"
        ttk.Label(
            header,
            text=f"SSH password:  {pw_status}",
            foreground="#0a0" if self.password else "#a00",
        ).pack(anchor="w")

        if self.use_tabs:
            help_text = (
                "Each button adds a new tab to the tmux window.\n"
                "Switch tabs: Ctrl-b then n / p / number.   Close a tab: type 'exit'."
            )
        else:
            help_text = (
                "Each button opens a new terminal window, sources install/setup.bash,\n"
                "and runs the command. Close the terminal (or Ctrl+C inside it) to stop."
            )
        ttk.Label(self, text=help_text, foreground="#555", justify="left").pack(
            **pad, anchor="w"
        )

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

        for label, cmd, src in JETSON_COMMANDS:
            ttk.Button(
                jetson_frame,
                text=label,
                command=lambda c=cmd, s=src, l=label: self.launch_jetson(c, s, l),
            ).pack(fill="x", padx=8, pady=4)

        if self.use_tabs:
            ttk.Button(
                self, text="Kill all tabs (close tmux session)",
                command=self.kill_all_tabs,
            ).pack(fill="x", **pad)

        self.status = ttk.Label(self, text="Ready.", foreground="#0a0")
        self.status.pack(fill="x", **pad, side="bottom")

        if not self.terminal:
            messagebox.showerror(
                "No terminal emulator",
                "None of gnome-terminal / konsole / xfce4-terminal / xterm "
                "are installed.\nInstall one, e.g.:\n  sudo apt install xterm",
            )

    # --- Build the bash one-liner for each kind of command ----------------

    def _build_local_bash(self, cmd, source_workspace):
        """Bash command for a local (non-SSH) launch."""
        return build_workspace_command(LOCAL_WORKSPACE, cmd, source_workspace)

    def _build_jetson_bash(self, cmd, source_workspace):
        """Bash command that SSHes into the Jetson and runs `cmd`.
        Reads the password from .env at runtime via `set -a; source .env`,
        so the password never appears in any argv (no `ps` exposure)."""
        remote = build_workspace_command(JETSON_WORKSPACE, cmd, source_workspace)
        ssh = (
            f"sshpass -e ssh -t -o StrictHostKeyChecking=accept-new "
            f"{JETSON_USER}@{JETSON_HOST} {shlex.quote(remote)}"
        )
        return (
            f"set -a; source {shlex.quote(str(ENV_FILE))}; set +a; "
            f"{ssh}; "
            f"echo; echo '[session ended -- press Enter to close]'; read"
        )

    # --- Common launch path (tabs OR new window) --------------------------

    def _launch(self, title, bash_cmd):
        if not self.terminal:
            messagebox.showerror("No terminal", "No terminal emulator available.")
            return False
        try:
            if self.use_tabs:
                self._launch_tab(title, bash_cmd)
            else:
                argv = build_terminal_argv(self.terminal, title, bash_cmd)
                subprocess.Popen(argv, start_new_session=True)
            return True
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Launch failed", str(exc))
            self.status.config(text=f"Failed: {exc}", foreground="#a00")
            return False

    def _launch_tab(self, title, bash_cmd):
        """Add a new tmux window in the shared session, opening a terminal
        attached to the session if one isn't already attached."""
        # tmux window names can't contain ':' so sanitise.
        window_name = title.replace(":", " -")[:40]
        if tmux_session_exists(TMUX_SESSION):
            subprocess.run(
                ["tmux", "new-window", "-t", f"{TMUX_SESSION}:",
                 "-n", window_name, bash_cmd],
                check=True,
            )
        else:
            subprocess.run(
                ["tmux", "new-session", "-d", "-s", TMUX_SESSION,
                 "-n", window_name, bash_cmd],
                check=True,
            )
        if not tmux_has_clients(TMUX_SESSION):
            attach_argv = build_terminal_argv(
                self.terminal, "Jetson Autonomy Launcher (tabs)",
                f"tmux attach -t {shlex.quote(TMUX_SESSION)}",
            )
            subprocess.Popen(attach_argv, start_new_session=True)

    # --- Button callbacks --------------------------------------------------

    def launch_local(self, cmd, source_workspace, label):
        bash = self._build_local_bash(cmd, source_workspace)
        if self._launch(f"Local: {label}", bash):
            self.status.config(text=f"Launched local: {label}", foreground="#0a0")

    def launch_jetson(self, cmd, source_workspace, label):
        if not self.password:
            messagebox.showerror(
                "Password not set",
                f"No JETSON_SSH_PASSWORD found.\n\n"
                f"Create {ENV_FILE} (copy from .env.example) and set:\n"
                f"    JETSON_SSH_PASSWORD=your_password\n\n"
                f"Or export JETSON_SSH_PASSWORD in your shell.",
            )
            return
        bash = self._build_jetson_bash(cmd, source_workspace)
        if self._launch(f"Jetson: {label}", bash):
            self.status.config(text=f"Launched jetson: {label}", foreground="#0a0")

    def kill_all_tabs(self):
        """Tear down the tmux session (closes every tab and any attached window)."""
        if not self.use_tabs:
            return
        if not tmux_session_exists(TMUX_SESSION):
            self.status.config(text="No tmux session running.", foreground="#555")
            return
        if not messagebox.askyesno(
            "Kill all tabs?",
            f"This will close every command running in the '{TMUX_SESSION}' tmux session.\n"
            f"Are you sure?",
        ):
            return
        subprocess.run(["tmux", "kill-session", "-t", TMUX_SESSION], check=False)
        self.status.config(text="Killed all tabs.", foreground="#a60")


def main():
    if not shutil.which("sshpass"):
        print(
            "Error: sshpass is not installed. Install it with:\n"
            "    sudo apt install sshpass",
            file=sys.stderr,
        )
        sys.exit(1)
    JetsonAutonomyLauncher().mainloop()


if __name__ == "__main__":
    main()
