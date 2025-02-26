#!/usr/bin/env python3
"""
A ROS2 node that launches a PyQt GUI to remotely operate your robot.

This GUI is launched locally, but every remote command is executed in two steps:
  1. An SSH login is performed.
  2. The desired command is run on the remote machine via a new SSH session.

When the GUI is closed, all active remote processes are killed.

Features:
  - Run a remote ROS2 node or launch file.
  - Each command runs in its own SSH session.
  - A list of running remote processes is maintained with the ability to kill any process.
  - Refresh and display remote topics.
  - Echo a remote topic and stream its output into the GUI.

Update REMOTE_USER, REMOTE_HOST, and REMOTE_PASSWORD with your remote credentials.
"""

import sys
import subprocess
import threading
import signal

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QListWidget, QTextEdit, QLabel, QLineEdit, QMessageBox
)
from PyQt5.QtCore import QTimer

import rclpy
from rclpy.node import Node

# Configure your remote SSH connection details
REMOTE_USER = "your_remote_username"
REMOTE_HOST = "your.remote.host"  # e.g., "192.168.1.2" or hostname
REMOTE_PASSWORD = "your_remote_password"


def run_remote_command(cmd):
    """
    Executes a command on the remote host in two steps:
      1. Perform an SSH login.
      2. Run the command in a new SSH session with nohup.
      
    Returns the remote process ID as a string.
    """
    # Step 1: SSH login
    login_cmd = (
        f"sshpass -p '{REMOTE_PASSWORD}' ssh -o StrictHostKeyChecking=no "
        f"{REMOTE_USER}@{REMOTE_HOST} 'echo SSH_LOGIN'"
    )
    try:
        subprocess.check_output(login_cmd, shell=True)
    except subprocess.CalledProcessError:
        return None

    # Step 2: Execute the command (run in background and echo PID)
    actual_cmd = (
        f"sshpass -p '{REMOTE_PASSWORD}' ssh -o StrictHostKeyChecking=no "
        f"{REMOTE_USER}@{REMOTE_HOST} 'nohup {cmd} > /dev/null 2>&1 & echo $!'"
    )
    try:
        result = subprocess.check_output(actual_cmd, shell=True)
        remote_pid = result.decode('utf-8').strip().splitlines()[0]
        return remote_pid
    except subprocess.CalledProcessError:
        return None


def run_remote_command_no_pid(cmd):
    """
    Executes a remote command in two steps:
      1. Perform an SSH login.
      2. Run the command in a new SSH session (no PID returned).
    """
    # Step 1: SSH login
    login_cmd = (
        f"sshpass -p '{REMOTE_PASSWORD}' ssh -o StrictHostKeyChecking=no "
        f"{REMOTE_USER}@{REMOTE_HOST} 'echo SSH_LOGIN'"
    )
    try:
        subprocess.check_output(login_cmd, shell=True)
    except subprocess.CalledProcessError:
        return

    # Step 2: Execute the actual command
    actual_cmd = (
        f"sshpass -p '{REMOTE_PASSWORD}' ssh -o StrictHostKeyChecking=no "
        f"{REMOTE_USER}@{REMOTE_HOST} '{cmd}'"
    )
    try:
        subprocess.check_call(actual_cmd, shell=True)
    except subprocess.CalledProcessError:
        pass


class RobotGUI(Node):
    def __init__(self):
        super().__init__('robot_gui_node')
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("Remote Robot Operation GUI")

        # Layouts
        self.main_layout = QVBoxLayout()
        self.cmd_layout = QHBoxLayout()

        # --- Remote Command Buttons ---
        self.run_node_button = QPushButton("Run Remote Node")
        self.run_launch_button = QPushButton("Run Remote Launch File")
        self.cmd_layout.addWidget(self.run_node_button)
        self.cmd_layout.addWidget(self.run_launch_button)
        self.run_node_button.clicked.connect(self.run_remote_node)
        self.run_launch_button.clicked.connect(self.run_remote_launch)

        self.main_layout.addLayout(self.cmd_layout)

        # --- List for Running Remote Processes ---
        self.remote_process_list = QListWidget()
        self.kill_selected_button = QPushButton("Kill Selected Remote Process")
        self.kill_selected_button.clicked.connect(self.kill_selected_remote)

        self.main_layout.addWidget(QLabel("Running Remote Processes:"))
        self.main_layout.addWidget(self.remote_process_list)
        self.main_layout.addWidget(self.kill_selected_button)

        # --- Topics List ---
        self.refresh_topics_button = QPushButton("Refresh Remote Topics")
        self.refresh_topics_button.clicked.connect(self.refresh_topics)
        self.topics_list = QListWidget()
        self.main_layout.addWidget(self.refresh_topics_button)
        self.main_layout.addWidget(QLabel("Current Remote Topics:"))
        self.main_layout.addWidget(self.topics_list)

        # --- Echo Topic Section ---
        self.echo_label = QLabel("Echo Topic:")
        self.echo_topic_input = QLineEdit()
        self.start_echo_button = QPushButton("Start Remote Echo")
        self.start_echo_button.clicked.connect(self.start_topic_echo)
        self.echo_output = QTextEdit()
        self.echo_output.setReadOnly(True)

        self.main_layout.addWidget(self.echo_label)
        self.main_layout.addWidget(self.echo_topic_input)
        self.main_layout.addWidget(self.start_echo_button)
        self.main_layout.addWidget(QLabel("Topic Output:"))
        self.main_layout.addWidget(self.echo_output)

        self.window.setLayout(self.main_layout)
        self.window.resize(700, 600)

        # Dictionaries to track remote processes: key=unique id, value=(command, remote_pid)
        self.remote_processes = {}
        self.process_counter = 0

        # For topic echo process and thread
        self.echo_process = None
        self.echo_thread = None

    def run(self):
        """Show the GUI and integrate ROS spinning with a QTimer."""
        self.window.show()
        timer = QTimer()
        timer.timeout.connect(lambda: rclpy.spin_once(self, timeout_sec=0.1))
        timer.start(100)
        self.app.aboutToQuit.connect(self.cleanup)
        sys.exit(self.app.exec_())

    def run_remote_node(self):
        """Launch a ROS2 node on the remote machine and store its PID."""
        # Modify the following command as needed (example using a demo talker node)
        cmd = "ros2 run demo_nodes_cpp talker"
        self.get_logger().info(f"Launching remote node: {cmd}")
        remote_pid = run_remote_command(cmd)
        if remote_pid is not None:
            label = f"Node: {cmd} (PID: {remote_pid})"
            self.remote_processes[self.process_counter] = (cmd, remote_pid)
            self.remote_process_list.addItem(f"[{self.process_counter}] {label}")
            self.process_counter += 1
        else:
            self.get_logger().error("Failed to launch remote node.")

    def run_remote_launch(self):
        """Launch a ROS2 launch file on the remote machine and store its PID."""
        # Modify the following command to your package and launch file
        cmd = "ros2 launch my_robot_package my_launch_file.launch.py"
        self.get_logger().info(f"Launching remote launch file: {cmd}")
        remote_pid = run_remote_command(cmd)
        if remote_pid is not None:
            label = f"Launch: {cmd} (PID: {remote_pid})"
            self.remote_processes[self.process_counter] = (cmd, remote_pid)
            self.remote_process_list.addItem(f"[{self.process_counter}] {label}")
            self.process_counter += 1
        else:
            self.get_logger().error("Failed to launch remote launch file.")

    def kill_selected_remote(self):
        """Kill the selected remote process by sending a kill command via SSH."""
        selected_items = self.remote_process_list.selectedItems()
        if not selected_items:
            QMessageBox.information(self.window, "No selection", "Please select a remote process to kill.")
            return

        item_text = selected_items[0].text()
        key_str = item_text.split(']')[0][1:]
        try:
            key = int(key_str)
        except ValueError:
            self.get_logger().error("Invalid selection format.")
            return

        if key in self.remote_processes:
            cmd, remote_pid = self.remote_processes[key]
            kill_cmd = f"kill {remote_pid}"
            self.get_logger().info(f"Killing remote process (PID: {remote_pid}) for command: {cmd}")
            run_remote_command_no_pid(kill_cmd)
            del self.remote_processes[key]
            self.remote_process_list.takeItem(self.remote_process_list.row(selected_items[0]))
        else:
            self.get_logger().info("Selected remote process not found.")

    def refresh_topics(self):
        """Retrieve the current list of ROS2 topics from the remote machine."""
        self.topics_list.clear()
        # Use two-step SSH: login then list topics
        login_cmd = (
            f"sshpass -p '{REMOTE_PASSWORD}' ssh -o StrictHostKeyChecking=no "
            f"{REMOTE_USER}@{REMOTE_HOST} 'echo SSH_LOGIN'"
        )
        try:
            subprocess.check_output(login_cmd, shell=True)
        except subprocess.CalledProcessError:
            self.get_logger().error("SSH login failed during topic refresh.")
            return

        ssh_cmd = (
            f"sshpass -p '{REMOTE_PASSWORD}' ssh -o StrictHostKeyChecking=no "
            f"{REMOTE_USER}@{REMOTE_HOST} 'ros2 topic list'"
        )
        try:
            result = subprocess.check_output(ssh_cmd, shell=True)
            topics = result.decode('utf-8').strip().splitlines()
            for topic in topics:
                self.topics_list.addItem(topic)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to refresh topics: {e}")

    def start_topic_echo(self):
        """Start echoing a ROS2 topic on the remote machine and stream its output."""
        topic = self.echo_topic_input.text().strip()
        if not topic:
            self.get_logger().info("Please enter a valid topic name.")
            return

        self.get_logger().info(f"Starting remote echo for topic: {topic}")
        cmd = f"ros2 topic echo {topic}"
        # Terminate any existing echo process
        if self.echo_process:
            self.echo_process.terminate()

        # Two-step SSH: first login, then run the echo command
        login_cmd = (
            f"sshpass -p '{REMOTE_PASSWORD}' ssh -o StrictHostKeyChecking=no "
            f"{REMOTE_USER}@{REMOTE_HOST} 'echo SSH_LOGIN'"
        )
        try:
            subprocess.check_output(login_cmd, shell=True)
        except subprocess.CalledProcessError:
            self.get_logger().error("SSH login failed for topic echo.")
            return

        ssh_cmd = (
            f"sshpass -p '{REMOTE_PASSWORD}' ssh -o StrictHostKeyChecking=no "
            f"{REMOTE_USER}@{REMOTE_HOST} '{cmd}'"
        )
        self.echo_process = subprocess.Popen(
            ssh_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )
        self.echo_thread = threading.Thread(target=self.update_echo_output, daemon=True)
        self.echo_thread.start()

    def update_echo_output(self):
        """Continuously update the text widget with output from the remote echo command."""
        if self.echo_process and self.echo_process.stdout:
            for line in self.echo_process.stdout:
                self.echo_output.append(line)

    def cleanup(self):
        """Kill all active remote processes and terminate the echo process before exiting."""
        self.get_logger().info("Cleaning up remote processes...")
        # Kill remote processes
        for key, (cmd, remote_pid) in self.remote_processes.items():
            kill_cmd = f"kill {remote_pid}"
            self.get_logger().info(f"Killing remote process (PID: {remote_pid}) for command: {cmd}")
            run_remote_command_no_pid(kill_cmd)
        self.remote_processes.clear()
        self.remote_process_list.clear()

        # Terminate the echo process if it exists
        if self.echo_process:
            self.echo_process.terminate()
            self.echo_process = None
        self.get_logger().info("Cleanup complete.")


def main(args=None):
    rclpy.init(args=args)
    gui_node = RobotGUI()

    # Ensure cleanup is performed on SIGINT or SIGTERM.
    def sigint_handler(sig, frame):
        gui_node.cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, sigint_handler)
    signal.signal(signal.SIGTERM, sigint_handler)

    try:
        gui_node.run()
    except KeyboardInterrupt:
        gui_node.get_logger().info("Shutting down GUI (KeyboardInterrupt)")
    finally:
        gui_node.cleanup()
        gui_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
