"""Transport-independent selection; callers supply monotonic receipt times."""
import math


class CommandMux:
    def __init__(self, timeout=0.5):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be positive and finite")
        self.timeout = timeout
        self.source = "teleop"
        self.estop = False
        self.commands = {}

    def select(self, source):
        if source not in ("teleop", "autonomy"):
            raise ValueError("source must be teleop or autonomy")
        self.source = source
        self.commands.clear()

    def emergency_stop(self):
        self.estop = True
        self.commands.clear()

    def reset(self):
        self.estop = False
        self.commands.clear()

    def receive(self, source, values, now):
        if source not in ("teleop", "autonomy"):
            raise ValueError("unknown command source")
        if not all(math.isfinite(v) for v in (*values, now)):
            self.commands.pop(source, None)
            return False
        if not self.estop:
            self.commands[source] = (tuple(values), now)
        return True

    def output(self, now):
        if self.estop or not math.isfinite(now):
            return None
        command = self.commands.get(self.source)
        if command is None or not 0 <= now - command[1] < self.timeout:
            return None
        return command[0]
