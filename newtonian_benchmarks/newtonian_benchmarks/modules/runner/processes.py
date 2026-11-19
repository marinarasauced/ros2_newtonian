
from __future__ import annotations

import os
import signal
import subprocess
from pathlib import Path


class Process:
    """
    Child process with its output in a log file.
    """

    def __init__(self, cmd: list[str], log: Path, env: dict[str, str]) -> None:
        """
        Starts the command 'cmd'.
        """
        log.parent.mkdir(parents=True, exist_ok=True)

        self.log = log
        self.fh = open(log, "wb")
        self.p = subprocess.Popen(cmd, stdout=self.fh, stderr=subprocess.STDOUT, env=env, start_new_session=True)

    def is_alive(self) -> bool:
        """
        Returns True while the process runs.
        """
        return self.p.poll() is None

    def stop(self) -> None:
        """
        Stops the process.
        """
        self._signal(sig=signal.SIGINT, timeout=8)
        self._signal(sig=signal.SIGKILL, timeout=5)
        self.fh.close()

    def _signal(self, sig: int, timeout: float) -> None:
        """
        Sends the signal 'sig' to the process group.
        """
        if self.p.poll() is not None:
            return

        try:
            os.killpg(self.p.pid, sig)
            self.p.wait(timeout=timeout)
        except Exception:
            pass


def expand_environment(text: str) -> str:
    """
    Returns 'text' with the environment variables expanded.
    """
    return os.path.expandvars(text)
