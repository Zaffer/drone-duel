"""Single keypresses from the terminal, without Enter, polled without blocking.

When stdin is not a terminal (a pipe in tests or scripts), every character read counts as a
key and newlines are dropped.
"""

import os
import select
import sys
import termios
import tty
from types import TracebackType
from typing import Any


class KeyReader:
    def __init__(self) -> None:
        self._fd = sys.stdin.fileno()
        self._tty = os.isatty(self._fd)
        self._saved: list[Any] | None = None
        self.closed = False

    def __enter__(self) -> "KeyReader":
        if self._tty:
            self._saved = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        if self._saved is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)

    def keys(self) -> str:
        """Every key pressed since the last call."""
        return self.read().replace("\n", "").replace("\r", "")

    def read(self) -> str:
        """Everything typed since the last call, Enter and escape sequences included."""
        out = ""
        while not self.closed and select.select([self._fd], [], [], 0)[0]:
            data = os.read(self._fd, 64)
            if not data:
                self.closed = True  # end of a piped stdin
                break
            out += data.decode(errors="ignore")
        return out
