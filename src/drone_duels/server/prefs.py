"""The player's preferences: controls and their calibration, video, graphics, audio.

The page owns their shape; the server keeps them in ~/.config/drone-duels/prefs.json (or
$DRONE_DUELS_PREFS), so a cleared browser keeps its radio's calibration (DESIGN.md §12).
"""

import json
import os
from pathlib import Path
from typing import Any


def path() -> Path:
    custom = os.environ.get("DRONE_DUELS_PREFS")
    if custom:
        return Path(custom)
    base = Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config")
    return base / "drone-duels" / "prefs.json"


def load() -> dict[str, Any]:
    p = path()
    try:
        data = json.loads(p.read_text())
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def save(data: dict[str, Any]) -> None:
    p = path()
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=2))
    tmp.replace(p)
