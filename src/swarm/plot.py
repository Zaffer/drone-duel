"""Plot a telemetry log (JSON lines from TelemetryPublisher) as a top-down trajectory figure."""

import json
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from swarm.telemetry import Snapshot  # noqa: E402

SERIES_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#4a3aa7"]
LEADER_MARKER_S = 90.0


def load_snapshots(path: Path) -> list[Snapshot]:
    with path.open() as f:
        return [json.loads(line) for line in f if line.strip()]


def plot_log(path: Path, out: Path | None = None) -> Path:
    snapshots = load_snapshots(path)
    tracks: dict[int, tuple[list[float], list[float]]] = defaultdict(lambda: ([], []))
    leader_points: list[tuple[float, float]] = []
    point_b: tuple[float, float] | None = None
    for snap in snapshots:
        if snap.get("kind") == "plan":
            continue  # a drone's uploaded plan, logged beside the snapshots
        tn, te = snap.get("target_n"), snap.get("target_e")
        if tn is not None and te is not None:
            point_b = (te, tn)
        for v in snap["vehicles"]:
            if not v["armed"]:
                continue
            east, north = tracks[v["sysid"]]
            east.append(v["e"])
            north.append(v["n"])
            if v["role"] == "leader":
                leader_points.append((v["e"], v["n"]))

    fig, ax = plt.subplots(figsize=(8, 8))
    for i, (sysid, (east, north)) in enumerate(sorted(tracks.items())):
        color = SERIES_COLORS[i % len(SERIES_COLORS)]
        ax.plot(east, north, color=color, linewidth=2, label=f"copter {sysid}")
        if east:
            ax.scatter(east[-1], north[-1], color=color, s=40, zorder=3)
    if leader_points:
        le, ln = zip(*leader_points, strict=True)
        ax.scatter(le, ln, s=6, color="#52514e", alpha=0.35, label="leader position", zorder=2)
    if point_b is not None:
        ax.scatter(*point_b, color="#52514e", marker="x", s=60, zorder=3, label="point B")
    ax.set_aspect("equal")
    ax.set_xlabel("east [m]")
    ax.set_ylabel("north [m]")
    ax.set_title("Swarm trajectories (local frame)")
    ax.grid(True, color="#e6e5e0", linewidth=0.8)
    ax.legend(loc="best", frameon=False)
    fig.tight_layout()

    out = out or path.with_suffix(".png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out
