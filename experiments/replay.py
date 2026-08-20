"""Render an animated 2D replay from the portable CSV logs."""

from __future__ import annotations

import argparse
import csv
import json
from bisect import bisect_right
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.lines import Line2D

TASK_COLORS = {
    "guard": "#d97706",
    "push": "#7c3aed",
    "surround": "#dc2626",
    "survey": "#0284c7",
}
TASK_MARKERS = {"guard": "^", "push": "s", "surround": "D", "survey": "x"}


@dataclass(slots=True)
class RobotTrack:
    robot_id: str
    times: np.ndarray
    x: np.ndarray
    y: np.ndarray
    battery: np.ndarray
    states: list[str]

    def sample(self, t: float) -> tuple[float, float, float, str]:
        index = max(0, min(len(self.times) - 1, int(np.searchsorted(self.times, t, side="right") - 1)))
        return (
            float(np.interp(t, self.times, self.x)),
            float(np.interp(t, self.times, self.y)),
            float(np.interp(t, self.times, self.battery)),
            self.states[index],
        )

    def trail(self, t: float, seconds: float) -> tuple[np.ndarray, np.ndarray]:
        mask = (self.times >= t - seconds) & (self.times <= t)
        x = self.x[mask]
        y = self.y[mask]
        current_x = float(np.interp(t, self.times, self.x))
        current_y = float(np.interp(t, self.times, self.y))
        return np.append(x, current_x), np.append(y, current_y)


@dataclass(slots=True)
class TaskTrack:
    task_id: str
    task_type: str
    position: tuple[float, float]
    events: list[tuple[float, str]] = field(default_factory=list)

    def status_at(self, t: float) -> str | None:
        times = [event[0] for event in self.events]
        index = bisect_right(times, t) - 1
        if index < 0:
            return None
        event_type = self.events[index][1]
        if event_type in {"DONE", "FAILED"}:
            return None
        if event_type == "AWARD":
            return "assigned"
        return "bidding"


@dataclass(slots=True)
class ReplayData:
    robots: list[RobotTrack]
    tasks: list[TaskTrack]
    arena_width: float
    arena_height: float
    strategy: str

    @property
    def start_time(self) -> float:
        return min(float(track.times[0]) for track in self.robots)

    @property
    def end_time(self) -> float:
        return max(float(track.times[-1]) for track in self.robots)


def _read_metadata(log_dir: Path) -> dict[str, Any]:
    path = log_dir / "metadata.json"
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}


def load_replay(log_dir: str | Path) -> ReplayData:
    directory = Path(log_dir)
    states_path = directory / "state.csv"
    events_path = directory / "events.csv"
    if not states_path.exists() or not events_path.exists():
        raise FileNotFoundError("the replay needs state.csv and events.csv in the log directory")

    by_robot: dict[str, list[dict[str, str]]] = {}
    with states_path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            by_robot.setdefault(row["robot_id"], []).append(row)
    robots: list[RobotTrack] = []
    for robot_id, rows in sorted(by_robot.items()):
        rows.sort(key=lambda row: float(row["t"]))
        robots.append(
            RobotTrack(
                robot_id,
                np.array([float(row["t"]) for row in rows]),
                np.array([float(row["x"]) for row in rows]),
                np.array([float(row["y"]) for row in rows]),
                np.array([float(row["battery"]) for row in rows]),
                [row["state"] for row in rows],
            )
        )
    if not robots:
        raise ValueError("state.csv contains no robot states")

    task_map: dict[str, TaskTrack] = {}
    with events_path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            task_id = row["task_id"]
            event_type = row["event_type"]
            if not task_id:
                continue
            payload = json.loads(row["payload_json"])
            if event_type == "ANNOUNCE" and task_id not in task_map:
                position = payload.get("pos", [0.0, 0.0])
                task_map[task_id] = TaskTrack(
                    task_id,
                    str(payload.get("type", "guard")),
                    (float(position[0]), float(position[1])),
                )
            if task_id in task_map and event_type in {"ANNOUNCE", "AWARD", "RELEASE", "DONE", "FAILED"}:
                task_map[task_id].events.append((float(row["t"]), event_type))
    for task in task_map.values():
        task.events.sort()

    metadata = _read_metadata(directory)
    config = metadata.get("config", {})
    return ReplayData(
        robots,
        list(task_map.values()),
        float(config.get("arena_width", 2.0)),
        float(config.get("arena_height", 2.0)),
        str(metadata.get("strategy", "unknown")),
    )


def create_replay(
    log_dir: str | Path,
    output: str | Path | None = None,
    speed: float = 15.0,
    fps: int = 15,
    trail_seconds: float = 20.0,
    include_survey: bool = True,
    dpi: int = 105,
) -> Path:
    if speed <= 0.0 or fps <= 0 or trail_seconds < 0.0:
        raise ValueError("speed and fps must be positive; trail_seconds must be non-negative")
    data = load_replay(log_dir)
    output_path = Path(output) if output is not None else Path(log_dir) / "replay.gif"
    if output_path.suffix.lower() != ".gif":
        raise ValueError("the dependency-free replay export currently supports .gif")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    duration = data.end_time - data.start_time
    frame_count = max(2, int(np.ceil(duration * fps / speed)) + 1)
    if frame_count > 2_000:
        raise ValueError("replay exceeds 2,000 frames; increase --speed or reduce --fps")
    frame_times = np.linspace(data.start_time, data.end_time, frame_count)
    robot_colors = plt.get_cmap("tab10")(np.arange(len(data.robots)) % 10)

    figure, axis = plt.subplots(figsize=(7.2, 7.2), constrained_layout=True)
    figure.patch.set_facecolor("#f8fafc")
    axis.set_facecolor("#ffffff")
    axis.set_xlim(0.0, data.arena_width)
    axis.set_ylim(0.0, data.arena_height)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.grid(color="#cbd5e1", linewidth=0.7, alpha=0.65)
    axis.set_title(f"Decentralized MRTA replay - {data.strategy}", pad=12)

    trail_lines = [
        axis.plot([], [], color=robot_colors[index], linewidth=1.4, alpha=0.45)[0]
        for index in range(len(data.robots))
    ]
    initial_positions = [track.sample(data.start_time)[:2] for track in data.robots]
    robot_scatter = axis.scatter(
        [position[0] for position in initial_positions],
        [position[1] for position in initial_positions],
        s=190,
        c=robot_colors,
        edgecolors="#0f172a",
        linewidths=1.0,
        zorder=5,
    )
    robot_labels = [
        axis.text(0.0, 0.0, "", ha="center", va="bottom", fontsize=7.5, zorder=6) for _ in data.robots
    ]

    task_scatters: dict[tuple[str, str], Any] = {}
    for task_type, task_color in TASK_COLORS.items():
        marker = TASK_MARKERS[task_type]
        for status in ("bidding", "assigned"):
            filled = status == "assigned" and marker != "x"
            color_options = (
                {"color": task_color}
                if marker == "x"
                else {
                    "facecolors": task_color if filled else "none",
                    "edgecolors": task_color,
                }
            )
            task_scatters[(task_type, status)] = axis.scatter(
                [],
                [],
                s=80 if task_type != "survey" else 45,
                marker=marker,
                linewidths=1.8,
                alpha=0.95 if status == "assigned" else 0.65,
                zorder=4,
                **color_options,
            )

    time_text = axis.text(
        0.01,
        0.99,
        "",
        transform=axis.transAxes,
        ha="left",
        va="top",
        fontsize=9,
        bbox={"facecolor": "#ffffff", "edgecolor": "none", "alpha": 0.82, "pad": 2.5},
        zorder=10,
    )
    legend_items = [
        Line2D(
            [0],
            [0],
            marker=TASK_MARKERS[task_type],
            color="none",
            markeredgecolor=color,
            markerfacecolor=color if TASK_MARKERS[task_type] != "x" else "none",
            markersize=7,
            label=task_type.upper(),
        )
        for task_type, color in TASK_COLORS.items()
        if include_survey or task_type != "survey"
    ]
    axis.legend(handles=legend_items, loc="lower right", framealpha=0.9, fontsize=8, ncols=2)

    visible_tasks = [task for task in data.tasks if include_survey or task.task_type != "survey"]

    def update(t: float) -> None:
        positions: list[tuple[float, float]] = []
        state_counts: dict[str, int] = {}
        for index, track in enumerate(data.robots):
            x, y, battery, state = track.sample(float(t))
            positions.append((x, y))
            state_counts[state] = state_counts.get(state, 0) + 1
            trail_x, trail_y = track.trail(float(t), trail_seconds)
            trail_lines[index].set_data(trail_x, trail_y)
            robot_labels[index].set_position((x, y + 0.045))
            robot_labels[index].set_text(f"{track.robot_id}  {battery:.0%}\n{state}")
        robot_scatter.set_offsets(np.asarray(positions))

        grouped: dict[tuple[str, str], list[tuple[float, float]]] = {key: [] for key in task_scatters}
        for task in visible_tasks:
            status = task.status_at(float(t))
            if status is not None and task.task_type in TASK_COLORS:
                grouped[(task.task_type, status)].append(task.position)
        for key, scatter in task_scatters.items():
            offsets = grouped[key]
            scatter.set_offsets(np.asarray(offsets) if offsets else np.empty((0, 2)))
        active_count = sum(len(points) for points in grouped.values())
        states = ", ".join(f"{name}:{count}" for name, count in sorted(state_counts.items()))
        time_text.set_text(f"t = {t:6.1f} / {data.end_time:.1f} s   |   tasks: {active_count}   |   {states}")

    animation = FuncAnimation(
        figure,
        update,
        frames=frame_times,
        interval=1_000 / fps,
        repeat=False,
        blit=False,
    )
    animation.save(output_path, writer=PillowWriter(fps=fps), dpi=dpi)
    plt.close(figure)
    return output_path.resolve()


def main() -> None:
    parser = argparse.ArgumentParser(description="Create a GIF replay from events.csv and state.csv")
    parser.add_argument("log_dir", type=Path, nargs="?", default=Path("logs/example"))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--speed", type=float, default=15.0, help="simulated seconds per replay second")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--trail-seconds", type=float, default=20.0)
    parser.add_argument("--hide-survey", action="store_true")
    args = parser.parse_args()
    path = create_replay(
        args.log_dir,
        args.output,
        args.speed,
        args.fps,
        args.trail_seconds,
        not args.hide_survey,
    )
    print(path)


if __name__ == "__main__":
    main()
