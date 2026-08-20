"""Build a self-contained interactive HTML replay from portable CSV logs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from .replay import ReplayData, load_replay


def _round_list(values: Any, digits: int = 4) -> list[float]:
    return [round(float(value), digits) for value in values]


def _payload(data: ReplayData) -> dict[str, Any]:
    return {
        "arena": {"width": data.arena_width, "height": data.arena_height},
        "strategy": data.strategy,
        "start": data.start_time,
        "end": data.end_time,
        "robots": [
            {
                "id": track.robot_id,
                "t": _round_list(track.times, 3),
                "x": _round_list(track.x),
                "y": _round_list(track.y),
                "battery": _round_list(track.battery),
                "state": track.states,
            }
            for track in data.robots
        ],
        "tasks": [
            {
                "id": task.task_id,
                "type": task.task_type,
                "position": [round(task.position[0], 4), round(task.position[1], 4)],
                "events": [[round(t, 3), event_type] for t, event_type in task.events],
            }
            for task in data.tasks
        ],
    }


def create_interactive_replay(
    log_dir: str | Path,
    output: str | Path | None = None,
) -> Path:
    data = load_replay(log_dir)
    output_path = Path(output) if output is not None else Path(log_dir) / "replay.html"
    if output_path.suffix.lower() != ".html":
        raise ValueError("interactive replay output must use the .html extension")
    template_path = Path(__file__).with_name("replay_player_template.html")
    template = template_path.read_text(encoding="utf-8")
    embedded = json.dumps(_payload(data), separators=(",", ":"), ensure_ascii=False).replace("</", "<\\/")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(template.replace("__REPLAY_DATA__", embedded), encoding="utf-8")
    return output_path.resolve()


def main() -> None:
    parser = argparse.ArgumentParser(description="Create an interactive HTML replay")
    parser.add_argument("log_dir", type=Path, nargs="?", default=Path("logs/example"))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    print(create_interactive_replay(args.log_dir, args.output))


if __name__ == "__main__":
    main()
