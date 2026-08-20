"""CLI for one inspectable run with events.csv and state.csv."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from allocation.bidding import BidWeights
from allocation.strategies import Strategy, StrategyName
from sim.config import SimulationConfig
from sim.world import run_simulation

from .common import read_json
from .interactive_replay import create_interactive_replay
from .replay import create_replay


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--strategy", choices=[item.value for item in StrategyName], default="market")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", type=Path, default=Path("logs/example"))
    parser.add_argument("--replay", action="store_true", help="render replay.gif after the run")
    parser.add_argument(
        "--interactive-replay",
        action="store_true",
        help="render replay.html with play/pause and timeline controls",
    )
    parser.add_argument("--replay-speed", type=float, default=15.0)
    parser.add_argument("--replay-fps", type=int, default=15)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    data = read_json(root / "parameters" / "default.json")
    config = SimulationConfig(**data["simulation"])
    weights = BidWeights(**data["market_weights"])
    result = run_simulation(config, Strategy(StrategyName(args.strategy), weights), args.seed, args.output)
    print(json.dumps(result.metrics, indent=2, sort_keys=True))
    if args.replay:
        replay_path = create_replay(
            args.output,
            args.output / "replay.gif",
            speed=args.replay_speed,
            fps=args.replay_fps,
        )
        print(f"Replay: {replay_path}")
    if args.interactive_replay:
        replay_path = create_interactive_replay(args.output, args.output / "replay.html")
        print(f"Interactive replay: {replay_path}")


if __name__ == "__main__":
    main()
