"""Auctioneer-side records; the caller owns transport and timing."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class HostedAuction:
    task_id: str
    n_req: int
    deadline: float
    attempt: int = 1
    bids: dict[str, float] = field(default_factory=dict)

    def submit(self, robot_id: str, cost: float, now: float) -> None:
        if now <= self.deadline and cost >= 0.0:
            self.bids[robot_id] = cost

    def winners(self) -> list[str]:
        ranked = sorted(self.bids.items(), key=lambda item: (item[1], item[0]))
        return [robot_id for robot_id, _ in ranked[: self.n_req]] if len(ranked) >= self.n_req else []

    def retry(self, now: float, bid_window: float) -> HostedAuction:
        return HostedAuction(self.task_id, self.n_req, now + bid_window, self.attempt + 1)
