"""Deterministic lossy UDP-like broadcast channel."""

from __future__ import annotations

import heapq
from dataclasses import dataclass
from math import dist
from random import Random

from allocation.messages import Message


@dataclass(order=True, slots=True)
class Envelope:
    delivery_time: float
    sequence: int
    recipient: str
    payload: bytes


class MessageBus:
    def __init__(
        self,
        robot_ids: list[str],
        rng: Random,
        latency_min: float,
        latency_max: float,
        packet_loss: float,
        communication_range: float | None,
    ) -> None:
        self.robot_ids = robot_ids
        self.rng = rng
        self.latency_min = latency_min
        self.latency_max = latency_max
        self.packet_loss = packet_loss
        self.communication_range = communication_range
        self.queue: list[Envelope] = []
        self.sequence = 0
        self.sent_messages = 0
        self.sent_bytes = 0
        self.sent_by_kind: dict[str, int] = {}

    def broadcast(
        self,
        message: Message,
        now: float,
        positions: dict[str, tuple[float, float]],
    ) -> None:
        payload = message.to_bytes()
        sender = str(message.fields["sender"])
        self.sent_messages += 1
        self.sent_bytes += len(payload)
        self.sent_by_kind[message.kind.value] = self.sent_by_kind.get(message.kind.value, 0) + 1
        for recipient in self.robot_ids:
            if (
                self.communication_range is not None
                and dist(positions[sender], positions[recipient]) > self.communication_range
            ):
                continue
            if self.rng.random() < self.packet_loss:
                continue
            latency = self.rng.uniform(self.latency_min, self.latency_max)
            self.sequence += 1
            heapq.heappush(self.queue, Envelope(now + latency, self.sequence, recipient, payload))

    def deliver(self, now: float) -> dict[str, list[bytes]]:
        delivered = {robot_id: [] for robot_id in self.robot_ids}
        while self.queue and self.queue[0].delivery_time <= now:
            envelope = heapq.heappop(self.queue)
            delivered[envelope.recipient].append(envelope.payload)
        return delivered
