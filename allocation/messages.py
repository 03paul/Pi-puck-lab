"""Compact, deterministic wire messages for peer-to-peer UDP broadcast."""

from __future__ import annotations

import json
from dataclasses import dataclass
from enum import Enum
from typing import Any


class MessageType(str, Enum):
    ANNOUNCE = "ANNOUNCE"
    BID = "BID"
    AWARD = "AWARD"
    DONE = "DONE"
    RELEASE = "RELEASE"


REQUIRED_FIELDS: dict[MessageType, frozenset[str]] = {
    MessageType.ANNOUNCE: frozenset(
        {"task_id", "pos", "type", "n_req", "p0", "lam", "k", "t_ref", "duration", "deadline", "sender", "t"}
    ),
    MessageType.BID: frozenset({"task_id", "cost", "sender", "t"}),
    MessageType.AWARD: frozenset({"task_id", "winners", "sender", "t"}),
    MessageType.DONE: frozenset({"task_id", "sender", "t"}),
    MessageType.RELEASE: frozenset({"task_id", "sender", "reason", "t"}),
}


@dataclass(frozen=True, slots=True)
class Message:
    kind: MessageType
    fields: dict[str, Any]

    def __post_init__(self) -> None:
        if frozenset(self.fields) != REQUIRED_FIELDS[self.kind]:
            missing = REQUIRED_FIELDS[self.kind] - frozenset(self.fields)
            extra = frozenset(self.fields) - REQUIRED_FIELDS[self.kind]
            raise ValueError(
                f"invalid {self.kind.value} fields; missing={sorted(missing)}, extra={sorted(extra)}"
            )

    def to_bytes(self) -> bytes:
        payload = {"kind": self.kind.value, **self.fields}
        return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")

    @classmethod
    def from_bytes(cls, data: bytes) -> Message:
        payload = json.loads(data.decode("utf-8"))
        kind = MessageType(payload.pop("kind"))
        return cls(kind, payload)


def announce_message(task: Any, deadline: float, sender: str, now: float) -> Message:
    priority = task.priority_fn
    return Message(
        MessageType.ANNOUNCE,
        {
            "task_id": task.task_id,
            "pos": list(task.position),
            "type": task.task_type.value,
            "n_req": task.n_req,
            "p0": priority.p0,
            "lam": priority.lam,
            "k": priority.k,
            "t_ref": priority.t_ref,
            "duration": task.duration,
            "deadline": deadline,
            "sender": sender,
            "t": now,
        },
    )


def bid_message(task_id: str, cost: float, sender: str, now: float) -> Message:
    return Message(MessageType.BID, {"task_id": task_id, "cost": cost, "sender": sender, "t": now})


def award_message(task_id: str, winners: list[str], sender: str, now: float) -> Message:
    return Message(MessageType.AWARD, {"task_id": task_id, "winners": winners, "sender": sender, "t": now})


def done_message(task_id: str, sender: str, now: float) -> Message:
    return Message(MessageType.DONE, {"task_id": task_id, "sender": sender, "t": now})


def release_message(task_id: str, sender: str, reason: str, now: float) -> Message:
    return Message(MessageType.RELEASE, {"task_id": task_id, "sender": sender, "reason": reason, "t": now})
