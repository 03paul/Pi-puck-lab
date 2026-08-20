"""Python 3.7 compatibility shim for the Tier-2 pilot. The Raspberry Pi
Zero 2 W lab image ships Python 3.7.3; this project's `allocation/`, `sim/`,
`metrics/`, and `backends/` modules assume 3.8+ (`math.dist`) and 3.10+
(`dataclasses.dataclass(slots=True)`).

Import this BEFORE any `allocation`/`sim`/`metrics`/`backends` import - both
patches below only take effect if the running interpreter actually lacks
the feature, and they change ZERO observable behaviour:

- `math.dist(p, q)` (added 3.8) is re-implemented from `math.sqrt` -
  identical IEEE-754 result for the same inputs, same formula the real
  implementation uses.
- `slots=True` (added to `dataclasses.dataclass` in 3.10) is purely a
  memory-layout optimisation (faster attribute access, no `__dict__` per
  instance) - it has no effect on equality, hashing, repr, or any decision
  logic. Every dataclass in this project's codebase uses it only for that
  optimisation, so stripping the keyword on old Python is behaviourally a
  no-op, not a logic change.

This file deliberately does NOT touch a single line inside `allocation/`,
`sim/`, `metrics/`, or `backends/base.py` - see docs/WEBOTS_MARKET_DEMO.md
and the project's report for why keeping that package's source untouched
across tiers matters. The patch lives entirely in this Tier-2-only,
docs/-local entry-point shim, on the same principle as the
`dataclasses.replace()` calibration overrides already used in
docs/webots_market_*.py and docs/pipuck_*.py.

No-op (and safe to import unconditionally) on any Python that already has
both features - e.g. the dev machine, or Webots' own Python.
"""

from __future__ import annotations

import dataclasses
import math
import sys

if not hasattr(math, "dist"):

    def _dist(p, q):
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(p, q)))

    math.dist = _dist  # type: ignore[attr-defined]

if sys.version_info < (3, 10):
    _original_dataclass = dataclasses.dataclass

    def _dataclass_without_slots(*args, **kwargs):
        kwargs.pop("slots", None)
        return _original_dataclass(*args, **kwargs)

    dataclasses.dataclass = _dataclass_without_slots  # type: ignore[assignment]
