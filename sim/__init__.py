"""Fast deterministic Tier-0 simulation."""

from .config import SimulationConfig
from .world import SimulationResult, World, run_simulation

__all__ = ["SimulationConfig", "SimulationResult", "World", "run_simulation"]
