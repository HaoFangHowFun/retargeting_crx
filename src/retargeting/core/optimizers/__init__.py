"""Retargeting objective implementations and their optimizer registry."""

from retargeting.core.optimizers.base import RetargetOptimizer, extract_solver_params
from retargeting.core.optimizers.registry import OPTIMIZER_CLASSES, get_optimizer_class
from retargeting.core.optimizers.vector_wrist_joint import VectorWristJointOptimizer, VectorWristJointOptimizerV2

__all__ = [
    "OPTIMIZER_CLASSES",
    "DexPilotOptimizer",
    "extract_solver_params",
    "RetargetOptimizer",
    "VectorWristJointOptimizer",
    "VectorWristJointOptimizerV2",
    "get_optimizer_class",
]


def __getattr__(name: str):
    """Load the optional Torch-based DexPilot optimizer only when requested."""
    if name == "DexPilotOptimizer":
        from retargeting.core.optimizers.dexpilot import DexPilotOptimizer

        return DexPilotOptimizer
    raise AttributeError(name)
