from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from DifferentiableOptimizationCBF.toy_example.unicycle_env import UnicycleState

if TYPE_CHECKING:
    from numpy.typing import NDArray


@dataclass
class PerformanceControllerCfg:
    kv: float = 0.5
    kω: float = 2.0


@dataclass
class PerformanceControllerGoalCfg:
    x: float
    y: float


class PerformanceController:
    def __init__(self, cfg: PerformanceControllerCfg) -> None:
        self.cfg = cfg

    def __call__(self, state: UnicycleState, goal: PerformanceControllerGoalCfg) -> NDArray:
        v = self.cfg.kv * np.sqrt((goal.x - state.x) ** 2 + (goal.y - state.y) ** 2)
        target_θ = np.arctan2(goal.y - state.y, goal.x - state.x)
        ω = self.cfg.kω * (target_θ - state.theta)

        return np.array([v, ω])
