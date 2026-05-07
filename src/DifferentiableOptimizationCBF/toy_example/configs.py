from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from numpy.typing import NDArray


@dataclass
class QPSolverCfg:
    n: int
    n_eq: int
    n_ieq: int
    eps_abs: float = 1.0e-6


@dataclass
class QPProblemCfg:
    H: NDArray
    g: NDArray
    C: NDArray
    lb: NDArray


@dataclass
class PerformanceControllerCfg:
    kv: float = 0.5
    kω: float = 2.0


@dataclass
class PerformanceControllerGoalCfg:
    x: float
    y: float


@dataclass
class CBFQPCfg:
    β: float = 1.05
    γ: float = 1.0
