"""QP data types and the proxsuite-backed solver shared by all examples."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import proxsuite

if TYPE_CHECKING:
    from numpy.typing import NDArray


@dataclass
class QPSolverCfg:
    n: int
    n_eq: int
    n_ieq: int
    eps_abs: float = 1.0e-6


@dataclass
class QPObjective:
    """The objective 1/2 x^T H x + g^T x."""

    H: NDArray
    g: NDArray


@dataclass
class QPProblem:
    """min 1/2 x^T H x + g^T x, subject to lb <= C x."""

    H: NDArray
    g: NDArray
    C: NDArray
    lb: NDArray


class QPSolver:
    def __init__(self, cfg: QPSolverCfg) -> None:
        self.cfg = cfg
        self.qp = proxsuite.proxqp.dense.QP(self.cfg.n, self.cfg.n_eq, self.cfg.n_ieq)
        self.initialized = False

    def solve(self, problem: QPProblem) -> NDArray:
        if not self.initialized:
            self.qp.init(H=problem.H, g=problem.g, C=problem.C, l=problem.lb)
            self.qp.settings.eps_abs = self.cfg.eps_abs
            self.initialized = True
        else:
            self.qp.update(H=problem.H, g=problem.g, C=problem.C, l=problem.lb)

        self.qp.solve()

        return self.qp.results.x
