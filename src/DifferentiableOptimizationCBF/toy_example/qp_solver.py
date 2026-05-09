from __future__ import annotations

import proxsuite

from DifferentiableOptimizationCBF.toy_example.configs import QPProblemCfg, QPSolverCfg


class QPSolver:
    def __init__(self, cfg: QPSolverCfg) -> None:
        self.cfg = cfg
        self.qp = proxsuite.proxqp.dense.QP(self.cfg.n, self.cfg.n_eq, self.cfg.n_ieq)
        self.initialized = False

    def solve(self, problem_cfg: QPProblemCfg) -> None:
        if not self.initialized:
            self.qp.init(H=problem_cfg.H, g=problem_cfg.g, C=problem_cfg.C, l=problem_cfg.lb)
            self.qp.settings.eps_abs = self.cfg.eps_abs
            self.initialized = True
        else:
            self.qp.update(H=problem_cfg.H, g=problem_cfg.g, C=problem_cfg.C, l=problem_cfg.lb)

        self.qp.solve()

        return self.qp.results.x
