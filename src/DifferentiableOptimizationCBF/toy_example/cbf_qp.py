from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from DifferentiableOptimizationCBF.toy_example.qp_solver import QPProblem
from DifferentiableOptimizationCBF.toy_example.unicycle_dynamics import (
    get_F_mat,
    get_Q_mat,
    quat_from_yaw,
)

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from DifferentiableOptimizationCBF.toy_example.unicycle_env import UnicycleEnv


@dataclass
class CBFQPCfg:
    β: float = 1.05
    γ: float = 1.0


class UnicycleCBFQPBuilder:
    """Assembles the CBF-QP for the unicycle env.

    Given a nominal control, CBF values αs and Jacobian J, and the current env,
    builds the QP that minimizes deviation from the nominal subject to the CBF
    safety constraints.
    """

    def __init__(self, cfg: CBFQPCfg) -> None:
        self.cfg = cfg

    def __call__(
        self,
        nominal_control: NDArray,
        αs: NDArray,
        J: NDArray,
        env: UnicycleEnv,
    ) -> QPProblem:
        H = np.eye(2)
        g = -nominal_control[:, np.newaxis]
        C = J @ get_Q_mat(quat_from_yaw(env.state.yaw)) @ get_F_mat(env.state)
        lb = -self.cfg.γ * (αs - self.cfg.β)[:, np.newaxis]
        return QPProblem(H=H, g=g, C=C, lb=lb)
