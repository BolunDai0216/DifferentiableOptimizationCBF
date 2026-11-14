from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from DifferentiableOptimizationCBF.manipulator.scene import FR3State


@dataclass
class JointTorqueLawCfg:
    kp: float = 30.0
    kd: float = 3.0
    damping: float = 0.1


class JointTorqueLaw:
    """Joint-space PD law that tracks a joint velocity target, with gravity compensation."""

    def __init__(self, cfg: JointTorqueLawCfg) -> None:
        self.cfg = cfg

    def __call__(self, state: FR3State, dq_target: NDArray, gravity: NDArray, dt: float) -> NDArray:
        """
        Args:
            dq_target: target joint velocities, shape (nv,)
            gravity: generalized gravity torques at state.q, shape (nv,)
            dt: horizon over which dq_target is integrated to get the position target [s]
        """
        q_target = state.q + dq_target * dt

        return (
            self.cfg.kp * (q_target - state.q)
            + self.cfg.kd * (dq_target - state.dq)
            + gravity
            - self.cfg.damping * state.dq
        )
