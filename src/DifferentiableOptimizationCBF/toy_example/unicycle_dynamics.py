from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import pinocchio as pin

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from DifferentiableOptimizationCBF.toy_example.unicycle_env import UnicycleState


def get_Q_mat(quat: pin.Quaternion) -> NDArray:
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, -quat.z],
            [0.0, 0.0, quat.y],
            [0.0, 0.0, -quat.x],
            [0.0, 0.0, quat.w],
        ]
    )


def get_F_mat(state: UnicycleState) -> NDArray:
    return np.array([[np.cos(state.theta), 0.0], [np.sin(state.theta), 0.0], [0.0, 1.0]])
