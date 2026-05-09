from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from DifferentiableOptimizationCBF.dc_utils import include_jl
from DifferentiableOptimizationCBF.toy_example.unicycle_dynamics import quat_from_yaw

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from DifferentiableOptimizationCBF.toy_example.unicycle_env import UnicycleState


class UnicycleCBFEvaluator:
    """Adapter around the Julia CBF function for the unicycle env.

    Loads the Julia runtime, runs the env setup on construction, and exposes a
    Python-friendly callable that returns clean numpy arrays.
    """

    def __init__(self) -> None:
        unicycle_env_setup_jl = include_jl("unicycle_env_setup.jl")
        self._get_cbf_jl = include_jl("get_cbf_unicycle_env.jl")
        unicycle_env_setup_jl()

    def __call__(self, state: UnicycleState) -> tuple[NDArray, NDArray]:
        robot_r = np.array([state.x, state.y, 0.0])
        robot_q = quat_from_yaw(state.yaw)
        robot_q_np = np.array([robot_q.w, robot_q.x, robot_q.y, robot_q.z])

        αs, Js = self._get_cbf_jl(robot_r, robot_q_np)

        αs = np.array(αs, copy=True)
        J = np.vstack(
            (
                np.array(Js[0][-1:, 7:], copy=True)[:, [0, 1, 3, 4, 5, 6]],
                np.array(Js[1][-1:, 7:], copy=True)[:, [0, 1, 3, 4, 5, 6]],
            )
        )
        return αs, J
