from __future__ import annotations

import copy
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
import pinocchio as pin
import proxsuite
import tyro

from DifferentiableOptimizationCBF.toy_example.configs import (
    CBFQPCfg,
    PerformanceControllerCfg,
    PerformanceControllerGoalCfg,
    QPProblemCfg,
    QPSolverCfg,
)
from DifferentiableOptimizationCBF.toy_example.unicycle_env import UnicycleEnv, UnicycleState
from DifferentiableOptimizationCBF.toy_example.unicycle_plot_utils import plot_unicycle

if TYPE_CHECKING:
    from numpy.typing import NDArray

DC_UTILS_DIR = Path(__file__).parent.parent / "dc_utils"


def load_julia_functions() -> tuple[Callable[..., Any], Callable[..., Any]]:
    from juliacall import Main as jl

    return (
        jl.include(str(DC_UTILS_DIR / "unicycle_env_setup.jl")),
        jl.include(str(DC_UTILS_DIR / "get_cbf_unicycle_env.jl")),
    )


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


@dataclass
class Args:
    show_plot: bool = False


class PerformanceController:
    def __init__(self, cfg: PerformanceControllerCfg) -> None:
        self.cfg = cfg

    def __call__(self, state: UnicycleState, goal: PerformanceControllerGoalCfg) -> NDArray:
        v = self.cfg.kv * np.sqrt((goal.x - state.x) ** 2 + (goal.y - state.y) ** 2)
        target_θ = np.arctan2(goal.y - state.y, goal.x - state.x)
        ω = self.cfg.kω * (target_θ - state.theta)

        return np.array([v, ω])


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


def main() -> None:
    args = tyro.cli(Args)

    unicycle_env_setup_jl, get_cbf_unicycle_env_jl = load_julia_functions()
    unicycle_env_setup_jl()

    env = UnicycleEnv()
    init_state = UnicycleState(x=-1.0, y=-3.0, theta=np.pi / 4)
    env.reset(init_state=init_state)

    cbfqp_cfg: CBFQPCfg = CBFQPCfg(β=1.05, γ=1.0)
    performance_controller_cfg: PerformanceControllerCfg = PerformanceControllerCfg(kv=0.5, kω=2.0)
    performance_controller_goal_cfg: PerformanceControllerGoalCfg = PerformanceControllerGoalCfg(
        x=5.0, y=3.0
    )
    qp_solver_cfg: QPSolverCfg = QPSolverCfg(n=2, n_eq=0, n_ieq=2)

    qp_solver = QPSolver(qp_solver_cfg)
    performance_controller = PerformanceController(performance_controller_cfg)

    # store data
    history = []
    comp_times = []

    for i in range(5000):
        nominal_control = performance_controller(env.state, performance_controller_goal_cfg)

        # get CBF
        tic = time.time()

        _robot_q_np = np.array([env.robot_q.w, env.robot_q.x, env.robot_q.y, env.robot_q.z])
        αs, Js = get_cbf_unicycle_env_jl(env.robot_r, _robot_q_np)

        if i >= 10:
            comp_times.append(time.time() - tic)  # account for JIT run

        QF_mat = get_Q_mat(env.robot_q) @ get_F_mat(env.state)

        J1 = np.array(Js[0], copy=True)[-1, 7:][[0, 1, 3, 4, 5, 6]][np.newaxis, :]
        J2 = np.array(Js[1], copy=True)[-1, 7:][[0, 1, 3, 4, 5, 6]][np.newaxis, :]

        C1 = J1 @ QF_mat
        C2 = J2 @ QF_mat

        # define CBFQP
        H = np.eye(2)
        g = -nominal_control[:, np.newaxis]
        C = np.vstack((C1, C2))
        lb = -cbfqp_cfg.γ * np.array([[αs[0] - cbfqp_cfg.β], [αs[1] - cbfqp_cfg.β]])

        problem_cfg = QPProblemCfg(H=H, g=g, C=C, lb=lb)

        # Get safe action
        safe_control = qp_solver.solve(problem_cfg)
        safe_control = np.clip(safe_control, -20.0, 20.0)

        # apply safe action
        env.step(safe_control)

        # store data
        history.append(copy.deepcopy(env.state.array))

    print("Average computation time: ", np.mean(comp_times))

    if args.show_plot:
        plot_unicycle(history)


if __name__ == "__main__":
    main()
