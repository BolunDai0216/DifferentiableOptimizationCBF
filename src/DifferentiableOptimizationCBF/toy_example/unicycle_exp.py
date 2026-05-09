from __future__ import annotations

import copy
import logging
import time
from dataclasses import dataclass

import numpy as np
import tyro

from DifferentiableOptimizationCBF.toy_example.configs import (
    CBFQPCfg,
    PerformanceControllerCfg,
    PerformanceControllerGoalCfg,
    QPProblemCfg,
    QPSolverCfg,
)
from DifferentiableOptimizationCBF.toy_example.julia_interop import load_julia_functions
from DifferentiableOptimizationCBF.toy_example.performance_controller import PerformanceController
from DifferentiableOptimizationCBF.toy_example.qp_solver import QPSolver
from DifferentiableOptimizationCBF.toy_example.unicycle_dynamics import get_F_mat, get_Q_mat
from DifferentiableOptimizationCBF.toy_example.unicycle_env import UnicycleEnv, UnicycleState
from DifferentiableOptimizationCBF.toy_example.unicycle_plot_utils import plot_unicycle

logger = logging.getLogger(__name__)


@dataclass
class Args:
    show_plot: bool = False
    control_magnitude: float = 20.0
    n_steps: int = 5000


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    args = tyro.cli(Args)

    # Define configs
    cbfqp_cfg: CBFQPCfg = CBFQPCfg(β=1.05, γ=1.0)
    performance_controller_cfg: PerformanceControllerCfg = PerformanceControllerCfg(kv=0.5, kω=2.0)
    performance_controller_goal_cfg: PerformanceControllerGoalCfg = PerformanceControllerGoalCfg(
        x=5.0, y=3.0
    )
    qp_solver_cfg: QPSolverCfg = QPSolverCfg(n=2, n_eq=0, n_ieq=2)

    # Setup Julia functions
    unicycle_env_setup_jl, get_cbf_unicycle_env_jl = load_julia_functions()
    unicycle_env_setup_jl()

    # Setup environment
    env = UnicycleEnv()
    init_state = UnicycleState(x=-1.0, y=-3.0, theta=np.pi / 4)
    env.reset(init_state=init_state)

    # Setup controllers
    cbfqp_solver = QPSolver(qp_solver_cfg)
    performance_controller = PerformanceController(performance_controller_cfg)

    # store data
    history = []
    comp_times = []

    for i in range(args.n_steps):
        nominal_control = performance_controller(env.state, performance_controller_goal_cfg)

        # get CBF
        tic = time.time()

        _robot_q_np = np.array([env.robot_q.w, env.robot_q.x, env.robot_q.y, env.robot_q.z])
        αs, Js = get_cbf_unicycle_env_jl(env.robot_r, _robot_q_np)

        if i >= 10:
            comp_times.append(time.time() - tic)  # account for JIT run

        J = np.vstack(
            (
                np.array(Js[0][-1:, 7:], copy=True)[:, [0, 1, 3, 4, 5, 6]],
                np.array(Js[1][-1:, 7:], copy=True)[:, [0, 1, 3, 4, 5, 6]],
            )
        )

        # define CBFQP
        H = np.eye(2)
        g = -nominal_control[:, np.newaxis]
        C = J @ get_Q_mat(env.robot_q) @ get_F_mat(env.state)
        lb = -cbfqp_cfg.γ * (np.array(αs, copy=True) - cbfqp_cfg.β)[:, np.newaxis]

        # Get safe action
        safe_control = cbfqp_solver.solve(QPProblemCfg(H=H, g=g, C=C, lb=lb)).clip(
            -args.control_magnitude, args.control_magnitude
        )

        # apply safe action
        env.step(safe_control)

        # store data
        history.append(copy.deepcopy(env.state.array))

    logger.info("Average computation time: %.3e", np.mean(comp_times))

    if args.show_plot:
        plot_unicycle(history)


if __name__ == "__main__":
    main()
