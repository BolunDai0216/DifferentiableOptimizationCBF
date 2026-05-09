from __future__ import annotations

import copy
import logging
import time
from dataclasses import dataclass

import numpy as np
import tyro

from DifferentiableOptimizationCBF.toy_example.cbf_qp import CBFQPCfg, UnicycleCBFQPBuilder
from DifferentiableOptimizationCBF.toy_example.julia_interop import UnicycleCBFEvaluator
from DifferentiableOptimizationCBF.toy_example.performance_controller import (
    PerformanceController,
    PerformanceControllerCfg,
    PerformanceControllerGoalCfg,
)
from DifferentiableOptimizationCBF.toy_example.qp_solver import QPSolver, QPSolverCfg
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

    # Setup environment
    env = UnicycleEnv()
    init_state = UnicycleState(x=-1.0, y=-3.0, theta=np.pi / 4)
    env.reset(init_state=init_state)

    # Setup CBF + controllers
    cbf_evaluator = UnicycleCBFEvaluator()
    cbfqp_builder = UnicycleCBFQPBuilder(cbfqp_cfg)
    cbfqp_solver = QPSolver(qp_solver_cfg)
    performance_controller = PerformanceController(performance_controller_cfg)

    # store data
    history = []
    comp_times = []

    for i in range(args.n_steps):
        nominal_control = performance_controller(env.state, performance_controller_goal_cfg)

        # get CBF
        tic = time.time()
        αs, J = cbf_evaluator(env.robot_r, env.robot_q)
        if i >= 10:
            comp_times.append(time.time() - tic)  # account for JIT run

        # define CBFQP and get safe action
        problem = cbfqp_builder(nominal_control, αs, J, env)
        safe_control = cbfqp_solver.solve(problem).clip(
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
