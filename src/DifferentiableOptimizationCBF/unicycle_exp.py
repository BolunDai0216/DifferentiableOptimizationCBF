import copy
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import proxsuite
import tyro

from DifferentiableOptimizationCBF.envs import UnicycleEnv
from DifferentiableOptimizationCBF.unicycle_plot_utils import plot_unicycle

DC_UTILS_DIR = Path(__file__).parent / "dc_utils"


def load_julia_functions():
    from juliacall import Main as jl

    return (
        jl.include(str(DC_UTILS_DIR / "unicycle_env_setup.jl")),
        jl.include(str(DC_UTILS_DIR / "get_cbf_unicycle_env.jl")),
    )


def get_Q_mat(q):
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, -q[3]],
            [0.0, 0.0, q[2]],
            [0.0, 0.0, -q[1]],
            [0.0, 0.0, q[0]],
        ]
    )


@dataclass
class Args:
    show_plot: bool = False


@dataclass
class QPSolverConfig:
    n: int
    n_eq: int
    n_ieq: int
    eps_abs: float = 1.0e-6


@dataclass
class QPProblem:
    H: np.ndarray
    g: np.ndarray
    C: np.ndarray
    lb: np.ndarray


class QPSolver:
    def __init__(self, cfg: QPSolverConfig) -> None:
        self.cfg = cfg
        self.qp = proxsuite.proxqp.dense.QP(self.cfg.n, self.cfg.n_eq, self.cfg.n_ieq)
        self.initialized = False

    def solve(self, problem: QPProblem) -> None:
        if not self.initialized:
            self.qp.init(H=problem.H, g=problem.g, C=problem.C, l=problem.lb)
            self.qp.settings.eps_abs = self.cfg.eps_abs
            self.initialized = True
        else:
            self.qp.update(H=problem.H, g=problem.g, C=problem.C, l=problem.lb)

        self.qp.solve()

        return self.qp.results.x


def main():
    args = tyro.cli(Args)

    unicycle_env_setup, get_cbf_unicycle_env = load_julia_functions()

    env = UnicycleEnv()
    env.reset(set_init_state=[-1.0, -3.0, np.pi / 4])
    unicycle_env_setup()

    qp_solver_cfg = QPSolverConfig(n=2, n_eq=0, n_ieq=2)
    qp_solver = QPSolver(qp_solver_cfg)

    # task parameters
    target_x = 5.0
    target_y = 3.0
    kv = 0.5
    kω = 2.0
    β = 1.05
    γ = 1.0

    # store data
    history = []
    comp_times = []

    for i in range(5000):
        # compute performance controller
        v = kv * np.sqrt((target_x - env.state[0]) ** 2 + (target_y - env.state[1]) ** 2)

        target_θ = np.arctan2(target_y - env.state[1], target_x - env.state[0])

        ω = kω * (target_θ - env.state[2])
        control = np.array([v, ω])

        # get CBF
        tic = time.time()
        αs, Js = get_cbf_unicycle_env(env.robot_r, env.robot_q)

        if i >= 10:
            # account for JIT run
            comp_times.append(time.time() - tic)

        Q_mat = get_Q_mat(env.robot_q)
        QF_mat = Q_mat @ env.F_mat

        J1 = np.array(Js[0])[-1, 7:][[0, 1, 3, 4, 5, 6]][np.newaxis, :]
        J2 = np.array(Js[1])[-1, 7:][[0, 1, 3, 4, 5, 6]][np.newaxis, :]

        C1 = J1 @ QF_mat
        C2 = J2 @ QF_mat

        C = np.vstack((C1, C2))
        lb = -γ * np.array([[αs[0] - β], [αs[1] - β]])

        # define CBFQP
        H = np.eye(2)
        g = -control[:, np.newaxis]

        problem = QPProblem(H=H, g=g, C=C, lb=lb)

        # Get safe action
        safe_control = qp_solver.solve(problem)
        safe_control = np.clip(safe_control, -20.0, 20.0)

        # apply safe action
        env.step(safe_control)

        # store data
        history.append(copy.deepcopy(env.state))

    print("Average computation time: ", np.mean(np.array(comp_times)))

    if args.show_plot:
        plot_unicycle(history)


if __name__ == "__main__":
    main()
