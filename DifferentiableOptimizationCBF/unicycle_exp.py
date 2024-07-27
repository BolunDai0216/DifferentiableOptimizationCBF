import argparse
import copy
import time
from sys import platform

import numpy as np
import proxsuite

from DifferentiableOptimizationCBF.envs.unicycle_env import UnicycleEnv
from DifferentiableOptimizationCBF.unicycle_plot_utils import plot_unicycle


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--show_plot", action="store_true")
    args = parser.parse_args()

    env = UnicycleEnv()
    env.reset(set_init_state=[-1.0, -3.0, np.pi / 4])
    unicycle_env_setup()
    initialized = False

    # define CBFQP solver
    qp = proxsuite.proxqp.dense.QP(2, 0, 2)

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
        v = kv * np.sqrt(
            (target_x - env.state[0]) ** 2 + (target_y - env.state[1]) ** 2
        )

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

        # solve CBFQP
        if initialized:
            qp.update(H=H, g=g, C=C, l=lb)
        else:
            qp.init(H=H, g=g, C=C, l=lb)
            initialized = True

        qp.solve()

        # Get safe action
        safe_control = qp.results.x
        safe_control = np.clip(safe_control, -20.0, 20.0)

        # apply safe action
        env.step(safe_control)

        # store data
        history.append(copy.deepcopy(env.state))

    print("Average computation time: ", np.mean(np.array(comp_times)))

    if args.show_plot:
        plot_unicycle(history)


if __name__ == "__main__":
    if platform == "darwin":
        from julia.api import Julia

        jl = Julia(compiled_modules=False)

    import julia

    j = julia.Julia()
    unicycle_env_setup = j.include("dc_utils/unicycle_env_setup.jl")
    get_cbf_unicycle_env = j.include("dc_utils/get_cbf_unicycle_env.jl")

    main()
