import time
from sys import platform

import numpy as np


def main():
    # create environment
    env = TwoWallsEnv(
        render_mode="human",
        record_path=None,
        crude_model=False,
    )

    # define solver
    controller = TwoWallsController()

    # reset environment
    info = env.reset(
        cameraDistance=2.0, cameraYaw=-1e-3, cameraPitch=-1e-3, lookat=[0.70, 0.0, 0.55]
    )

    # initialize clock
    t = 0.0

    # create list to store data
    history = []
    torques = []

    # how many iterations per update velocity control command
    control_interval = 10

    computation_times = []

    for i in range(30000):
        t += env.dt

        # get data from info
        q = info["q"]
        dq = info["dq"]
        G = info["G"][:, np.newaxis]

        tic = time.time()
        if i % control_interval == 0:
            dq_target, _info = controller.controller(t, q, dq)

            # store data for plotting
            _info["dq_target"] = dq_target
            history.append(_info)

        # compute torque command
        τ = (
            6.0 * (dq_target[:, np.newaxis] - dq[:, np.newaxis])
            + G
            - 0.1 * dq[:, np.newaxis]
        )
        torques.append(τ)

        if i >= 1:
            computation_times.append(time.time() - tic)

        # send joint commands to motor
        info = env.step(τ)


if __name__ == "__main__":
    if platform == "darwin":
        from julia.api import Julia

        jl = Julia(compiled_modules=False)

    from DifferentiableOptimizationCBF.envs.two_walls_env import TwoWallsEnv
    from DifferentiableOptimizationCBF.two_walls_controller import TwoWallsController

    main()
