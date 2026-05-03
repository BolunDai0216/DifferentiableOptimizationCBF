import time

import numpy as np


def main():
    # create environment
    env = ThreeBlocksEnv(
        render_mode="human",
        record_path=None,
        crude_type="ellipsoid",
    )

    controller = ThreeBlocksController()

    # reset environment
    info = env.reset(
        cameraDistance=2.0, cameraYaw=-1e-3, cameraPitch=-1e-3, lookat=[0.45, 0.0, 0.55]
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
        τ = 6.0 * (dq_target[:, np.newaxis] - dq[:, np.newaxis]) + G - 0.1 * dq[:, np.newaxis]
        torques.append(τ)

        if i >= 1:
            computation_times.append(time.time() - tic)

        # send joint commands to motor
        info = env.step(τ)


if __name__ == "__main__":
    from DifferentiableOptimizationCBF.envs.three_blocks_env import ThreeBlocksEnv
    from DifferentiableOptimizationCBF.three_blocks_controller import (
        ThreeBlocksController,
    )

    main()
