from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Literal

import numpy as np
import tyro

from DifferentiableOptimizationCBF.manipulator.controller import (
    DiffOptCBFController,
    DiffOptCBFControllerCfg,
)
from DifferentiableOptimizationCBF.manipulator.mujoco_env import FR3MuJoCoEnv, FR3MuJoCoEnvCfg
from DifferentiableOptimizationCBF.manipulator.scene import SCENES
from DifferentiableOptimizationCBF.manipulator.torque_law import JointTorqueLaw, JointTorqueLawCfg

logger = logging.getLogger(__name__)

SceneName = Literal["three-blocks", "two-walls"]


@dataclass
class Args:
    scene: SceneName = "three-blocks"
    headless: bool = False
    n_steps: int = 30000
    # number of simulation steps per update of the joint velocity targets
    control_interval: int = 10
    # slow-motion factor of the viewer (1.0 = real time), ignored when headless
    slowdown: float = 1.0


def main(default_scene: SceneName = "three-blocks") -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    args = tyro.cli(Args, default=Args(scene=default_scene))
    scene = SCENES[args.scene]

    # Define configs
    env_cfg: FR3MuJoCoEnvCfg = FR3MuJoCoEnvCfg(
        dt=1e-3, render=not args.headless, slowdown=args.slowdown
    )
    controller_cfg: DiffOptCBFControllerCfg = DiffOptCBFControllerCfg()
    torque_law_cfg: JointTorqueLawCfg = JointTorqueLawCfg(kp=30.0, kd=3.0, damping=0.1)

    # Setup environment
    env = FR3MuJoCoEnv(env_cfg, scene)
    state = env.reset()

    # Setup CBF controller + torque law
    controller = DiffOptCBFController(controller_cfg, scene)
    torque_law = JointTorqueLaw(torque_law_cfg)

    # store data
    comp_times = []
    min_α = np.inf

    for i in range(args.n_steps):
        if not env.is_running():
            break

        # get safe joint velocity targets
        if i % args.control_interval == 0:
            dq_target, info = controller(state)

            min_α = min(min_α, info.αs.min())
            if i >= 10 * args.control_interval:
                comp_times.append(info.cbf_time)  # account for JIT run

        # apply joint torques
        τ = torque_law(state, dq_target, env.bias_forces(), env.dt)
        state = env.step(τ)

    had_obstacle_contact = env.had_obstacle_contact
    env.close()

    if comp_times:
        logger.info("Average CBF computation time: %.3e", np.mean(comp_times))
    logger.info("Minimum α: %.4f (bounding shapes clear of the obstacles iff > 1)", min_α)
    position_error = np.linalg.norm(
        np.array(scene.target_position) - info.kinematics.end_effector.position
    )
    logger.info("Final end-effector position error: %.4f m", position_error)
    logger.info("Robot-obstacle contact during the run: %s", had_obstacle_contact)


def three_blocks_main() -> None:
    main(default_scene="three-blocks")


def two_walls_main() -> None:
    main(default_scene="two-walls")


if __name__ == "__main__":
    main()
