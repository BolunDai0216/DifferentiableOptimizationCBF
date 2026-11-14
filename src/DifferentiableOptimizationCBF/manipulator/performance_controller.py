from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np
from scipy.spatial.transform import Rotation

from DifferentiableOptimizationCBF.manipulator.scene import FR3_Q_NOMINAL
from DifferentiableOptimizationCBF.qp_solver import QPObjective

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from DifferentiableOptimizationCBF.manipulator.kinematics import FrameKinematics
    from DifferentiableOptimizationCBF.manipulator.scene import FR3State, Scene


@dataclass
class PerformanceControllerCfg:
    # task-space proportional gain
    kp: float = 1.0
    # joint-centering gain, acts in the nullspace of the end-effector task
    k_nullspace: float = 0.5
    q_nominal: NDArray = field(default_factory=lambda: FR3_Q_NOMINAL.copy())


@dataclass
class EndEffectorGoalCfg:
    # goal position in the base frame [m]
    position: tuple[float, float, float]
    # goal orientation as roll/pitch/yaw [deg], relative to the orientation
    # of the end-effector the first time the controller is called
    relative_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @classmethod
    def from_scene(cls, scene: Scene) -> EndEffectorGoalCfg:
        return cls(position=scene.target_position, relative_rpy=scene.target_relative_rpy)


class PerformanceController:
    """Nominal task: reach an end-effector pose, center the joints in the nullspace.

    Instead of a control it returns the objective of a QP over the joint velocities dq,

        ||J dq - kp * pose_error||^2 + ||N (dq - dq_nominal)||^2,

    with J the end-effector Jacobian, N = I - pinv(J) J its nullspace projector
    and dq_nominal = k_nullspace * (q_nominal - q), so that the CBF constraints
    can be added on top of it.
    """

    def __init__(self, cfg: PerformanceControllerCfg) -> None:
        self.cfg = cfg
        self._R_start: NDArray | None = None

    def reset(self) -> None:
        """Forget the start orientation, the next call latches a new one."""
        self._R_start = None

    def __call__(
        self, state: FR3State, end_effector: FrameKinematics, goal: EndEffectorGoalCfg
    ) -> QPObjective:
        if self._R_start is None:
            self._R_start = end_effector.rotation.copy()

        # end-effector pose error, the orientation part in axis-angle form
        R_error = self._goal_rotation(goal) @ end_effector.rotation.T
        pose_error = np.zeros((6, 1))
        pose_error[:3, 0] = np.asarray(goal.position, dtype=float) - end_effector.position
        pose_error[3:, 0] = Rotation.from_matrix(R_error).as_rotvec()

        # target end-effector twist
        a = self.cfg.kp * pose_error

        # joint centering
        dq_nominal = self.cfg.k_nullspace * (self.cfg.q_nominal - state.q)[:, np.newaxis]

        J = end_effector.jacobian
        N = np.eye(J.shape[1]) - np.linalg.pinv(J) @ J

        H = 2 * J.T @ J + 2 * N.T @ N
        g = -2 * (a.T @ J + dq_nominal.T @ N.T @ N).T

        return QPObjective(H=H, g=g)

    def _goal_rotation(self, goal: EndEffectorGoalCfg) -> NDArray:
        roll, pitch, yaw = goal.relative_rpy
        R_goal = (
            Rotation.from_euler("x", roll, degrees=True).as_matrix()
            @ Rotation.from_euler("y", pitch, degrees=True).as_matrix()
            @ Rotation.from_euler("z", yaw, degrees=True).as_matrix()
            @ self._R_start
        )
        return Rotation.from_matrix(R_goal).as_matrix()
