from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from scipy.spatial.transform import Rotation

from DifferentiableOptimizationCBF.qp_solver import QPProblem

if TYPE_CHECKING:
    from collections.abc import Sequence

    from numpy.typing import NDArray

    from DifferentiableOptimizationCBF.manipulator.julia_interop import CBFEvaluation
    from DifferentiableOptimizationCBF.manipulator.kinematics import FrameKinematics
    from DifferentiableOptimizationCBF.qp_solver import QPObjective


@dataclass
class CBFQPCfg:
    # the CBF is h = α - β, the constraint is dh/dt >= -γ h
    β: float = 1.03
    γ: float = 5.0
    # How the angular velocity of a link is mapped to the rate of the quaternion
    # that ∂α/∂quat is taken with respect to.
    #   True:  the exact mapping, see `get_Q_mat_exact`. With it the constraint
    #          rows are the true dα/dθ (they match finite differences of α).
    #   False: the mapping of the original implementation (the simulations of
    #          the paper), see `get_Q_mat_legacy`. Only exact for spheres; kept
    #          to reproduce the original results.
    exact_quaternion_rate: bool = True


def get_Q_mat_legacy(frame_rotation: NDArray) -> NDArray:
    """Quaternion-rate matrix of the original implementation, shape (4, 3).

    Built from the quaternion of the link *frame* (not of the primitive handed
    to Julia) and with the body-frame formula quat_dot = 1/2 quat ⊗ ω, although
    ω is expressed in the base frame.
    """
    qx, qy, qz, qw = Rotation.from_matrix(frame_rotation).as_quat()
    return np.array([[-qx, -qy, -qz], [qw, -qz, qy], [qz, qw, -qx], [-qy, qx, qw]])


def get_Q_mat_exact(primitive_quat: NDArray) -> NDArray:
    """Matrix Q with quat_dot = 1/2 Q ω, shape (4, 3).

    ``primitive_quat`` = [qw, qx, qy, qz] is the quaternion handed to Julia and
    ω the angular velocity expressed in the base frame: quat_dot = 1/2 ω ⊗ quat.
    """
    qw, qx, qy, qz = primitive_quat
    return np.array([[-qx, -qy, -qz], [qw, qz, -qy], [-qz, qw, qx], [qy, -qx, qw]])


class ManipulatorCBFQPBuilder:
    """Assembles the CBF-QP of the manipulator.

    Given the objective of the performance controller, the CBF values and
    Jacobians, and the kinematics of the links, builds the QP over the joint
    velocities with one CBF constraint per (link, obstacle) pair. The rows are
    ordered link by link: row = link_index * n_obstacles + obstacle_index.
    """

    def __init__(self, cfg: CBFQPCfg) -> None:
        self.cfg = cfg

    def __call__(
        self,
        objective: QPObjective,
        cbf: CBFEvaluation,
        links: Sequence[FrameKinematics],
    ) -> QPProblem:
        n_obstacles, n_links = cbf.αs.shape
        if len(links) != n_links:
            raise ValueError(f"Got CBFs for {n_links} links but kinematics for {len(links)}")

        Cs = []
        for k, link in enumerate(links):
            if self.cfg.exact_quaternion_rate:
                Q_mat = get_Q_mat_exact(cbf.link_quats[k])
            else:
                Q_mat = get_Q_mat_legacy(link.rotation)

            # [v; ω] -> d(r, quat)/dt
            pose_rate_from_twist = np.zeros((7, 6))
            pose_rate_from_twist[:3, :3] = np.eye(3)
            pose_rate_from_twist[3:, 3:] = 0.5 * Q_mat

            Cs.append(cbf.Js[:, k, :] @ pose_rate_from_twist @ link.jacobian)

        C = np.concatenate(Cs, axis=0)
        lb = -self.cfg.γ * (cbf.αs.T.reshape(-1, 1) - self.cfg.β)

        return QPProblem(H=objective.H, g=objective.g, C=C, lb=lb)
