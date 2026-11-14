from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import FR3Env
import pinocchio as pin

if TYPE_CHECKING:
    from collections.abc import Sequence

    from numpy.typing import NDArray

    from DifferentiableOptimizationCBF.manipulator.scene import LinkShape


@dataclass
class FR3KinematicsCfg:
    # None -> the FR3 with bounding-box frames that ships with FR3Env
    urdf_path: str | None = None
    # frame tracked by the performance controller
    end_effector_frame: str = "fr3_hand_bounding_box"


@dataclass
class FrameKinematics:
    """Pose and Jacobian of one frame, expressed in the robot base frame."""

    position: NDArray  # shape (3,)
    rotation: NDArray  # shape (3, 3)
    # [linear; angular] velocity of the frame = jacobian @ dq, in the
    # LOCAL_WORLD_ALIGNED convention (origin at the frame, axes of the base)
    jacobian: NDArray  # shape (6, nv)


@dataclass
class FR3KinematicsData:
    links: tuple[FrameKinematics, ...]  # one per LinkShape, in the same order
    end_effector: FrameKinematics


class FR3Kinematics:
    """Pinocchio model of the FR3: frame poses, Jacobians and gravity torques."""

    def __init__(self, cfg: FR3KinematicsCfg, link_shapes: Sequence[LinkShape]) -> None:
        self.cfg = cfg

        urdf_path = cfg.urdf_path
        if urdf_path is None:
            urdf_path = FR3Env.getDataPath() + "/robots/fr3_with_bounding_boxes.urdf"

        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        self._link_frame_ids = tuple(self._frame_id(shape.frame) for shape in link_shapes)
        self._end_effector_frame_id = self._frame_id(cfg.end_effector_frame)

    @property
    def nq(self) -> int:
        return self.model.nq

    @property
    def nv(self) -> int:
        return self.model.nv

    def __call__(self, q: NDArray) -> FR3KinematicsData:
        pin.computeJointJacobians(self.model, self.data, q)
        pin.framesForwardKinematics(self.model, self.data, q)

        return FR3KinematicsData(
            links=tuple(self._frame_kinematics(frame_id) for frame_id in self._link_frame_ids),
            end_effector=self._frame_kinematics(self._end_effector_frame_id),
        )

    def gravity(self, q: NDArray) -> NDArray:
        """Generalized gravity torques, shape (nv,)."""
        return pin.computeGeneralizedGravity(self.model, self.data, q).copy()

    def _frame_id(self, name: str) -> int:
        if not self.model.existFrame(name):
            raise ValueError(f"Frame {name!r} does not exist in the robot model")
        return self.model.getFrameId(name)

    def _frame_kinematics(self, frame_id: int) -> FrameKinematics:
        pose = self.data.oMf[frame_id]
        return FrameKinematics(
            position=pose.translation.copy(),
            rotation=pose.rotation.copy(),
            jacobian=pin.getFrameJacobian(
                self.model, self.data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            ),
        )
