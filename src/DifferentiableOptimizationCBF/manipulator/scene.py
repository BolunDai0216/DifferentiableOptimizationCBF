"""Single source of truth for what the manipulator CBF controller and the simulator share.

Everything here is plain data: the robot state, the bounding primitives wrapped
around the robot links, the obstacles, and the task. The Julia side
(DifferentiableCollisions.jl), the CBF-QP and the MuJoCo scene are all built
from these objects, so a new task only needs a new ``Scene``.

All positions are expressed in the robot base frame (``fr3_link0``), which is
also the world frame of the simulation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Literal

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

CrudeType = Literal["ellipsoid", "capsule"]


@dataclass
class FR3State:
    """Joint state of the FR3 with gripper: 7 arm joints + 2 finger joints."""

    q: NDArray  # shape (9,)
    dq: NDArray  # shape (9,)


@dataclass(frozen=True)
class BoxObstacle:
    """Axis-aligned box, given by its center and half extents [m]."""

    name: str
    position: tuple[float, float, float]
    half_extents: tuple[float, float, float]


@dataclass(frozen=True)
class LinkShape:
    """Bounding primitive attached to a frame of the robot URDF.

    The primitive's long axis is the z-axis of ``frame`` (the Julia primitives
    use the x-axis, the conversion is handled by the CBF evaluator).

    - ``ellipsoid``: semi-axes ``(long, short, short)`` [m]
    - ``capsule``: ``(radius, length)`` [m]
    - ``sphere``: ``(radius,)`` [m]
    """

    name: str
    frame: str
    kind: Literal["ellipsoid", "capsule", "sphere"]
    size: tuple[float, ...]


_HAND = LinkShape("HAND", "fr3_hand_bounding_box", "sphere", (0.12,))

FR3_ELLIPSOID_LINKS: tuple[LinkShape, ...] = (
    LinkShape("LINK3", "fr3_link3_bounding_box", "ellipsoid", (0.165, 0.09, 0.09)),
    LinkShape("LINK4", "fr3_link4_bounding_box", "ellipsoid", (0.15, 0.09, 0.09)),
    LinkShape("LINK5_1", "fr3_link5_1_bounding_box", "ellipsoid", (0.14, 0.09, 0.09)),
    LinkShape("LINK5_2", "fr3_link5_2_bounding_box", "ellipsoid", (0.125, 0.055, 0.055)),
    LinkShape("LINK6", "fr3_link6_bounding_box", "ellipsoid", (0.11, 0.08, 0.08)),
    LinkShape("LINK7", "fr3_link7_bounding_box", "ellipsoid", (0.14, 0.07, 0.07)),
    _HAND,
)

FR3_CAPSULE_LINKS: tuple[LinkShape, ...] = (
    LinkShape("LINK3", "fr3_link3_bounding_box", "capsule", (0.09, 0.15)),
    LinkShape("LINK4", "fr3_link4_bounding_box", "capsule", (0.09, 0.12)),
    LinkShape("LINK5_1", "fr3_link5_1_bounding_box", "capsule", (0.09, 0.10)),
    LinkShape("LINK5_2", "fr3_link5_2_bounding_box", "capsule", (0.055, 0.14)),
    LinkShape("LINK6", "fr3_link6_bounding_box", "capsule", (0.08, 0.06)),
    LinkShape("LINK7", "fr3_link7_bounding_box", "capsule", (0.07, 0.14)),
    _HAND,
)


def fr3_link_shapes(crude_type: CrudeType) -> tuple[LinkShape, ...]:
    if crude_type == "ellipsoid":
        return FR3_ELLIPSOID_LINKS
    if crude_type == "capsule":
        return FR3_CAPSULE_LINKS
    raise ValueError(f"Unknown crude_type {crude_type!r}, expected 'ellipsoid' or 'capsule'")


FR3_Q_NOMINAL: NDArray = np.array(
    [0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4, 0.001, 0.001]
)


@dataclass(frozen=True)
class Scene:
    """A complete task: obstacles, robot bounding primitives and the goal."""

    name: str
    obstacles: tuple[BoxObstacle, ...]
    link_shapes: tuple[LinkShape, ...]
    # end-effector goal position, in the base frame [m]
    target_position: tuple[float, float, float]
    # end-effector goal orientation, as roll/pitch/yaw [deg] relative to the
    # orientation of the end-effector at the first control step
    target_relative_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # task-space proportional gain of the performance controller
    kp: float = 1.0
    q_init: NDArray = field(default_factory=lambda: FR3_Q_NOMINAL.copy())


THREE_BLOCKS = Scene(
    name="three-blocks",
    obstacles=(
        BoxObstacle("block1", (0.55, 0.1, 0.7), (0.1, 0.1, 0.1)),
        BoxObstacle("block2", (0.65, -0.1, 0.3), (0.1, 0.1, 0.1)),
        BoxObstacle("block3", (0.75, 0.0, 0.1), (0.1, 0.1, 0.1)),
    ),
    link_shapes=FR3_ELLIPSOID_LINKS,
    target_position=(0.7, 0.05, 0.1),
    target_relative_rpy=(0.0, 0.0, 0.0),
    kp=1.0,
)

TWO_WALLS = Scene(
    name="two-walls",
    obstacles=(
        BoxObstacle("wall_front_upper", (0.55, 0.0, 0.8), (0.05, 0.5, 0.25)),
        BoxObstacle("wall_front_lower", (0.55, 0.0, 0.1), (0.05, 0.5, 0.1)),
        BoxObstacle("wall_back_upper", (0.85, 0.0, 0.85), (0.05, 0.5, 0.2)),
        BoxObstacle("wall_back_lower", (0.85, 0.0, 0.15), (0.05, 0.5, 0.15)),
    ),
    link_shapes=FR3_CAPSULE_LINKS,
    target_position=(1.2, 0.0, 0.35),
    target_relative_rpy=(0.0, -90.0, 0.0),
    kp=0.2,
)

SCENES: dict[str, Scene] = {scene.name: scene for scene in (THREE_BLOCKS, TWO_WALLS)}
