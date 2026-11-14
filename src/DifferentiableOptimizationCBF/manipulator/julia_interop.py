from __future__ import annotations

from dataclasses import dataclass
from functools import cache
from typing import TYPE_CHECKING

import numpy as np
from scipy.spatial.transform import Rotation

from DifferentiableOptimizationCBF.dc_utils import include_jl

if TYPE_CHECKING:
    from collections.abc import Sequence

    from numpy.typing import NDArray

    from DifferentiableOptimizationCBF.manipulator.scene import BoxObstacle, LinkShape, Scene

# A LinkShape has its long axis along the z-axis of its frame, the Julia
# primitives have theirs along the x-axis: R_primitive = R_frame @ R_PRIMITIVE_FROM_FRAME
R_PRIMITIVE_FROM_FRAME: NDArray = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])

_N_SIZE_PARAMS = {"ellipsoid": 3, "capsule": 2, "sphere": 1}


@dataclass
class CBFEvaluatorCfg:
    # tolerance of the primal-dual interior point solver of DifferentiableCollisions.jl
    pdip_tol: float = 1.0e-6


@dataclass
class CBFEvaluation:
    """Scaling factors between every (obstacle, link) pair and their Jacobians."""

    αs: NDArray  # shape (n_obstacles, n_links)
    # ∂α/∂(r, quat) of the link primitive, quat = [qw, qx, qy, qz], shape (n_obstacles, n_links, 7)
    Js: NDArray
    # the primitive quaternions [qw, qx, qy, qz] the Jacobians were taken at, shape (n_links, 4)
    link_quats: NDArray


@cache
def _load_julia_module():
    return include_jl("manipulator_cbf.jl")


class ManipulatorCBFEvaluator:
    """Adapter around DifferentiableCollisions.jl for a set of link primitives and box obstacles.

    Builds the Julia objects from plain data on construction, and exposes a
    Python-friendly callable that returns clean numpy arrays. Every instance
    owns its Julia objects, so several evaluators can coexist in one process.
    """

    def __init__(
        self,
        link_shapes: Sequence[LinkShape],
        obstacles: Sequence[BoxObstacle],
        cfg: CBFEvaluatorCfg | None = None,
    ) -> None:
        self.cfg = cfg if cfg is not None else CBFEvaluatorCfg()
        self.link_shapes = tuple(link_shapes)
        self.obstacles = tuple(obstacles)

        if not self.link_shapes or not self.obstacles:
            raise ValueError("ManipulatorCBFEvaluator needs at least one link and one obstacle")

        self._jl = _load_julia_module()
        self._jl_scene = self._jl.create_scene(self.cfg.pdip_tol)

        for shape in self.link_shapes:
            if shape.kind not in _N_SIZE_PARAMS:
                raise ValueError(f"Link {shape.name!r}: unknown kind {shape.kind!r}")
            if len(shape.size) != _N_SIZE_PARAMS[shape.kind]:
                raise ValueError(
                    f"Link {shape.name!r}: a {shape.kind} takes {_N_SIZE_PARAMS[shape.kind]} "
                    f"size parameters, got {len(shape.size)}"
                )
            self._jl.add_link(self._jl_scene, shape.kind, tuple(float(s) for s in shape.size))

        for obstacle in self.obstacles:
            self._jl.add_box_obstacle(
                self._jl_scene,
                tuple(float(p) for p in obstacle.position),
                tuple(float(h) for h in obstacle.half_extents),
            )

        # a sphere has no long axis, its frame orientation is used as is
        self._R_offsets = np.array(
            [np.eye(3) if s.kind == "sphere" else R_PRIMITIVE_FROM_FRAME for s in self.link_shapes]
        )

    @classmethod
    def from_scene(
        cls, scene: Scene, cfg: CBFEvaluatorCfg | None = None
    ) -> ManipulatorCBFEvaluator:
        return cls(scene.link_shapes, scene.obstacles, cfg)

    @property
    def n_links(self) -> int:
        return len(self.link_shapes)

    @property
    def n_obstacles(self) -> int:
        return len(self.obstacles)

    def __call__(self, positions: NDArray, rotations: NDArray) -> CBFEvaluation:
        """Evaluate the CBFs for the given poses of the link frames.

        Args:
            positions: positions of the link frames, shape (n_links, 3)
            rotations: rotation matrices of the link frames, shape (n_links, 3, 3)
        """
        rs = np.ascontiguousarray(positions, dtype=float)
        rotations = np.asarray(rotations, dtype=float)
        if rs.shape != (self.n_links, 3) or rotations.shape != (self.n_links, 3, 3):
            raise ValueError(
                f"Expected positions of shape ({self.n_links}, 3) and rotations of shape "
                f"({self.n_links}, 3, 3), got {rs.shape} and {rotations.shape}"
            )

        quats_xyzw = Rotation.from_matrix(rotations @ self._R_offsets).as_quat()
        qs = np.ascontiguousarray(quats_xyzw[:, [3, 0, 1, 2]])

        αs, Js = self._jl.evaluate(self._jl_scene, rs, qs)

        return CBFEvaluation(αs=np.array(αs, copy=True), Js=np.array(Js, copy=True), link_quats=qs)
