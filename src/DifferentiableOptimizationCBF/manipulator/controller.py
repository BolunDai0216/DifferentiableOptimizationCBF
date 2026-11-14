from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Protocol

import numpy as np

from DifferentiableOptimizationCBF.manipulator.cbf_qp import CBFQPCfg, ManipulatorCBFQPBuilder
from DifferentiableOptimizationCBF.manipulator.julia_interop import (
    CBFEvaluation,
    CBFEvaluatorCfg,
    ManipulatorCBFEvaluator,
)
from DifferentiableOptimizationCBF.manipulator.kinematics import (
    FR3Kinematics,
    FR3KinematicsCfg,
    FR3KinematicsData,
    FrameKinematics,
)
from DifferentiableOptimizationCBF.manipulator.performance_controller import (
    EndEffectorGoalCfg,
    PerformanceController,
    PerformanceControllerCfg,
)
from DifferentiableOptimizationCBF.qp_solver import QPObjective, QPProblem, QPSolver, QPSolverCfg

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from DifferentiableOptimizationCBF.manipulator.scene import FR3State, Scene


class PerformanceControllerProtocol(Protocol):
    """What a custom performance controller has to implement."""

    def __call__(
        self, state: FR3State, end_effector: FrameKinematics, goal: EndEffectorGoalCfg
    ) -> QPObjective: ...


@dataclass
class DiffOptCBFControllerCfg:
    cbf_qp: CBFQPCfg = field(default_factory=CBFQPCfg)
    # None -> PerformanceControllerCfg(kp=scene.kp)
    performance_controller: PerformanceControllerCfg | None = None
    kinematics: FR3KinematicsCfg = field(default_factory=FR3KinematicsCfg)
    cbf_evaluator: CBFEvaluatorCfg = field(default_factory=CBFEvaluatorCfg)
    qp_eps_abs: float = 1.0e-6


@dataclass
class DiffOptCBFControllerInfo:
    kinematics: FR3KinematicsData
    cbf: CBFEvaluation
    problem: QPProblem
    # wall-clock time spent computing the CBFs and their Jacobians [s]
    cbf_time: float

    @property
    def αs(self) -> NDArray:
        """Scaling factors, shape (n_obstacles, n_links). Collision-free iff all are > 1."""
        return self.cbf.αs


class DiffOptCBFController:
    """Differentiable optimization based CBF-QP controller for the FR3.

    Maps the joint state to safe joint velocity targets. It only needs the
    joint state, so it runs the same way in simulation and on the real robot:

        controller = DiffOptCBFController(DiffOptCBFControllerCfg(), scene)
        dq_target, info = controller(FR3State(q=q, dq=dq))

    The pieces it is made of are attributes (``kinematics``, ``cbf_evaluator``,
    ``performance_controller``, ``qp_builder``, ``qp_solver``, ``goal``) and
    each of them can be replaced through the keyword arguments of the constructor.
    """

    def __init__(
        self,
        cfg: DiffOptCBFControllerCfg,
        scene: Scene,
        *,
        kinematics: FR3Kinematics | None = None,
        cbf_evaluator: ManipulatorCBFEvaluator | None = None,
        performance_controller: PerformanceControllerProtocol | None = None,
        qp_builder: ManipulatorCBFQPBuilder | None = None,
        qp_solver: QPSolver | None = None,
    ) -> None:
        self.cfg = cfg
        self.scene = scene
        self.goal = EndEffectorGoalCfg.from_scene(scene)

        self.kinematics = kinematics or FR3Kinematics(cfg.kinematics, scene.link_shapes)
        self.cbf_evaluator = cbf_evaluator or ManipulatorCBFEvaluator.from_scene(
            scene, cfg.cbf_evaluator
        )
        self.performance_controller = performance_controller or PerformanceController(
            cfg.performance_controller or PerformanceControllerCfg(kp=scene.kp)
        )
        self.qp_builder = qp_builder or ManipulatorCBFQPBuilder(cfg.cbf_qp)
        self.qp_solver = qp_solver or QPSolver(
            QPSolverCfg(
                n=self.kinematics.nv,
                n_eq=0,
                n_ieq=len(scene.link_shapes) * len(scene.obstacles),
                eps_abs=cfg.qp_eps_abs,
            )
        )

    def __call__(self, state: FR3State) -> tuple[NDArray, DiffOptCBFControllerInfo]:
        """Returns the safe joint velocity targets, shape (nv,), and what they were computed from."""
        kinematics = self.kinematics(state.q)

        objective = self.performance_controller(state, kinematics.end_effector, self.goal)

        tic = time.perf_counter()
        cbf = self.cbf_evaluator(
            np.array([link.position for link in kinematics.links]),
            np.array([link.rotation for link in kinematics.links]),
        )
        cbf_time = time.perf_counter() - tic

        problem = self.qp_builder(objective, cbf, kinematics.links)
        dq_target = np.array(self.qp_solver.solve(problem), copy=True)

        info = DiffOptCBFControllerInfo(
            kinematics=kinematics, cbf=cbf, problem=problem, cbf_time=cbf_time
        )
        return dq_target, info
