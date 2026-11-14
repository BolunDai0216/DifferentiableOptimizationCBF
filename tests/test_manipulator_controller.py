"""Tests of the manipulator CBF controller. No simulator needed, only pinocchio + Julia.

The first Julia call is slow (JIT), so the whole file takes about a minute.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from DifferentiableOptimizationCBF.manipulator import (
    FR3_Q_NOMINAL,
    SCENES,
    THREE_BLOCKS,
    TWO_WALLS,
    BoxObstacle,
    CBFEvaluatorCfg,
    CBFQPCfg,
    DiffOptCBFController,
    DiffOptCBFControllerCfg,
    FR3Kinematics,
    FR3KinematicsCfg,
    FR3State,
    JointTorqueLaw,
    JointTorqueLawCfg,
    LinkShape,
    ManipulatorCBFEvaluator,
    ManipulatorCBFQPBuilder,
    Scene,
)
from DifferentiableOptimizationCBF.qp_solver import QPObjective

DATA_DIR = Path(__file__).parent / "data"

# Obstacle-free-ish configurations: the nominal one plus seeded perturbations
# that keep every α of both shipped scenes above 1.06.
REFERENCE_QS = np.load(DATA_DIR / "original_controller_three_blocks.npz")["qs"]


def _link_poses(kinematics_data):
    return (
        np.array([link.position for link in kinematics_data.links]),
        np.array([link.rotation for link in kinematics_data.links]),
    )


def _random_joint_direction(rng):
    direction = np.zeros(9)
    direction[:7] = rng.normal(size=7)
    return direction / np.linalg.norm(direction)


def test_kinematics_jacobians_match_finite_differences():
    kinematics = FR3Kinematics(FR3KinematicsCfg(), THREE_BLOCKS.link_shapes)
    rng = np.random.default_rng(0)
    q, direction, eps = REFERENCE_QS[1], _random_joint_direction(rng), 1e-6

    data = kinematics(q)
    assert len(data.links) == len(THREE_BLOCKS.link_shapes)
    assert kinematics.gravity(q).shape == (9,)

    plus, minus = kinematics(q + eps * direction), kinematics(q - eps * direction)
    for link, link_plus, link_minus in zip(data.links, plus.links, minus.links, strict=True):
        assert link.position.shape == (3,)
        assert link.rotation.shape == (3, 3)
        assert link.jacobian.shape == (6, 9)

        velocity = (link_plus.position - link_minus.position) / (2 * eps)
        # R_dot R^T = [ω]x, with ω expressed in the base frame
        skew = (link_plus.rotation - link_minus.rotation) / (2 * eps) @ link.rotation.T
        angular_velocity = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])

        twist = link.jacobian @ direction
        np.testing.assert_allclose(twist[:3], velocity, atol=1e-7)
        np.testing.assert_allclose(twist[3:], angular_velocity, atol=1e-7)


def test_unknown_frame_fails_loudly():
    with pytest.raises(ValueError, match="does not exist"):
        FR3Kinematics(FR3KinematicsCfg(), (LinkShape("X", "no_such_frame", "sphere", (0.1,)),))


@pytest.mark.parametrize("scene", [THREE_BLOCKS, TWO_WALLS], ids=lambda scene: scene.name)
def test_evaluator_shapes(scene):
    kinematics = FR3Kinematics(FR3KinematicsCfg(), scene.link_shapes)
    evaluator = ManipulatorCBFEvaluator.from_scene(scene)
    n_obstacles, n_links = len(scene.obstacles), len(scene.link_shapes)

    cbf = evaluator(*_link_poses(kinematics(FR3_Q_NOMINAL)))

    assert cbf.αs.shape == (n_obstacles, n_links)
    assert cbf.Js.shape == (n_obstacles, n_links, 7)
    assert cbf.link_quats.shape == (n_links, 4)
    # the robot starts collision-free
    assert cbf.αs.min() > 1.0

    with pytest.raises(ValueError, match="Expected positions"):
        evaluator(np.zeros((n_links + 1, 3)), np.tile(np.eye(3), (n_links + 1, 1, 1)))


def test_evaluator_rejects_malformed_link_shape():
    with pytest.raises(ValueError, match="size parameters"):
        ManipulatorCBFEvaluator(
            (LinkShape("LINK3", "fr3_link3_bounding_box", "capsule", (0.1,)),),
            THREE_BLOCKS.obstacles,
        )


def test_evaluators_of_different_scenes_do_not_interfere():
    poses = {}
    evaluators = {}
    for name, scene in SCENES.items():
        kinematics = FR3Kinematics(FR3KinematicsCfg(), scene.link_shapes)
        poses[name] = [_link_poses(kinematics(q)) for q in REFERENCE_QS[:2]]
        evaluators[name] = ManipulatorCBFEvaluator.from_scene(scene)

    first = evaluators["three-blocks"](*poses["three-blocks"][0])
    evaluators["two-walls"](*poses["two-walls"][1])
    again = evaluators["three-blocks"](*poses["three-blocks"][0])

    np.testing.assert_array_equal(first.αs, again.αs)
    np.testing.assert_array_equal(first.Js, again.Js)


def test_default_constraint_rows_match_finite_differences_of_α():
    """C @ dq is dα/dt: check it against finite differences of α along joint directions.

    Uses the ellipsoid scene, where α is smooth (a capsule next to a box is not
    differentiable where its axis is parallel to a face, e.g. at the nominal
    configuration of the two-walls scene).
    """
    scene = THREE_BLOCKS
    kinematics = FR3Kinematics(FR3KinematicsCfg(), scene.link_shapes)
    # tight tolerance: the accuracy of ∂α/∂(r, quat) follows the solver tolerance
    evaluator = ManipulatorCBFEvaluator.from_scene(scene, CBFEvaluatorCfg(pdip_tol=1e-10))
    builder = ManipulatorCBFQPBuilder(CBFQPCfg())
    objective = QPObjective(H=np.eye(9), g=np.zeros((9, 1)))
    rng = np.random.default_rng(0)
    eps = 1e-4

    def evaluate(q):
        data = kinematics(q)
        return data, evaluator(*_link_poses(data))

    for q in REFERENCE_QS[:4]:
        direction = _random_joint_direction(rng)

        data, cbf = evaluate(q)
        problem = builder(objective, cbf, data.links)
        assert problem.C.shape == (len(scene.link_shapes) * len(scene.obstacles), 9)
        assert problem.lb.shape == (len(scene.link_shapes) * len(scene.obstacles), 1)

        α_plus, α_minus = evaluate(q + eps * direction)[1].αs, evaluate(q - eps * direction)[1].αs
        # rows are ordered link by link
        dα_fd = ((α_plus - α_minus) / (2 * eps)).T.reshape(-1)

        np.testing.assert_allclose(problem.C @ direction, dα_fd, atol=1e-4)


@pytest.mark.parametrize("scene", [THREE_BLOCKS, TWO_WALLS], ids=lambda scene: scene.name)
def test_legacy_controller_reproduces_original_implementation(scene):
    """Regression against outputs recorded from the pre-refactor controllers.

    (``ThreeBlocksController`` / ``TwoWallsController`` at commit 8b28d71, called
    on ``qs`` in this order with a fresh controller.)
    """
    reference = np.load(DATA_DIR / f"original_controller_{scene.name.replace('-', '_')}.npz")
    cfg = DiffOptCBFControllerCfg(cbf_qp=CBFQPCfg(exact_quaternion_rate=False))
    controller = DiffOptCBFController(cfg, scene)

    for i, q in enumerate(reference["qs"]):
        dq_target, info = controller(FR3State(q=q, dq=np.zeros(9)))

        np.testing.assert_allclose(info.αs, reference["alphas"][i], atol=1e-6)
        np.testing.assert_allclose(info.problem.C, reference["C"][i], atol=1e-5)
        np.testing.assert_allclose(info.problem.lb, reference["lb"][i], atol=1e-5)
        np.testing.assert_allclose(dq_target, reference["dq_target"][i], atol=1e-4)


@pytest.mark.parametrize("exact_quaternion_rate", [False, True])
def test_custom_scene_end_to_end(exact_quaternion_rate):
    """A user-defined scene: other obstacles, other links, driven without a simulator."""
    scene = Scene(
        name="two-boxes",
        obstacles=(
            BoxObstacle("post", (0.45, 0.25, 0.25), (0.05, 0.05, 0.25)),
            BoxObstacle("shelf", (0.6, 0.0, 0.85), (0.2, 0.3, 0.02)),
        ),
        link_shapes=(
            LinkShape("LINK5_2", "fr3_link5_2_bounding_box", "capsule", (0.055, 0.14)),
            LinkShape("LINK6", "fr3_link6_bounding_box", "ellipsoid", (0.11, 0.08, 0.08)),
            LinkShape("LINK7", "fr3_link7_bounding_box", "capsule", (0.07, 0.14)),
            LinkShape("HAND", "fr3_hand_bounding_box", "sphere", (0.12,)),
        ),
        # on the other side of the post
        target_position=(0.35, 0.5, 0.3),
        kp=1.0,
    )
    cfg = DiffOptCBFControllerCfg(cbf_qp=CBFQPCfg(exact_quaternion_rate=exact_quaternion_rate))
    controller = DiffOptCBFController(cfg, scene)

    # ideal joint velocity tracking
    dt = 0.01
    state = FR3State(q=scene.q_init.copy(), dq=np.zeros(9))
    min_αs = []
    for _ in range(800):
        dq_target, info = controller(state)

        assert dq_target.shape == (9,)
        assert np.all(np.isfinite(dq_target))
        assert info.αs.shape == (2, 4)
        assert info.problem.C.shape == (8, 9)
        # the QP solution satisfies the CBF constraints
        assert np.all(info.problem.C @ dq_target >= info.problem.lb[:, 0] - 1e-4)
        min_αs.append(info.αs.min())

        state = FR3State(q=state.q + dq_target * dt, dq=dq_target)

    # the CBFs were active, and the robot never touched an obstacle
    assert min(min_αs) < cfg.cbf_qp.β + 0.05
    assert min(min_αs) > 1.0
    # it made progress towards the goal
    start = controller.kinematics(scene.q_init).end_effector.position
    end = controller.kinematics(state.q).end_effector.position
    goal = np.array(scene.target_position)
    assert np.linalg.norm(end - goal) < 0.5 * np.linalg.norm(start - goal)


def test_joint_torque_law():
    cfg = JointTorqueLawCfg(kp=30.0, kd=3.0, damping=0.1)
    state = FR3State(q=np.full(9, 0.2), dq=np.full(9, 0.5))
    dq_target, gravity, dt = np.full(9, 1.0), np.full(9, 2.0), 1e-3

    τ = JointTorqueLaw(cfg)(state, dq_target, gravity, dt)

    # 30 * (1.0 * 1e-3) + 3 * (1.0 - 0.5) + 2.0 - 0.1 * 0.5
    np.testing.assert_allclose(τ, np.full(9, 3.48))
