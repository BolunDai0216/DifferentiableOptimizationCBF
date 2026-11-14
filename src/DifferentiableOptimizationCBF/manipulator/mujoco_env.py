"""MuJoCo simulation of the FR3 for a given ``Scene``.

The model is assembled with ``mujoco.MjSpec``: the robot comes from the MJCF
shipped with ``FR3Env`` (base at the world origin), the obstacles and the
bounding primitives drawn on the links are generated from the ``Scene``. The
generalized coordinates are ordered as in the pinocchio model of
``fr3_with_bounding_boxes.urdf``: 7 arm joints followed by the 2 fingers.
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import FR3Env
import mujoco
import numpy as np
import pinocchio as pin

from DifferentiableOptimizationCBF.manipulator.scene import FR3State, LinkShape, Scene

if TYPE_CHECKING:
    from numpy.typing import NDArray

_ROBOTS_DIR = Path(FR3Env.getDataPath()) / "robots"
FR3_MJCF_PATH = _ROBOTS_DIR / "fr3.xml"
FR3_URDF_PATH = _ROBOTS_DIR / "fr3_with_bounding_boxes.urdf"

# geom groups, can be toggled with the number keys in the viewer
_OBSTACLE_GROUP = 0
_LINK_SHAPE_GROUP = 1

_MJPYTHON_HELP = (
    "On macOS the interactive MuJoCo viewer only works when the script is started with the "
    "`mjpython` launcher that ships with mujoco, e.g. `uv run mjpython -m <module>` instead of "
    "`uv run python -m <module>` (the `just run-two-walls` / `just run-three-blocks` recipes do "
    "this for you). Alternatively run headless with `render=False`."
)

# delay the viewer is allowed to catch up on by running faster than requested
_MAX_CATCH_UP_FRAMES = 5


@dataclass
class FR3MuJoCoEnvCfg:
    dt: float = 1e-3
    # passive mujoco viewer; False = headless
    render: bool = True
    # draw the translucent bounding primitives of the scene on the links
    show_link_shapes: bool = True
    # draw a small marker at the end-effector goal position
    show_target: bool = True
    # viewer refresh rate [Hz], the simulation itself always runs at 1 / dt
    render_fps: float = 60.0
    # when rendering, do not let the simulation run faster than wall-clock time
    realtime: bool = True
    # slow-motion factor of the viewer when ``realtime`` is on: 1 s of simulated
    # time is shown over ``slowdown`` seconds (3.0 = three times slower)
    slowdown: float = 1.0
    # free camera of the viewer and of ``render_rgb``
    camera_lookat: tuple[float, float, float] = (0.4, 0.0, 0.4)
    camera_distance: float = 2.6
    camera_azimuth: float = 135.0
    camera_elevation: float = -20.0
    link_shape_rgba: tuple[float, float, float, float] = (0.3921, 0.5843, 0.9294, 0.4)
    obstacle_rgba: tuple[float, float, float, float] = (1.0, 0.549, 0.0, 1.0)
    target_rgba: tuple[float, float, float, float] = (0.1, 0.8, 0.2, 0.8)


class FR3MuJoCoEnv:
    def __init__(self, cfg: FR3MuJoCoEnvCfg, scene: Scene) -> None:
        self.cfg = cfg
        self.scene = scene
        self.dt: float = cfg.dt

        self.model: mujoco.MjModel = _build_model(cfg, scene)
        self.data: mujoco.MjData = mujoco.MjData(self.model)
        if self.model.nq != scene.q_init.shape[0] or self.model.nu != self.model.nv:
            raise RuntimeError(
                f"Expected a fully actuated model with nq = {scene.q_init.shape[0]}, got "
                f"nq = {self.model.nq}, nu = {self.model.nu}"
            )

        self._obstacle_geom_ids = np.array(
            [self.model.geom(obstacle.name).id for obstacle in scene.obstacles], dtype=int
        )
        self.had_obstacle_contact: bool = False

        self._camera = mujoco.MjvCamera()
        self._camera.lookat[:] = cfg.camera_lookat
        self._camera.distance = cfg.camera_distance
        self._camera.azimuth = cfg.camera_azimuth
        self._camera.elevation = cfg.camera_elevation

        self._renderer: mujoco.Renderer | None = None
        self._viewer = self._launch_viewer() if cfg.render else None
        self._steps_per_sync = max(1, round(1.0 / (cfg.render_fps * cfg.dt * cfg.slowdown)))
        self._step_count = 0
        self._next_frame_time = time.perf_counter()

        self.reset()

    def reset(self, q_init: NDArray | None = None) -> FR3State:
        """Put the robot at rest at ``q_init`` (default: ``scene.q_init``)."""
        q = np.asarray(self.scene.q_init if q_init is None else q_init, dtype=float)
        if q.shape != (self.model.nq,):
            raise ValueError(f"q_init must have shape ({self.model.nq},), got {q.shape}")

        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = q
        mujoco.mj_forward(self.model, self.data)

        self.had_obstacle_contact = self.obstacle_contacts() > 0
        self._step_count = 0
        self._next_frame_time = time.perf_counter()
        if self._viewer is not None:
            self._viewer.sync()

        return self._get_state()

    def step(self, tau: NDArray) -> FR3State:
        """Apply the joint efforts ``tau`` (7 arm torques + 2 finger forces) for one time step.

        The arm torques are clipped to the FR3 torque limits of the actuators.
        """
        tau = np.asarray(tau, dtype=float)
        if tau.shape != (self.model.nu,):
            raise ValueError(f"tau must have shape ({self.model.nu},), got {tau.shape}")

        self.data.ctrl[:] = tau
        mujoco.mj_step(self.model, self.data)
        self._step_count += 1

        # contacts in ``data`` are the ones that were active during this step
        if self.data.ncon > 0 and self.obstacle_contacts() > 0:
            self.had_obstacle_contact = True

        if self._viewer is not None and self._step_count % self._steps_per_sync == 0:
            self._viewer.sync()
            if self.cfg.realtime:
                # Follow a wall-clock schedule, but forget any delay beyond a
                # few frames: time lost outside of ``step`` (e.g. the Julia
                # start-up of the controller) is then not caught up by
                # fast-forwarding the visualization.
                frame_time = self._steps_per_sync * self.dt * self.cfg.slowdown
                now = time.perf_counter()
                if self._next_frame_time > now:
                    time.sleep(self._next_frame_time - now)
                self._next_frame_time = max(
                    self._next_frame_time + frame_time, now - _MAX_CATCH_UP_FRAMES * frame_time
                )

        return self._get_state()

    def bias_forces(self) -> NDArray:
        """Gravity + Coriolis joint efforts of the simulated robot, shape (9,).

        Feeding these forward plays the role of the gravity compensation that the
        real FR3 applies internally. They are evaluated at the start of the last
        step, i.e. they lag the returned state by one ``dt``.
        """
        return self.data.qfrc_bias.copy()

    def obstacle_contacts(self) -> int:
        """Number of contact points currently detected between the robot and the obstacles."""
        if self.data.ncon == 0 or self._obstacle_geom_ids.size == 0:
            return 0
        geoms = self.data.contact.geom[: self.data.ncon]
        return int(np.isin(geoms, self._obstacle_geom_ids).any(axis=1).sum())

    def render_rgb(self, width: int = 960, height: int = 720) -> NDArray:
        """Offscreen image of the current state, shape (height, width, 3), dtype uint8."""
        if self._renderer is None or (self._renderer.width, self._renderer.height) != (
            width,
            height,
        ):
            if self._renderer is not None:
                self._renderer.close()
            self._renderer = mujoco.Renderer(self.model, height=height, width=width)
        self._renderer.update_scene(self.data, camera=self._camera)
        return self._renderer.render()

    def is_running(self) -> bool:
        """False once the user closed the viewer window, always True when headless."""
        if not self.cfg.render:
            return True
        return self._viewer is not None and self._viewer.is_running()

    def close(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    def _get_state(self) -> FR3State:
        return FR3State(q=self.data.qpos.copy(), dq=self.data.qvel.copy())

    def _launch_viewer(self):
        import mujoco.viewer

        try:
            viewer = mujoco.viewer.launch_passive(
                self.model, self.data, show_left_ui=False, show_right_ui=False
            )
        except RuntimeError as error:
            if sys.platform == "darwin":
                raise RuntimeError(_MJPYTHON_HELP) from error
            raise

        with viewer.lock():
            viewer.cam.lookat[:] = self._camera.lookat
            viewer.cam.distance = self._camera.distance
            viewer.cam.azimuth = self._camera.azimuth
            viewer.cam.elevation = self._camera.elevation
        return viewer


def _build_model(cfg: FR3MuJoCoEnvCfg, scene: Scene) -> mujoco.MjModel:
    spec = mujoco.MjSpec.from_file(str(FR3_MJCF_PATH))
    spec.option.timestep = cfg.dt
    spec.visual.global_.offwidth = 1920
    spec.visual.global_.offheight = 1080

    _make_torque_controlled(spec)

    for obstacle in scene.obstacles:
        spec.worldbody.add_geom(
            name=obstacle.name,
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=obstacle.position,
            size=obstacle.half_extents,
            rgba=cfg.obstacle_rgba,
            group=_OBSTACLE_GROUP,
        )

    if cfg.show_target:
        spec.worldbody.add_geom(
            name="target",
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            pos=scene.target_position,
            size=[0.02, 0.0, 0.0],
            rgba=cfg.target_rgba,
            contype=0,
            conaffinity=0,
            group=_OBSTACLE_GROUP,
        )

    if cfg.show_link_shapes and scene.link_shapes:
        _add_link_shapes(spec, scene.link_shapes, cfg.link_shape_rgba)

    return spec.compile()


def _make_torque_controlled(spec: mujoco.MjSpec) -> None:
    """One effort actuator per degree of freedom, in the order of the joints.

    The MJCF drives both fingers through a single position servo on a tendon and
    ties them together with an equality constraint. The pinocchio model has two
    independent finger joints, so these are replaced by a motor on each finger.
    """
    for element in (*spec.keys, *spec.equalities, *spec.tendons):
        spec.delete(element)
    for actuator in list(spec.actuators):
        if actuator.trntype != mujoco.mjtTrn.mjTRN_JOINT:
            spec.delete(actuator)

    for joint in spec.joints:
        if joint.type == mujoco.mjtJoint.mjJNT_SLIDE:
            # 100 N is the effort limit of the fingers in the URDF
            spec.add_actuator(
                name=f"actuator_{joint.name}",
                trntype=mujoco.mjtTrn.mjTRN_JOINT,
                target=joint.name,
                ctrlrange=[-100.0, 100.0],
                ctrllimited=mujoco.mjtLimited.mjLIMITED_TRUE,
            )


def _add_link_shapes(
    spec: mujoco.MjSpec,
    link_shapes: tuple[LinkShape, ...],
    rgba: tuple[float, float, float, float],
) -> None:
    """Attach the bounding primitives to the bodies that carry their URDF frames.

    A URDF frame is rigidly attached to a pinocchio joint, and the i-th pinocchio
    joint moves the body of the i-th MuJoCo joint. The pose of the primitive in
    that body is found by comparing the two models in the zero configuration.
    """
    pin_model = pin.buildModelFromUrdf(str(FR3_URDF_PATH))
    pin_data = pin_model.createData()
    pin.forwardKinematics(pin_model, pin_data, pin.neutral(pin_model))

    mj_model = spec.compile()
    mj_data = mujoco.MjData(mj_model)
    if mj_model.njnt != pin_model.njoints - 1 or mj_model.nq != pin_model.nq:
        raise RuntimeError("The MuJoCo and pinocchio models of the FR3 have different joints")
    mj_data.qpos[:] = 0.0
    mujoco.mj_kinematics(mj_model, mj_data)

    for shape in link_shapes:
        if not pin_model.existFrame(shape.frame):
            raise ValueError(f"Link shape {shape.name!r}: no frame {shape.frame!r} in the URDF")
        frame = pin_model.frames[pin_model.getFrameId(shape.frame)]
        # pinocchio 2 calls it ``parent``, pinocchio 3 ``parentJoint``
        parent_joint = frame.parentJoint if hasattr(frame, "parentJoint") else frame.parent

        if parent_joint == 0:
            body = spec.worldbody
            body_M_frame = frame.placement
        else:
            body_id = mj_model.jnt_bodyid[parent_joint - 1]
            body = spec.body(mj_model.body(body_id).name)
            world_M_body = pin.SE3(mj_data.xmat[body_id].reshape(3, 3), mj_data.xpos[body_id])
            body_M_frame = world_M_body.actInv(pin_data.oMi[parent_joint] * frame.placement)

        quat_xyzw = pin.Quaternion(body_M_frame.rotation).coeffs()
        geom_type, size = _geom_type_and_size(shape)
        body.add_geom(
            name=f"{shape.name}_shape",
            type=geom_type,
            size=size,
            pos=body_M_frame.translation,
            quat=[quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
            rgba=rgba,
            contype=0,
            conaffinity=0,
            group=_LINK_SHAPE_GROUP,
            # visual only: must not change the inertia of the link
            mass=0.0,
            density=0.0,
        )


def _geom_type_and_size(shape: LinkShape) -> tuple[mujoco.mjtGeom, list[float]]:
    """MuJoCo geom of a link shape, whose long axis is the z-axis of its frame."""
    if shape.kind == "ellipsoid":
        long, short_1, short_2 = shape.size
        return mujoco.mjtGeom.mjGEOM_ELLIPSOID, [short_1, short_2, long]
    if shape.kind == "capsule":
        radius, length = shape.size
        return mujoco.mjtGeom.mjGEOM_CAPSULE, [radius, length / 2, 0.0]
    if shape.kind == "sphere":
        (radius,) = shape.size
        return mujoco.mjtGeom.mjGEOM_SPHERE, [radius, 0.0, 0.0]
    raise ValueError(f"Link shape {shape.name!r}: unknown kind {shape.kind!r}")
