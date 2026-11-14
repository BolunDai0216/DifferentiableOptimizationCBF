# Controller Implementaion

In this tutorial, we show how our differentiable optimization based CBFQP controller for the FR3 is implemented, and how to use it with your own obstacles. The code lives in `DifferentiableOptimizationCBF.manipulator` and is split into small pieces that are composed by `DiffOptCBFController`:

| Piece | File | Role |
| --- | --- | --- |
| `Scene`, `BoxObstacle`, `LinkShape`, `FR3State` | `scene.py` | plain data: the obstacles, the bounding shapes of the robot links and the goal |
| `FR3Kinematics` | `kinematics.py` | pose and Jacobian of every bounding shape (using `pinocchio`) |
| `ManipulatorCBFEvaluator` | `julia_interop.py` | scaling factors and their Jacobians (using `DifferentiableCollisions.jl`) |
| `PerformanceController` | `performance_controller.py` | objective function of the QP |
| `ManipulatorCBFQPBuilder` | `cbf_qp.py` | adds the CBF constraints to obtain the QP |
| `QPSolver` | `qp_solver.py` | solves the QP (using `proxsuite`) |

One control step, i.e., `DiffOptCBFController.__call__`, is

```python
kinematics = self.kinematics(state.q)

objective = self.performance_controller(state, kinematics.end_effector, self.goal)

cbf = self.cbf_evaluator(
    np.array([link.position for link in kinematics.links]),
    np.array([link.rotation for link in kinematics.links]),
)

problem = self.qp_builder(objective, cbf, kinematics.links)
dq_target = self.qp_solver.solve(problem)
```

## Use the controller with your own obstacles

The controller only needs the joint positions and velocities of the robot, so it can be used without any simulator, e.g., on the real robot. A task is described by a `Scene`: the obstacles (axis-aligned boxes), the bounding shapes of the robot links, and the end-effector goal. The Julia objects and the QP are built from it, no Julia code has to be written.

```python
import numpy as np

from DifferentiableOptimizationCBF.manipulator import (
    FR3_CAPSULE_LINKS,
    BoxObstacle,
    DiffOptCBFController,
    DiffOptCBFControllerCfg,
    FR3State,
    Scene,
)

scene = Scene(
    name="my-scene",
    obstacles=(
        # name, center [m], half extents [m], in the robot base frame
        BoxObstacle("post", (0.45, 0.25, 0.25), (0.05, 0.05, 0.25)),
        BoxObstacle("shelf", (0.6, 0.0, 0.85), (0.2, 0.3, 0.02)),
    ),
    link_shapes=FR3_CAPSULE_LINKS,
    target_position=(0.35, 0.5, 0.3),
)
controller = DiffOptCBFController(DiffOptCBFControllerCfg(), scene)

# q and dq have shape (9,): 7 arm joints + 2 finger joints
dq_target, info = controller(FR3State(q=q, dq=dq))
```

`dq_target` contains the safe joint velocities, and `info` what they were computed from, e.g., `info.αs` are the scaling factors with shape `(n_obstacles, n_links)`. The bounding shapes can be changed as well: a `LinkShape` attaches an ellipsoid, a capsule or a sphere to a frame of the robot URDF, e.g., `LinkShape("LINK6", "fr3_link6_bounding_box", "capsule", (0.08, 0.06))`.

To turn the joint velocities into joint torques, the simulation examples use `JointTorqueLaw` (see `manipulator_exp.py`)

```python
torque_law = JointTorqueLaw(JointTorqueLawCfg(kp=30.0, kd=3.0, damping=0.1))
τ = torque_law(state, dq_target, controller.kinematics.gravity(state.q), dt)
```

Each piece of the controller can be replaced, e.g., to use your own performance controller, write a class with the same `__call__` as `PerformanceController` (it returns the $H$ and $g$ of the objective function as a `QPObjective`) and pass it to the controller

```python
controller = DiffOptCBFController(cfg, scene, performance_controller=MyPerformanceController())
```

The remainder of this tutorial goes through the pieces.

## Obtain the CBFs

To get the CBFs, we need the position and orientation of each bounding shape of the robot links, which are obtained from the forward kinematics as

```python
kinematics = self.kinematics(state.q)
```

where `kinematics.links[k]` holds the `position`, the `rotation` and the `jacobian` of the frame the `k`-th bounding shape is attached to.

Then, the scaling factors along with their Jacobians are computed as

```python
cbf = self.cbf_evaluator(positions, rotations)
```

where `cbf.αs[j, k]` is the scaling factor between the `j`-th obstacle and the `k`-th bounding shape, and `cbf.Js[j, k]` is its Jacobian with respect to the position and the quaternion of the bounding shape. This is a single call to Julia (`evaluate` in `dc_utils/manipulator_cbf.jl`), which updates the poses of the bounding shapes and calls `dc.proximity_jacobian` for every pair of obstacle and bounding shape.

## Optimization Problem

The optimization problem is defined as

$$
\begin{align}
\min_{\dot{\theta}_\mathrm{des}}\ & \Big\|J(\theta)\dot{\theta}_\mathrm{des} - \Big[K_p(p_\mathrm{des} - p) + \dot{p}_\mathrm{des}\Big]\Big\|_2^2 + \epsilon\Big\|\mathcal{N}(\theta)[\dot{\theta}_\mathrm{des} - K_p^\prime(\theta_\mathrm{nominal} - \theta)]\Big\|_2^2\\
\mathrm{subject\ to}\ & \frac{\partial\mathbf{H}}{\partial x}\dot{\theta}_\mathrm{des} \geq -\gamma\mathbf{H}(x).
\end{align}
$$

To solve this using `proxsuite`, we need to transform it into the standard form of

$$
\begin{align}
\min_x\ & \frac{1}{2}x^THx + g^Tx\\
\mathrm{subject\ to}\ & \mathrm{lb} \leq Cx.
\end{align}
$$

### CBF Constraints

For each individual CBF constraint, it is in the form of

$$
\frac{\partial\mathbf{h}}{\partial x}G(x)u = \frac{\partial\mathbf{h}}{\partial\mu}\frac{\partial\mu}{\partial x}G(x)u \geq -\gamma\mathbf{h}(x).
$$

The above inequality is equivalent to

$$
\frac{\partial\alpha}{\partial\mu}\begin{bmatrix}
        J_v(x)\\
        \displaystyle\frac{1}{2}\mathbf{Q}J_\omega(x)
\end{bmatrix}G(x)u = \frac{\partial\alpha}{\partial\mu}\begin{bmatrix}
    \mathbf{I} & \mathbf{0}\\
    \mathbf{0} & \displaystyle\frac{1}{2}\mathbf{Q}
\end{bmatrix}\begin{bmatrix}
    J_v(x)\\
    J_\omega(x)
\end{bmatrix}u \geq -\gamma(\alpha - \beta).
$$

If we let

$$
\begin{align}
    \texttt{cbf.αs[j, k]} &\leftarrow \alpha\\
    \texttt{cbf.Js[j, k]} &\leftarrow \frac{\partial\alpha}{\partial\mu}\\
    \texttt{pose_rate_from_twist} &\leftarrow \begin{bmatrix}
        \mathbf{I} & \mathbf{0}\\
        \mathbf{0} & \displaystyle\frac{1}{2}\mathbf{Q}
    \end{bmatrix}\\
    \texttt{link.jacobian} &\leftarrow \begin{bmatrix}
        J_v(x)\\
        J_\omega(x)
    \end{bmatrix}\\
    \gamma &\leftarrow \texttt{cfg.γ} = 5.0\\
    \beta &\leftarrow \texttt{cfg.β} = 1.03
\end{align}
$$

then, we can stack all of the CBFs and obtain the $\mathrm{lb}$ and $C$ matrices as follows (`ManipulatorCBFQPBuilder` in `cbf_qp.py`)

```python
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
```

```{note}
There are two choices for $\mathbf{Q}$, selected with `CBFQPCfg.exact_quaternion_rate`. The default (`True`) is the exact map from the angular velocity to the rate of the quaternion that $\partial\alpha/\partial\mu$ is taken with respect to: `link.jacobian` gives $\omega$ in the axes of the base frame, so $\dot{q} = \frac{1}{2}\omega\otimes q$, with $q$ the quaternion of the primitive handed to Julia (the link frame rotated so that the long axis is $x$). With it $C\dot{\theta}$ matches the finite differences of $\alpha$. The $\mathbf{Q}$ printed in the paper corresponds to $\dot{q} = \frac{1}{2}q\otimes\omega$, i.e., to $\omega$ expressed in the body frame. `False` selects the matrix used by the original implementation of the simulations (the paper's $\mathbf{Q}$ built from the link frame quaternion, combined with the base-frame $\omega$), which is only exact for spheres and is kept to reproduce the original results.
```

### Objective Function

To compute the remaining terms in the objective function, we first define 

$$
\begin{align}
a &= K_p(p_\mathrm{des} - p) + \dot{p}_\mathrm{des}\\
\dot{q}_\mathrm{nominal} &= K_p^\prime(\theta_\mathrm{nominal} - \theta).
\end{align}
$$

Then, the objective function can be simplified as

$$
\begin{align*}
&\ \Big(J(\theta)\dot{\theta}_\mathrm{des} - a\Big)^T\Big(J(\theta)\dot{\theta}_\mathrm{des} - a\Big) + \epsilon\Big(\mathcal{N}(\theta)[\dot{\theta}_\mathrm{des} - \dot{q}_\mathrm{nominal}]\Big)^T\Big(\mathcal{N}(\theta)[\dot{\theta}_\mathrm{des} - \dot{q}_\mathrm{nominal}]\Big)\\
=&\ \Big(\dot{\theta}_\mathrm{des}^TJ(\theta)^T - a^T\Big)\Big(J(\theta)\dot{\theta}_\mathrm{des} - a\Big) + \epsilon\Big([\dot{\theta}_\mathrm{des} - \dot{q}_\mathrm{nominal}]^T\mathcal{N}(\theta)^T\Big)\Big(\mathcal{N}(\theta)[\dot{\theta}_\mathrm{des} - \dot{q}_\mathrm{nominal}]\Big)
\end{align*}
$$

The first part can be expanded as

$$
\begin{align*}
&\ \Big(\dot{\theta}_\mathrm{des}^TJ(\theta)^T - a^T\Big)\Big(J(\theta)\dot{\theta}_\mathrm{des} - a\Big)\\
=&\ \dot{\theta}_\mathrm{des}^TJ(\theta)^TJ(\theta)\dot{\theta}_\mathrm{des} - 2a^TJ(\theta)\dot{\theta}_\mathrm{des} + a^Ta\\
\equiv &\ \dot{\theta}_\mathrm{des}^TJ(\theta)^TJ(\theta)\dot{\theta}_\mathrm{des} - 2a^TJ(\theta)\dot{\theta}_\mathrm{des} & \text{since } a^Ta \text{ is not dependent on}\ \dot{\theta}_\mathrm{des}.
\end{align*}
$$

The second part can be expanded as

$$
\begin{align*}
&\ \Big([\dot{\theta}_\mathrm{des} - \dot{q}_\mathrm{nominal}]^T\mathcal{N}(\theta)^T\Big)\Big(\mathcal{N}(\theta)[\dot{\theta}_\mathrm{des} - \dot{q}_\mathrm{nominal}]\Big)\\
=&\ \Big(\dot{\theta}_\mathrm{des}^T\mathcal{N}(\theta)^T - \dot{q}_\mathrm{nominal}^T\mathcal{N}(\theta)^T\Big)\Big(\mathcal{N}(\theta)\dot{\theta}_\mathrm{des} - \mathcal{N}(\theta)\dot{q}_\mathrm{nominal}\Big)\\
\equiv&\ \dot{\theta}_\mathrm{des}^T\mathcal{N}(\theta)^T\mathcal{N}(\theta)\dot{\theta}_\mathrm{des} - 2\dot{q}_\mathrm{nominal}^T\mathcal{N}(\theta)^T\mathcal{N}(\theta)\dot{\theta}_\mathrm{des}.
\end{align*}
$$

Then, the objective function can be written as

$$
\mathcal{J} = \dot{\theta}_\mathrm{des}^T\Big(J(\theta)^TJ(\theta) + \mathcal{N}(\theta)^T\mathcal{N}(\theta)\Big)\dot{\theta}_\mathrm{des} - 2\Big(a^TJ(\theta) + \dot{q}_\mathrm{nominal}^T\mathcal{N}(\theta)^T\mathcal{N}(\theta)\Big)\dot{\theta}_\mathrm{des}
$$

Therefore, we have $H$ and $g$ as

$$
\begin{align*}
H &= 2\Big(J(\theta)^TJ(\theta) + \mathcal{N}(\theta)^T\mathcal{N}(\theta)\Big)\\
g &= -2\Big(a^TJ(\theta) + \dot{q}_\mathrm{nominal}^T\mathcal{N}(\theta)^T\mathcal{N}(\theta)\Big)^T
\end{align*}
$$

In `PerformanceController` (`performance_controller.py`), with `J` the Jacobian of the end-effector, we have

```python
# target end-effector twist
a = self.cfg.kp * pose_error

# joint centering
dq_nominal = self.cfg.k_nullspace * (self.cfg.q_nominal - state.q)[:, np.newaxis]

J = end_effector.jacobian
N = np.eye(J.shape[1]) - np.linalg.pinv(J) @ J

H = 2 * J.T @ J + 2 * N.T @ N
g = -2 * (a.T @ J + dq_nominal.T @ N.T @ N).T
```

where `pose_error` stacks the position error and the orientation error (in axis-angle form) of the end-effector. Now, we can solve the optimization problem using `proxsuite` (`QPSolver` in `qp_solver.py`) and obtain the desired joint velocities.
