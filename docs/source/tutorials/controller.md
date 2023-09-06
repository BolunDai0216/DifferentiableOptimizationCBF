# Controller Implementaion

In this tutorial, we show how to implement our differentiable optimization based CBFQP controller to solve the `three-blocks` task. The controller class inherits from `DifferentiableOptimizationCBF.base_controller.BaseController`.

## Obtain the CBFs

To get the CBFs, we would need the position and orientation of each bounding shape of the robot links, which is obtained as

```python
rs, qs = self.compute_rs_qs(info)
```

where `rs` is an array that contains the positions of the seven bounding shapes and `qs` is an array that contains the orientation (using quaternions) of the seven bounding shapes. 

Then, the scaling factors along with its Jacobians are computed as

```python
_αs, Js = self.get_cbf(rs, qs)
```

where `_αs` contains the scaling factors and `Js` contains the Jacobians of the scaling factors.

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
    \texttt{α} &\leftarrow \alpha\\
    \texttt{J_link[7:]} &\leftarrow \frac{\partial\alpha}{\partial\mu}\\
    \texttt{Q_mat_link} &\leftarrow \begin{bmatrix}
        \mathbf{I} & \mathbf{0}\\
        \mathbf{0} & \displaystyle\frac{1}{2}\mathbf{Q}
    \end{bmatrix}\\
    \texttt{info[f"J_{link}"]} &\leftarrow \begin{bmatrix}
        J_v(x)\\
        J_\omega(x)
    \end{bmatrix}\\
    \gamma &\leftarrow 5.0\\
    \beta &\leftarrow 1.03
\end{align}
$$

then, we can stack all of the CBFs and obtain the $\mathrm{lb}$ and $C$ matrices as follows

```python
# compute α's and J's
αs = []
Cs = []

for k, link in enumerate(self.frame_names):
    _Q_mat_link = get_Q_mat(info[f"q_{link}"])
    Q_mat_link = block_diag(np.eye(3), 0.5 * _Q_mat_link)

    for j in range(3):
        α, J_link = _αs[j][k], np.array(Js[j][k])
        αs.append(copy.deepcopy(α))
        Cs.append(J_link[-1, 7:][np.newaxis, :] @ Q_mat_link @ info[f"J_{link}"])

lb = -5.0 * (np.array(αs)[:, np.newaxis] - 1.03)
C = np.concatenate(Cs, axis=0)
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

Define the dictionary

```python
params = {
    "Jacobian": jacobian,
    "p_error": p_error,
    "p_current": p_current,
    "dp_target": dp_target,
    "Kp": 0.1 * np.eye(6),
    "dq_nominal": dq_nominal,
    "nullspace_proj": np.eye(9) - pinv_jac @ jacobian,
    "lb": lb,
    "C": C,
}
```

Then, we have $H$ and $g$ as

```python
H = 2 * params["Jacobian"].T @ params["Jacobian"] + 2 * params["nullspace_proj"].T @ params["nullspace_proj"]
a = params["Kp"] @ params["p_error"] + params["dp_target"]
g = -2 * (a.T @ params["Jacobian"] + params["dq_nominal"].T @ params["nullspace_proj"].T @ params["nullspace_proj"]).T
```

Now, we can solve the optimization problem using `proxsuite` and obtain the desired joint velocities.