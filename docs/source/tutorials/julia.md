# DifferentiableCollisions.jl and how to use in Python

In this tutorial, we show how we can use [`DifferentiableCollisions.jl`](https://github.com/kevin-tracy/DifferentiableCollisions.jl) to compute the minimum uniform scaling factor $\alpha$ and how we can port this functionality to Python.

## Julia Library

To compute the minimum uniform scaling factor $\alpha$ we utilize the [`DifferentiableCollisions.jl`](https://github.com/kevin-tracy/DifferentiableCollisions.jl) library. Let's say we want to compute the value of $\alpha$ between an ellipsoid and a polygon. The surface of the ellisoid is parameterized as

$$
\frac{x^2}{a^2} + \frac{y^2}{b^2} + \frac{z^2}{c^2} = \begin{bmatrix}
    x\\
    y\\
    z
\end{bmatrix}^T\begin{bmatrix}
    1 / a^2 & 0 & 0\\
    0 & 1 / b^2 & 0\\
    0 & 0 & 1 / c^2
\end{bmatrix}\begin{bmatrix}
    x\\
    y\\
    z
\end{bmatrix} = \mathbf{x}^T\mathbf{P}\mathbf{x} = 1.
$$(eqn:ellipsoid_parameterization)

The surface of the polygon is parameterized as

$$
    \mathbf{A}\mathbf{x} = \mathbf{b}
$$(eqn:polygon_parameterization)

where $\mathbf{A}$ is the matrix that contains all of the normal vectors of the polygon faces, $\mathbf{b}$ is the distance the faces are from the center of the polygon along the corresponding normal vectors. We first define the two shapes in `DifferentiableCollisions.jl` as

```julia
import StaticArrays as sa
import DifferentiableCollisions as dc

a = 0.165
b = 0.09
c = 0.09

P = sa.@SMatrix [1/(a*a) 0.0 0.0
                 0.0 1/(b*b) 0.0
                 0.0 0.0 1/(c*c)]
ellipsoid = dc.Ellipsoid(P)

A = sa.@SMatrix [1.0 0.0 0.0
                 0.0 1.0 0.0
                 0.0 0.0 1.0
                 -1.0 0.0 0.0
                 0.0 -1.0 0.0
                 0.0 0.0 -1.0]
b = sa.@SVector [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
polygon = dc.Polytope(A, b)
```

Now that we have the two shapes, we next need to define the position and orientation of the two shapes

```julia
ellipsoid.r = sa.SVector{3}([0.0, 0.0, 0.0])
ellipsoid.q = sa.SVector{4}([1.0, 0.0, 0.0, 0.0])

polygon.r = sa.SVector{3}([2.0, 2.0, 2.0])
polygon.q = sa.SVector{4}([1.0, 0.0, 0.0, 0.0])
```

Then, we can get the minimum uniform scaling factor $\alpha$ by calling

```julia
α, _, J = dc.proximity_jacobian(ellipsoid, polygon; verbose=false, pdip_tol=1e-6)
```

where `α` gives the minimum scaling factor and `J` represents

$$
    J = \begin{bmatrix}
        \displaystyle\frac{\partial\alpha}{\partial r_\mathrm{e}} & 
        \displaystyle\frac{\partial\alpha}{\partial q_\mathrm{e}} & 
        \displaystyle\frac{\partial\alpha}{\partial r_\mathrm{p}} & 
        \displaystyle\frac{\partial\alpha}{\partial q_\mathrm{p}}
    \end{bmatrix} \in \mathbb{R}^{14}.
$$