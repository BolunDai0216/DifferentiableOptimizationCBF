# DifferentiableCollisions.jl and how to use in Python

In this tutorial, we show how we can use [`DifferentiableCollisions.jl`](https://github.com/kevin-tracy/DifferentiableCollisions.jl) to compute the minimum uniform scaling factor $\alpha$ and how we can port this functionality to Python.

## Julia

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

Now that we have the two shapes, we next need to define the position (`r`) and orientation (`q`) of the two shapes

```julia
ellipsoid.r = sa.SVector{3}([0.0, 0.0, 0.0])  # [x, y, z]
ellipsoid.q = sa.SVector{4}([1.0, 0.0, 0.0, 0.0])  # [qw, qx, qy, qz]

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

## Python

To achieve the same functionality as above in Python, we first create two Julia functions. The first one creates the shapes by defining there geometries and creating global variables that can accessed outside of the function by the Julia runtime.

```julia
# create_shapes.jl
import StaticArrays as sa
import DifferentiableCollisions as dc

function create_shapes()
    a = 0.165
    b = 0.09
    c = 0.09

    P = sa.@SMatrix [1/(a*a) 0.0 0.0
                    0.0 1/(b*b) 0.0
                    0.0 0.0 1/(c*c)]
    global ellipsoid = dc.Ellipsoid(P)

    A = sa.@SMatrix [1.0 0.0 0.0
                    0.0 1.0 0.0
                    0.0 0.0 1.0
                    -1.0 0.0 0.0
                    0.0 -1.0 0.0
                    0.0 0.0 -1.0]
    b = sa.@SVector [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    global polygon = dc.Polytope(A, b)
end
```

The second function sets the position and orientation of the shapes and computes the minimum uniform scaling factor $\alpha$ and the Jacobian $J$.

```julia
# get_α_J.jl
import StaticArrays as sa
import DifferentiableCollisions as dc

function get_α_J(rs::Vector{Float64}, qs::Vector{Float64})
    # set the position and orientation
    ellipsoid.r = sa.SVector{3}(rs[1:3])
    ellipsoid.q = sa.SVector{4}(qs[1:4])
    polygon.r = sa.SVector{3}(rs[4:6])
    polygon.q = sa.SVector{4}(qs[5:8])

    # compute the minimum uniform scaling factor and Jacobian
    α, _, J = dc.proximity_jacobian(ellipsoid, polygon; verbose=false, pdip_tol=1e-6)

    return α, J
end
```

Assuming the `PyJulia` is installed, we can call Julia code from Python as follows. First, we import the relevant packages then load the two Julia functions above.

```python
from julia import Main

create_shapes = Main.include("create_shapes.jl")
get_α_J = Main.include("get_α_J.jl")
```

We can then create the shapes 

```python
create_shapes()
```

and get the minimum uniform scaling factor $\alpha$ and Jacobian $J$ by calling

```python
rs = np.array([0.0, 0.0, 0.0, 2.0, 2.0, 2.0])
qs = np.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])

α, J = get_α_J(rs, qs)
```
