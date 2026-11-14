#=
    Generic DifferentiableCollisions.jl backend of the manipulator CBF.

    Nothing in here knows about a specific robot or experiment: a `CBFScene`
    is a container of link primitives and box obstacles that is filled from
    Python (see `manipulator/julia_interop.py`). Every `CBFScene` owns its
    objects, so several scenes can live in the same Julia session.
=#
module ManipulatorCBF

import StaticArrays as sa
import DifferentiableCollisions as dc

# Faces of an axis-aligned box: A x <= b, with b = [half extents; half extents]
const BOX_A = sa.@SMatrix [1.0 0.0 0.0
    0.0 1.0 0.0
    0.0 0.0 1.0
    -1.0 0.0 0.0
    0.0 -1.0 0.0
    0.0 0.0 -1.0]

struct CBFScene
    links::Vector{Any}
    obstacles::Vector{Any}
    pdip_tol::Float64
end

create_scene(pdip_tol::Real) = CBFScene(Any[], Any[], Float64(pdip_tol))

function create_link(kind::AbstractString, size)
    #=
        The long axis of every primitive is its local x-axis.

        ellipsoid: size = (a, b, c), the semi-axes of x^2/a^2 + y^2/b^2 + z^2/c^2 <= 1,
                   represented as x^T P x <= 1 with P = diag(1/a^2, 1/b^2, 1/c^2)
        capsule:   size = (radius, length)
        sphere:    size = (radius,)
    =#
    if kind == "ellipsoid"
        a, b, c = Float64.(size)
        P = sa.@SMatrix [1/(a*a) 0.0 0.0
            0.0 1/(b*b) 0.0
            0.0 0.0 1/(c*c)]
        return dc.Ellipsoid(P)
    elseif kind == "capsule"
        radius, length = Float64.(size)
        return dc.Capsule(radius, length)
    elseif kind == "sphere"
        return dc.Sphere(Float64(size[1]))
    end
    error("Unknown link primitive kind \"$kind\", expected ellipsoid, capsule or sphere")
end

function add_link(scene::CBFScene, kind::AbstractString, size)
    push!(scene.links, create_link(kind, size))
    return length(scene.links)
end

function add_box_obstacle(scene::CBFScene, center, half_extents)
    hx, hy, hz = Float64.(half_extents)
    box = dc.Polytope(BOX_A, sa.SVector{6}(hx, hy, hz, hx, hy, hz))
    box.r = sa.SVector{3}(Float64.(center))
    box.q = sa.SVector{4}(1.0, 0.0, 0.0, 0.0)
    push!(scene.obstacles, box)
    return length(scene.obstacles)
end

function update_links(scene::CBFScene, rs::AbstractMatrix{<:Real}, qs::AbstractMatrix{<:Real})
    #=
        rs: (n_links, 3) positions, qs: (n_links, 4) quaternions [qw, qx, qy, qz]
    =#
    n_links = length(scene.links)
    size(rs) == (n_links, 3) || error("rs must have size ($n_links, 3), got $(size(rs))")
    size(qs) == (n_links, 4) || error("qs must have size ($n_links, 4), got $(size(qs))")

    for (k, link) in enumerate(scene.links)
        link.r = sa.SVector{3,Float64}(rs[k, 1], rs[k, 2], rs[k, 3])
        link.q = sa.SVector{4,Float64}(qs[k, 1], qs[k, 2], qs[k, 3], qs[k, 4])
    end
end

function proximity(obstacle, link, pdip_tol::Float64)
    # J = ∂(x, α)/∂(r_obstacle, q_obstacle, r_link, q_link) ∈ R^{4x14}
    α, _, J = dc.proximity_jacobian(obstacle, link; verbose=false, pdip_tol=pdip_tol)
    return α, J
end

function evaluate(scene::CBFScene, rs::AbstractMatrix{<:Real}, qs::AbstractMatrix{<:Real})
    #=
        Moves the links to (rs, qs) and computes, for every (obstacle, link) pair,
        α and ∂α/∂(r_link, q_link).

        Returns αs with size (n_obstacles, n_links) and Js with size (n_obstacles, n_links, 7).
    =#
    update_links(scene, rs, qs)

    n_obstacles, n_links = length(scene.obstacles), length(scene.links)
    αs = Matrix{Float64}(undef, n_obstacles, n_links)
    Js = Array{Float64,3}(undef, n_obstacles, n_links, 7)

    for (k, link) in enumerate(scene.links), (j, obstacle) in enumerate(scene.obstacles)
        α, J = proximity(obstacle, link, scene.pdip_tol)
        αs[j, k] = α
        for i in 1:7
            Js[j, k, i] = J[4, 7+i]
        end
    end

    return αs, Js
end

end # module

ManipulatorCBF
