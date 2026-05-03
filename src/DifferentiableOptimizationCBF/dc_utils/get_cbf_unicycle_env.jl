import DifferentiableCollisions as dc

function get_cbf_unicycle_env(rs::AbstractVector{<:Real}, qs::AbstractVector{<:Real})
    unicycle.r = sa.SVector{3}(rs[1:3])
    unicycle.q = sa.SVector{4}(qs[1:4])

    α1, _, J1 = dc.proximity_jacobian(obstacle1, unicycle; verbose=false, pdip_tol=1e-6)
    α2, _, J2 = dc.proximity_jacobian(obstacle2, unicycle; verbose=false, pdip_tol=1e-6)

    return [α1, α2], [J1, J2]
end