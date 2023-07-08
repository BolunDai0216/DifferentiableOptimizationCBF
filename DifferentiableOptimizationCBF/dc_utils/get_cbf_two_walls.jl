include("update_arm.jl")
include("get_cbf_link_polygon_obstacle.jl")

function get_cbf_two_walls(rs::Vector{Float64}, qs::Vector{Float64})
    #=
        This function computes the value of
        α and ∂α/∂(r, q) between four polygon 
        obstacles and all of the robot links 
        that are relavant. This function requires 
        all of the links and obstacles be already 
        initialized.
    =#

    # update the position and orientation of all of the links
    update_arm(rs, qs)

    # compute α and J
    αs_ob1, Js_ob1 = get_cbf_link_polygon_obstacle(wall_front_upper)
    αs_ob2, Js_ob2 = get_cbf_link_polygon_obstacle(wall_front_lower)
    αs_ob3, Js_ob3 = get_cbf_link_polygon_obstacle(wall_back_upper)
    αs_ob4, Js_ob4 = get_cbf_link_polygon_obstacle(wall_back_lower)

    return [αs_ob1, αs_ob2, αs_ob3, αs_ob4], [Js_ob1, Js_ob2, Js_ob3, Js_ob4]
end