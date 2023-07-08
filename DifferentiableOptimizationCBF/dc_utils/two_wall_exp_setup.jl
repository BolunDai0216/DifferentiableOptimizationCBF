import StaticArrays as sa
import DifferentiableCollisions as dc

function two_wall_exp_setup()
    wall_A = sa.@SMatrix [1.0 0.0 0.0
        0.0 1.0 0.0
        0.0 0.0 1.0
        -1.0 0.0 0.0
        0.0 -1.0 0.0
        0.0 0.0 -1.0]

    wall_front_upper_b = sa.@SVector [0.05, 0.5, 0.25, 0.05, 0.5, 0.25]
    wall_front_lower_b = sa.@SVector [0.05, 0.5, 0.1, 0.05, 0.5, 0.1]
    wall_back_upper_b = sa.@SVector [0.05, 0.5, 0.2, 0.05, 0.5, 0.2]
    wall_back_lower_b = sa.@SVector [0.05, 0.5, 0.15, 0.05, 0.5, 0.15]

    global wall_front_upper = dc.Polytope(wall_A, wall_front_upper_b)
    global wall_front_lower = dc.Polytope(wall_A, wall_front_lower_b)
    global wall_back_upper = dc.Polytope(wall_A, wall_back_upper_b)
    global wall_back_lower = dc.Polytope(wall_A, wall_back_lower_b)

    wall_front_upper.r = sa.@SVector [0.55, 0, 0.8]
    wall_front_upper.q = sa.@SVector [1.0, 0.0, 0.0, 0.0]

    wall_front_lower.r = sa.@SVector [0.55, 0, 0.1]
    wall_front_lower.q = sa.@SVector [1.0, 0.0, 0.0, 0.0]

    wall_back_upper.r = sa.@SVector [0.85, 0, 0.85]
    wall_back_upper.q = sa.@SVector [1.0, 0.0, 0.0, 0.0]

    wall_back_lower.r = sa.@SVector [0.85, 0, 0.15]
    wall_back_lower.q = sa.@SVector [1.0, 0.0, 0.0, 0.0]
end