import StaticArrays as sa
import DifferentiableCollisions as dc

function unicycle_env_setup()
    obstacle_A = sa.@SMatrix [1.0 0.0 0.0
        0.0 1.0 0.0
        0.0 0.0 1.0
        -1.0 0.0 0.0
        0.0 -1.0 0.0
        0.0 0.0 -1.0]
    obstacle_B = sa.@SVector [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    # obstacle_B = sa.@SVector [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]

    global obstacle1 = dc.Polytope(obstacle_A, obstacle_B)
    global obstacle2 = dc.Polytope(obstacle_A, obstacle_B)

    obstacle1.r = sa.@SVector [0.0, 0.0, 0.0]
    obstacle1.q = sa.@SVector [1.0, 0.0, 0.0, 0.0]

    obstacle2.r = sa.@SVector [4.0, 0.0, 0.0]
    # obstacle2.r = sa.@SVector [3.5, 0.5, 0.0]
    obstacle2.q = sa.@SVector [1.0, 0.0, 0.0, 0.0]

    global unicycle = dc.Capsule(0.3, 0.5)

    unicycle.r = sa.@SVector [0.0, -3.0, 0.0]
    unicycle.q = sa.@SVector [1.0, 0.0, 0.0, 0.0]

end