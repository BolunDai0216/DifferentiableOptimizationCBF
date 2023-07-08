import StaticArrays as sa
import DifferentiableCollisions as dc

function three_blocks_exp_setup()
    block_A = sa.@SMatrix [1.0 0.0 0.0
        0.0 1.0 0.0
        0.0 0.0 1.0
        -1.0 0.0 0.0
        0.0 -1.0 0.0
        0.0 0.0 -1.0]

    block1_b = sa.@SVector [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    block2_b = sa.@SVector [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    block3_b = sa.@SVector [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

    global block1 = dc.Polytope(block_A, block1_b)
    global block2 = dc.Polytope(block_A, block2_b)
    global block3 = dc.Polytope(block_A, block3_b)

    block1.r = sa.@SVector [0.55, 0.1, 0.7]
    block1.q = sa.@SVector [1.0, 0.0, 0.0, 0.0]

    block2.r = sa.@SVector [0.65, -0.1, 0.3]
    block2.q = sa.@SVector [1.0, 0.0, 0.0, 0.0]

    block3.r = sa.@SVector [0.75, 0.0, 0.1]
    block3.q = sa.@SVector [1.0, 0.0, 0.0, 0.0]
end