import StaticArrays as sa

function update_arm(rs::Vector{Float64}, qs::Vector{Float64})
    #=
        This function updates the position and 
        orientation of all of the robot links.
        This function requires all of the links
        be already initialized.
    =#
    link3.r = sa.SVector{3}(rs[1:3])
    link3.q = sa.SVector{4}(qs[1:4])

    link4.r = sa.SVector{3}(rs[4:6])
    link4.q = sa.SVector{4}(qs[5:8])

    link5_1.r = sa.SVector{3}(rs[7:9])
    link5_1.q = sa.SVector{4}(qs[9:12])

    link5_2.r = sa.SVector{3}(rs[10:12])
    link5_2.q = sa.SVector{4}(qs[13:16])

    link6.r = sa.SVector{3}(rs[13:15])
    link6.q = sa.SVector{4}(qs[17:20])

    link7.r = sa.SVector{3}(rs[16:18])
    link7.q = sa.SVector{4}(qs[21:24])

    hand.r = sa.SVector{3}(rs[19:21])
    hand.q = sa.SVector{4}(qs[25:28])
end