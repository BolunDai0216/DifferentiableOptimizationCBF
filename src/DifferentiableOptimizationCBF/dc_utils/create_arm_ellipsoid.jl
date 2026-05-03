import StaticArrays as sa
import DifferentiableCollisions as dc

function create_arm()
    #=
        Create all of the link objects.

        The ellipsoid function is: x^2/a^2 + y^2/b^2 + z^2/c^2 <= 1

        The P matrices are in the form of diag([1/(a*a), 1/(b*b), 1/(c*c)])
        where a, b, and c are the lengths of the semi-axes of the ellipsoid.

        The ellipsoid surface and interior can then be represented 
        as x^TPx <= 1, where x is a point on the surface of the ellipsoid.
    =#
    P3 = sa.@SMatrix [1/(0.165*0.165) 0.0 0.0
        0.0 1/(0.09*0.09) 0.0
        0.0 0.0 1/(0.09*0.09)]

    P4 = sa.@SMatrix [1/(0.15*0.15) 0.0 0.0
        0.0 1/(0.09*0.09) 0.0
        0.0 0.0 1/(0.09*0.09)]

    P5_1 = sa.@SMatrix [1/(0.14*0.14) 0.0 0.0
        0.0 1/(0.09*0.09) 0.0
        0.0 0.0 1/(0.09*0.09)]

    P5_2 = sa.@SMatrix [1/(0.125*0.125) 0.0 0.0
        0.0 1/(0.055*0.055) 0.0
        0.0 0.0 1/(0.055*0.055)]

    P6 = sa.@SMatrix [1/(0.11*0.11) 0.0 0.0
        0.0 1/(0.08*0.08) 0.0
        0.0 0.0 1/(0.08*0.08)]

    P7 = sa.@SMatrix [1/(0.14*0.14) 0.0 0.0
        0.0 1/(0.07*0.07) 0.0
        0.0 0.0 1/(0.07*0.07)]

    global link3 = dc.Ellipsoid(P3)
    global link4 = dc.Ellipsoid(P4)
    global link5_1 = dc.Ellipsoid(P5_1)
    global link5_2 = dc.Ellipsoid(P5_2)
    global link6 = dc.Ellipsoid(P6)
    global link7 = dc.Ellipsoid(P7)
    global hand = dc.Sphere(0.12)
end