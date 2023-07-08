import DifferentiableCollisions as dc

function create_arm()
    #=
        Create all of the link objects.
    =#
    global link3 = dc.Capsule(0.09, 0.15)
    global link4 = dc.Capsule(0.09, 0.12)
    global link5_1 = dc.Capsule(0.09, 0.10)
    global link5_2 = dc.Capsule(0.055, 0.14)
    global link6 = dc.Capsule(0.08, 0.06)
    global link7 = dc.Capsule(0.07, 0.14)
    global hand = dc.Sphere(0.12)
end