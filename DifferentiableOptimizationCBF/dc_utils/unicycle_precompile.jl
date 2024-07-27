using DifferentiableCollisions
using StaticArrays

# load the functions to precompile
include("unicycle_env_setup.jl")
include("get_cbf_unicycle_env.jl")

# run the functions
unicycle_env_setup()
rs = [0.0, 0.0, 0.0]
qs = [1.0, 0.0, 0.0, 0.0]
get_cbf_unicycle_env(rs, qs)