using PackageCompiler
create_sysimage(
    ["DifferentiableCollisions", "StaticArrays"], 
    sysimage_path="unicycle_sysimage.so",
    precompile_execution_file="unicycle_precompile.jl",
)