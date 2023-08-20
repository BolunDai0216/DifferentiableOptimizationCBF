# Tutorials

To faciliate the use of our package, we provide a few tutorials to demonstrate the usage of our package:

- Using `DifferentiableCollisions.jl` in Julia and Python
- Controller implementation

To run the two simulation examples, first following the installation process, then run the following commands in the terminal:

```bash
# three-blocks example
python3 three_blocks_exp.py

# two-walls example
python3 two_walls_exp.py
```

Note that because of the JIT compilation of Julia, it takes a while to start the simulation.

```{toctree}
:hidden:

julia
controller
```