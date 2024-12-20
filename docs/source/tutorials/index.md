# Tutorials

To faciliate the use of our package, we provide a few tutorials to demonstrate the usage of our package:

- [Using `DifferentiableCollisions.jl` in Julia and Python](julia.md)
- [Controller implementation](controller.md)

To run the three simulation examples, first following the installation process, then `cd` into the folder `/path/to/DifferentiableOptimizationCBF/DifferentiableOptimizationCBF` (**change `/path/to/` to the path you cloned `DifferentiableOptimizationCBF` into**) and run the following commands in the terminal:

```bash
# For the unicycle example, run the following command
python3 unicycle_exp.py

# For the three-blocks example, run the following command
python3 three_blocks_exp.py

# For the two-walls example, run the following command
python3 two_walls_exp.py
```

Note that because of the JIT compilation of Julia, it takes a while to start the simulation.

## FAQ

- If you see the following error message `Your Python interpreter /usr/bin/python3 is statically linked to libpython. Currently, PyJulia does not fully support such Python interpreter.` Try changing `python3` to `python-jl`.
- If you see `Cannot connect to X server`, on the host machine try running `xhost +`.

```{toctree}
:hidden:

julia
controller
```