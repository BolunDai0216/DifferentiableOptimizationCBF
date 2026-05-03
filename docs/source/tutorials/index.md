# Tutorials

To faciliate the use of our package, we provide a few tutorials to demonstrate the usage of our package:

- [Using `DifferentiableCollisions.jl` in Julia and Python](julia.md)
- [Controller implementation](controller.md)

To run the three simulation examples, first follow the installation process, then run the following commands from the repository root:

```bash
# For the unicycle example, run the following command
uv run diffoptcbf-unicycle

# For the three-blocks example, run the following command
uv run diffoptcbf-three-blocks

# For the two-walls example, run the following command
uv run diffoptcbf-two-walls
```

Note that because of the JIT compilation of Julia, it takes a while to start the simulation.

## FAQ

- If Julia/Python interop reports a statically linked Python interpreter, make sure you are running inside the `uv` environment created by `uv sync`.
- If you see `Cannot connect to X server`, on the host machine try running `xhost +`.

```{toctree}
:hidden:

julia
controller
```
