# Installation

This page describes how to install the project locally. The instructions are tested on Ubuntu and macOS.

The project uses [`mise`](https://mise.jdx.dev/) to pin the toolchain (Python 3.11, Julia 1.10, [`uv`](https://docs.astral.sh/uv/), [`just`](https://just.systems/)), [`uv`](https://docs.astral.sh/uv/) for Python dependencies (declared in `pyproject.toml`), and [`juliacall`](https://juliapy.github.io/PythonCall.jl/) for Julia dependencies (declared in `juliapkg.json`).

## Quickstart

`mise` is the only tool you need to install yourself — it then provisions Python, Julia, `uv`, and `just` for you. See the [mise install docs](https://mise.jdx.dev/getting-started.html) for alternatives such as Homebrew or apt.

```bash
# 1. Install mise (one-time)
curl https://mise.run | sh

# 2. Clone and enter the repo
git clone https://github.com/BolunDai0216/DifferentiableOptimizationCBF.git
cd DifferentiableOptimizationCBF

# 3. Trust the project's mise.toml and install the pinned toolchain
#    (Python 3.11, Julia 1.10, uv, just).
mise trust
mise install

# 4. Install Python + Julia project dependencies via just.
just install
```

`mise install` reads `mise.toml` and installs the pinned versions of Python, Julia, `uv`, and `just`. `just install` then runs `uv sync` to populate the Python virtual environment under `.venv/` and triggers `juliacall` so it resolves and installs the Julia packages declared in `juliapkg.json` into `.venv/julia_env/`.

To include documentation dependencies as well, run:

```bash
uv sync --extra docs
```

## Running Experiments

The project ships several `just` recipes that wrap the console entry points:

```bash
just run-unicycle             # 2D unicycle obstacle avoidance (no GUI)
just run-unicycle --show_plot # same, with matplotlib trajectory plot
just run-two-walls            # FR3 manipulator weaving between walls (MuJoCo viewer)
just run-three-blocks         # FR3 manipulator avoiding three blocks (MuJoCo viewer)
```

Run `just` (or `just --list`) to see every available recipe, including lint/format helpers (`just lint`, `just format`, `just check`) and `just clean` to remove the virtual environment.

You can also invoke the console scripts directly through `uv` if you prefer:

```bash
uv run diffoptcbf-unicycle
uv run python -m DifferentiableOptimizationCBF.unicycle_exp
```

## Notes for macOS users

- The FR3 experiments are simulated with [MuJoCo](https://mujoco.readthedocs.io/) 4. On macOS its interactive viewer only works when Python is started through the `mjpython` launcher that ships with the `mujoco` package; `just run-two-walls` and `just run-three-blocks` take care of this. If you start a script that opens the viewer yourself, use `uv run mjpython ...` instead of `uv run python ...` (the simulation environment raises an error explaining this otherwise). When `mjpython` fails with `Library not loaded: @rpath/libpython3.11.dylib`, add the directory printed by `uv run python -c 'import sysconfig; print(sysconfig.get_config_var("LIBDIR"))'` to `DYLD_FALLBACK_LIBRARY_PATH`, as the `just` recipes do.
- `fr3env` is only used for its robot description files (URDF, MJCF, meshes). Its `pybullet` and `ndcurves` requirements, which do not install cleanly on macOS arm64 and are not needed here, are dropped via `tool.uv.exclude-dependencies` in `pyproject.toml`.
- `proxsuite` is pinned to `0.6.2` because newer macOS arm64 wheels (`>=0.6.5`) ship with broken RPATHs from the upstream CI build pipeline. Bump the pin once upstream republishes correctly relocated wheels.
