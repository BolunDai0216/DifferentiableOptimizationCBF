# $\textsf{\color{BlueViolet}{Differentiable Optimization Based CBFs}}$

[![License](https://img.shields.io/badge/License-BSD--3-cfd8dc?style=flat-square&labelColor=darkblue&color=lightgray)](https://github.com/BolunDai0216/DifferentiableOptimizationCBF/blob/main/LICENSE)
[![Documentation](https://img.shields.io/badge/Documentation-darkblue?style=flat-square&logo=readthedocs&logoColor=white)](https://differentiableoptimizationcbf.readthedocs.io/en/latest/)

This repo contains the codebase of [**Safe Navigation and Obstacle Avoidance Using Differentiable Optimization Based Control Barrier Functions**](https://arxiv.org/abs/2304.08586). For an overview of the paper and a detailed walkthrough of the codebase, please refer to the [official paper website](https://differentiableoptimizationcbf.readthedocs.io/).

## Quickstart

This project uses [`mise`](https://mise.jdx.dev/) to pin Python, Julia, [`uv`](https://docs.astral.sh/uv/), and [`just`](https://just.systems/) to known-good versions. `mise` is the only thing you need to install yourself — everything else (including `just`) gets installed by `mise install`.

```bash
# 1. Install mise (one-time, see https://mise.jdx.dev/getting-started.html for alternatives)
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

`mise install` reads `mise.toml` and installs the pinned versions of Python, Julia, `uv`, and `just`. `just install` then runs `uv sync` to set up the Python venv at `.venv/` and triggers `juliacall` to resolve the Julia environment under `.venv/julia_env/` from `juliapkg.json`.

## Running experiments

```bash
just run-unicycle             # 2D unicycle obstacle avoidance (no GUI)
just run-unicycle --show_plot # same, with matplotlib trajectory plot
just run-two-walls            # FR3 manipulator weaving between walls (MuJoCo viewer)
just run-three-blocks         # FR3 manipulator avoiding three blocks (MuJoCo viewer)
```

To list every available recipe: `just` (or `just --list`).

## Common tasks

| Recipe                          | What it does                                                   |
| ------------------------------- | -------------------------------------------------------------- |
| `just install-tools`            | Install pinned python/julia/uv/just via mise (alias for `mise install`) |
| `just install`                  | `uv sync` + first-time Julia env resolution                    |
| `just run-unicycle [ARGS]`      | Run the unicycle experiment                                    |
| `just run-two-walls`            | Run the two-walls FR3 experiment                               |
| `just run-three-blocks`         | Run the three-blocks FR3 experiment                            |
| `just lint` / `just format`     | Lint/format Python with `ruff` (config in `pyproject.toml`)    |
| `just check`                    | CI-style lint+format check (read-only)                         |
| `just clean`                    | Remove `.venv`, `dist`, build artifacts, `__pycache__`         |

## Notes for macOS users

- The FR3 experiments are simulated with [MuJoCo](https://mujoco.readthedocs.io/) 4. On macOS its interactive viewer only works when Python is started through the `mjpython` launcher that ships with the `mujoco` package; `just run-two-walls` and `just run-three-blocks` take care of this. If you start a script that opens the viewer yourself, use `uv run mjpython ...` instead of `uv run python ...` (the simulation environment raises an error explaining this otherwise). When `mjpython` fails with `Library not loaded: @rpath/libpython3.11.dylib`, add the directory printed by `uv run python -c 'import sysconfig; print(sysconfig.get_config_var("LIBDIR"))'` to `DYLD_FALLBACK_LIBRARY_PATH`, as the `just` recipes do.
- `fr3env` is only used for its robot description files (URDF, MJCF, meshes). Its `pybullet` and `ndcurves` requirements, which do not install cleanly on macOS arm64 and are not needed here, are dropped via `tool.uv.exclude-dependencies` in `pyproject.toml`.
- `proxsuite` is pinned to `0.6.2` because newer macOS arm64 wheels (`>=0.6.5`) ship with broken RPATHs from the upstream CI. See the comment in `pyproject.toml`.

## 📖 $\textsf{\large\color{cornflowerblue}{Citation}}$

To cite our paper, please use the following bibtex

```bibtex
@article{DaiKKGTK23,
  author       = {Bolun Dai and Rooholla Khorrambakht and Prashanth Krishnamurthy and Vin{\'{\i}}cius Gon{\c{c}}alves and Anthony Tzes and Farshad Khorrami},
  title        = {Safe Navigation and Obstacle Avoidance Using Differentiable Optimization Based Control Barrier Functions},
  journal      = {{IEEE} Robotics and Automation Letters},
  year         = {2023},
  volume       = {8},
  number       = {9},
  pages        = {5376-5383},
}
```
