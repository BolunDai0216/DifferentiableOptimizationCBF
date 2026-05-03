# Installation

This part of the tutorial provides steps required to run the provided code and tutorials. The following steps are only tested on Ubuntu and MacOS.

## Dev Container (Recommended, but only for Linux Machines)

To use the dev container, first install docker and the VS Code devcontainer extension: `ms-vscode-remote.remote-containers`. Then, go to the `provisioning` folder and run `bash setup.sh`, which will setup the `.zsh_history` file for the devcontainer which will make the zsh command history persistent over docker builds.

After these steps, open a VS Code window in the `DifferentiableOptimizationCBF` directory and press `Shift + Ctrl + P` in VS Code, which will open up the command palette, in the command palette type/search for `Dev Containers: Rebuild and Reopen in Container`. This will start the process of building the devcontainer.

## Local Installation

Below are the steps to install the required dependencies for the code to run locally. The project uses `uv`, Python 3.11, and `juliacall`. Python dependencies are declared in `pyproject.toml`, and Julia dependencies are declared in `juliapkg.json`.

First, clone the package and enter the repository:

```bash
git clone https://github.com/BolunDai0216/DifferentiableOptimizationCBF.git
cd DifferentiableOptimizationCBF
```

If you use `mise`, install the pinned toolchain with:

```bash
mise install
```

Otherwise, install `uv`, Python 3.11, and Julia 1.10. Then synchronize the Python environment:

```bash
uv sync
```

To include documentation dependencies:

```bash
uv sync --extra docs
```

The package uses a standard `src/` layout. Run modules or console entrypoints through `uv` from the repository root, for example:

```bash
uv run diffoptcbf-unicycle
uv run python -m DifferentiableOptimizationCBF.unicycle_exp
```

## Using Julia System Images

One way to reduce the Julia startup time is to use system images. We provide an example of doing so for the unicycle experiments. To compile the system image, run the following command

```bash
cd /path/to/DifferentiableOptimizationCBF/src/DifferentiableOptimizationCBF/dc_utils
julia unicycle_sysimage.jl
```

To use the system image see the example in [`unicycle_exp_sysimage.py`](https://github.com/BolunDai0216/DifferentiableOptimizationCBF/blob/main/src/DifferentiableOptimizationCBF/unicycle_exp_sysimage.py).
