# Installation

This part of the tutorial provides steps required to run the provided code and tutorials. The following steps are only tested on Ubuntu and MacOS.

## Dev Container (Recommended, but only for Linux Machines)

To use the dev container, first install docker and the VS Code devcontainer extension: `ms-vscode-remote.remote-containers`. Then, go to the `provisioning` folder and run `bash setup.sh`, which will setup the `.zsh_history` file for the devcontainer which will make the zsh command history persistent over docker builds.

After these steps, open a VS Code window in the `DifferentiableOptimizationCBF` directory and press `Shift + Ctrl + P` in VS Code, which will open up the command palette, in the command palette type/search for `Dev Containers: Rebuild and Reopen in Container`. This will start the process of building the devcontainer.

## Local Installation

Below are the steps to install the required dependencies for the code to run locally.

### Install non-statically linked Python (Optional, not tested on Mac OS)

First, we need to install a Python interpreter that is **not** statically linked to libpython. To do this we use `pyenv`. First, install the dependencies (or follow the instructions [here](https://realpython.com/intro-to-pyenv/#build-dependencies)):

```console
sudo apt-get install -y make build-essential libssl-dev zlib1g-dev \
libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm libncurses5-dev \
libncursesw5-dev xz-utils tk-dev libffi-dev liblzma-dev python-openssl
```

Then, use the `pyenv-installer` to install `pyenv`

```console
curl https://pyenv.run | zsh
```

Finally, add these lines to `.zshrc`

```text
# pyenv
export PYENV_ROOT="$HOME/.pyenv"
command -v pyenv >/dev/null || export PATH="$PYENV_ROOT/bin:$PATH"
eval "$(pyenv init -)"
eval "$(pyenv virtualenv-init -)"
```

Now that `pyenv` is installed, we install Python 3.9.16 using the command

```console
PYTHON_CONFIGURE_OPTS="--enable-shared" pyenv install 3.9.16
```

And we can switch to the install Python version using

```console
pyenv global 3.9.16
exec "$SHELL"
```

### Install Julia and Julia dependencies

We can download the Julia binaries using the commands

```bash
wget https://julialang-s3.julialang.org/bin/linux/x64/1.8/julia-1.8.3-linux-x86_64.tar.gz
tar zxvf julia-1.8.3-linux-x86_64.tar.gz
```

Then, simply add the line

```text
export PATH="$PATH:/path/to/<Julia directory>/bin"
```

to your `.zshrc` file. Start Julia using the command `julia`, and press `]` to open the package manager. We need to install `DifferentiableCollisions.jl` and `StaticArrays.jl`. We can do that by using the command

```bash
add DifferentiableCollisions
add StaticArrays
```

in the package manager.

### Install Python dependencies

We can then install the dependencies of `DiffOptCBF`. First, install pinochhio and proxsuite using

```bash
python3 -m pip install pin
python3 -m pip install proxsuite
```

Then clone `FR3Env` and install it using

```bash
git clone https://github.com/BolunDai0216/FR3Env.git
cd FR3Env
python3 -m pip install -e .
```

The next step is to install `PyJulia`. We can install `PyJulia` using

```bash
python3 -m pip install julia
```

Then, we can install `PyCall` in Julia, using

```python
import julia
julia.install()
```

### Install DifferentiableOptimizationCBF

First, clone the package 

```bash
git clone https://github.com/BolunDai0216/DifferentiableOptimizationCBF.git
```

then install it using

```bash
cd DifferentiableOptimizationCBF
python3 -m pip install -e .
```

## Using Julia System Images

One way to reduce the Julia startup time is to use system images. We provide an example of doing so for the unicycle experiments. To compile the system image, run the following command

```bash
cd /path/to/dc_utils && julia unicycle_sysimage.jl
```

To use the system image see the example in [`unicycle_exp_sysimage.py`](https://github.com/BolunDai0216/DifferentiableOptimizationCBF/blob/main/DifferentiableOptimizationCBF/unicycle_exp_sysimage.py).
