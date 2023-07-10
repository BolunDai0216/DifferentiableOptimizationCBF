# Installation

This part of the tutorial provides steps required to run the provided code and tutorials.

## Install Julia and Julia dependencies

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

## Install Python dependencies

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