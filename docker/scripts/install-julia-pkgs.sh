#!/bin/bash

set -e

cd /tmp/

# Install julia
wget https://julialang-s3.julialang.org/bin/linux/x64/1.10/julia-1.10.4-linux-x86_64.tar.gz
tar zxvf julia-1.10.4-linux-x86_64.tar.gz
mv julia-1.10.4 /opt/julia
ln -s /opt/julia/bin/julia /usr/local/bin/julia
rm -rf /tmp/julia-1.10.4-linux-x86_64.tar.gz

# Install requirements
export JULIA_DEPOT_PATH="/opt/julia/:$JULIA_DEPOT_PATH"
julia -e '
  using Pkg;
  Pkg.add.([
    "DifferentiableCollisions",
    "StaticArrays",
    "YAML",
    "PackageCompiler",
    "PyCall"
  ])'
