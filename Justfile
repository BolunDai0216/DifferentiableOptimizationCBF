default:
    @just --list

# Install pinned toolchain (python, julia, uv, just) via mise.
install-tools:
    mise install

# Sync Python deps and trigger first-time Julia env resolution.
install: install-tools
    uv sync
    uv run python -c "import juliacall"

# Run the unicycle obstacle-avoidance experiment.
run-unicycle *ARGS:
    uv run diffoptcbf-unicycle {{ARGS}}

# Run the two-walls FR3 experiment (needs pybullet GUI).
run-two-walls:
    uv run diffoptcbf-two-walls

# Run the three-blocks FR3 experiment (needs pybullet GUI).
run-three-blocks:
    uv run diffoptcbf-three-blocks

# Run the unicycle experiment using a precompiled Julia sysimage.
run-unicycle-sysimage *ARGS:
    uv run diffoptcbf-unicycle-sysimage {{ARGS}}

# Build the Julia sysimage for the unicycle experiment (~5 min).
build-sysimage:
    cd src/DifferentiableOptimizationCBF/dc_utils && \
        julia --project={{justfile_directory()}}/.venv/julia_env unicycle_sysimage.jl

# Lint with ruff (autofixable issues fixed in place).
lint:
    uv run ruff check --fix .

# Format with ruff.
format:
    uv run ruff format .

# Run lint + format checks without modifying files (CI mode).
check:
    uv run ruff check .
    uv run ruff format --check .

# Remove caches, venv, and Julia env. `just install` to recreate.
clean:
    rm -rf .venv dist build *.egg-info
    find . -type d -name __pycache__ -prune -exec rm -rf {} +
