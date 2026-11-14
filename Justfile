default:
    @just --list

# Install pinned toolchain via mise (run before `just install`).
install-tools:
    mise install

# Install Python + Julia deps (run `mise install` or `just install-tools` first).
install:
    uv sync
    uv run python -c "import juliacall"

# Run the unicycle obstacle-avoidance experiment.
run-unicycle *ARGS:
    uv run diffoptcbf-unicycle {{ARGS}}

# Run the two-walls FR3 experiment (opens the MuJoCo viewer).
run-two-walls *ARGS: (_run-fr3 "diffoptcbf-two-walls" ARGS)

# Run the three-blocks FR3 experiment (opens the MuJoCo viewer).
run-three-blocks *ARGS: (_run-fr3 "diffoptcbf-three-blocks" ARGS)

# On macOS the MuJoCo viewer only works under the `mjpython` launcher shipped
# with mujoco, which in turn has to be told where libpython (and, for
# juliacall, libjulia) live when it runs from a virtual environment.
[macos]
_run-fr3 SCRIPT *ARGS:
    DYLD_FALLBACK_LIBRARY_PATH="$(uv run python -c 'import sysconfig; print(sysconfig.get_config_var("LIBDIR"))')${PYTHON_JULIACALL_BINDIR:+:$PYTHON_JULIACALL_BINDIR/../lib}" \
        uv run mjpython "$(uv run which {{SCRIPT}})" {{ARGS}}

[linux]
[windows]
_run-fr3 SCRIPT *ARGS:
    uv run {{SCRIPT}} {{ARGS}}

# Run the test suite (slow on first run: Julia JIT).
test *ARGS:
    uv run pytest tests {{ARGS}}

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
