FROM nvidia/cuda:12.1.1-devel-ubuntu22.04

SHELL ["/bin/bash", "-c"]

# Install dependencies
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=bind,source=docker/scripts/install-tools.sh,target=/tmp/install-tools.sh \
    bash /tmp/install-tools.sh
RUN --mount=type=cache,target=/var/cache/pip \
    --mount=type=bind,source=docker/scripts/install-python-pkgs.sh,target=/tmp/install-python-pkgs.sh \
    bash /tmp/install-python-pkgs.sh
RUN --mount=type=cache,target=/var/cache/pip \
    --mount=type=bind,source=docker/scripts/install-julia-pkgs.sh,target=/tmp/install-julia-pkgs.sh \
    bash /tmp/install-julia-pkgs.sh

# Clean cache
RUN apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace directory
RUN mkdir -p /workspace/diffoptcbf-devcontainer/src/diffoptcbf-devcontainer \
    && cd /workspace/diffoptcbf-devcontainer

# Default powerline10k theme, no plugins installed
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.0/zsh-in-docker.sh)"

RUN --mount=type=bind,source=docker/config/docker.zshrc,target=/tmp/docker.zshrc \
    cat /tmp/docker.zshrc >> $HOME/.zshrc

# NVIDIA-related environment variables
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics