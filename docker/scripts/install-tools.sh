#!/bin/bash

set -e

# Install tools, utilities, and etc.
apt-get update
apt-get install -y --no-install-recommends \
    curl \
    freeglut3-dev \
    git \
    git-lfs \
    iputils-ping \
    libglvnd-dev \
    libegl1-mesa-dev \
    libgles2-mesa-dev \
    libgl1-mesa-dev \
    libxext6 \
    libx11-6 \
    net-tools \
    ninja-build \
    psmisc \
    python3-pip \
    software-properties-common \
    unzip \
    usbutils \
    vim \
    wget \
    zsh