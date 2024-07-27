#!/bin/bash

set -e

# Install the Go2Py in editable mode
cd /home && git clone https://github.com/BolunDai0216/FR3Env.git && cd FR3Env && python3 -m pip install -e .

# Install Python dependencies
pip3 install \
    black \
    ipykernel \
    isort \
    julia \
    matplotlib