#!/bin/bash
# =================================================================================================
# DNA shell requirements install
#
# Notes:
# - This file is used in DNA Dockerfile.project-core-pre
# - It is executed before python.requirements-dna.txt
# - N2ST library is available in script i.e., shell script function prefixed 'n2st::'
#
# =================================================================================================

# ADD YOUR CODE HERE
# ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓
# ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓

# ....Example 1: Update pip to latest..............................................................
python3 -m pip install --upgrade pip

# ....Example 2: Package install...................................................................
{
  apt-get update \
  && apt-get install --assume-yes --no-install-recommends \
    vim \
    tree \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/* ;
} || n2st::print_msg_error_and_exit "Failed apt-get package install!"

# ....Example 3: Host specific logic...............................................................
if [[ $( n2st::which_architecture_and_os ) == "l4t\arm64" ]]; then
  n2st::print_msg "Is running on a Jetson..."
  # Add Jetson logic e.g., cat /proc/device-tree/model
fi

# ....Example 4: Python version specific logic.....................................................
if [[ $( n2st::which_python3_version ) == 3.10 ]]; then
  n2st::print_msg "Execute python 3.10 specialized install..."
  # Add python logic
fi

# ....Examples 5: Python package specific logic with alias declaration.............................
# Note: Assuming the base image is nvidia cuda enable and has pycuda and pytorch installed
if pip -qq show torch; then
    {
      echo "..........................................." && \
      echo "Sanity check" && \
      python -c "import torch" && \
      echo "..........................................." ;
    } || n2st::print_msg_error_and_exit "Failed torch sanity check!"
    (
      echo
      echo "alias dn-pytorch-cuda-check='python3 /ros2_ws/src/dockerized-norlab-project-mock/src/dna_example/try_pytorch.py'"
      echo
    ) >> /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash
fi

if pip -qq show pycuda; then
    {
      echo "..........................................." && \
      echo "Sanity check" && \
      python -c "import pycuda" && \
      echo "..........................................." ;
    } || n2st::print_msg_error_and_exit "Failed pycuda sanity check!"
    (
      echo
      echo "alias dn-pycuda-check='python3 /ros2_ws/src/dockerized-norlab-project-mock/src/dna_example/try_pycuda.py'"
      echo
    ) >> /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash
fi
