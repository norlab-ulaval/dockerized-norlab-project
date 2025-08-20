# =================================================================================================
#
#   👍  You can change the code in this file to take advantage of the Docker cache layer mechanism
#       and the Docker multi-stage build feature.
#
# =================================================================================================
FROM base_image AS user-project-custom-steps

ARG TARGETPLATFORM
ARG BUILDPLATFORM
WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}

# ADD YOUR CODE HERE
# ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓
# ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓

# Example
RUN <<EOF
    # ....Examples.................................................................................
    {
        echo "...Sanity check............................" && \
        echo "python version: $(n2st::which_python3_version)" && \
        python -c "import pytest" && \
        echo "..........................................." ;
    } || n2st::print_msg_error_and_exit "Failed sanity check!"
EOF

RUN <<EOF
    # ....Examples with cuda.......................................................................
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
EOF
