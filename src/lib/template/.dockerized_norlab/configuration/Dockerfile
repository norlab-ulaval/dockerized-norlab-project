# =================================================================================================
#
#   👍  You can change the code in this file to take advantage of the Docker cache layer mechanism
#       and the Docker multi-stage build feature.
#
# =================================================================================================
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS user-project-custom-steps

ARG TARGETPLATFORM
ARG BUILDPLATFORM
WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}

# ADD YOUR CODE HERE
# ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓
# ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓

# ....Example......................................................................................
RUN <<EOF
    {
        echo "...Sanity check............................" && \
        echo "python version: $(n2st::which_python3_version)" && \
        python -c "import pytest" && \
        echo "..........................................." ;
    } || n2st::print_msg_error_and_exit "Failed sanity check!"
EOF

# ....Recipies: Install python project from pyproject.toml.........................................
# Recommended approach for modern python centric project.
# Note:
#  - If you are using poetry, uv, hatch or any other python dependency management tools, just make
#    sure your install destination match the container default python3. Python install in a docker
#    container usualy don't rely in virtual environment, especialy when dealing with ros and cuda.
#  - (pro) Take advantage of dedicated tools for building and distribution.
#  - (con) Requires copying project source code before installing python package in editable mode.
#    Remark that 'DN_PROJECT_PATH' will be overwritten at runtime in dna develop image because
#    'SUPER_PROJECT_ROOT' is mounted to 'DN_PROJECT_PATH'.
# ↓ ↓ ↓ UNMUTE ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓
#COPY  . ${DN_PROJECT_PATH:?err}/
#RUN <<EOF
#  echo -e "python version: $(n2st::which_python3_version)\ndefault python path: $(which python)\n"
#  n2st::print_msg "Install project via pyproject.toml..."
#  cd "${DN_PROJECT_PATH}"
#  python -m pip install --editable . \
#    || n2st::print_msg_error_and_exit "Failed project install via pyproject.toml!"
#EOF


# ....Recipies: Install python project from requirement.txt file...................................
# Recommended only for python project dependency management requiring special handling
# Note:
#  - (pro) No need to copy the src code in the container before install
#  - (con) Require handling building, python path and distribution manualy
#  - Requires setting PYTHONPATH explicitly in the container
# ↓ ↓ ↓ UNMUTE ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓
#ENV PYTHONPATH=${DN_PROJECT_PATH}/src${PYTHONPATH:+:${PYTHONPATH}}
#COPY ./requirements.txt "${DN_PROJECT_PATH:?err}/requirements.txt"
#RUN <<EOF
#  n2st::print_msg "Execute pip install from repository root python requirement file..."
#  python -m pip install --verbose -r "${DN_PROJECT_PATH}/requirements.txt" \
#    || n2st::print_msg_error_and_exit "Failed pip install from repository root requirement file!"
#EOF

# ....Recipies: Build ROS workspace................................................................
# Note:
#  - Requires installing python project first
#  - Requires copying ros source code before building and installing ros package.
#    Remark that 'DN_PROJECT_PATH' will be overwritten at runtime in dna develop image because
#    'SUPER_PROJECT_ROOT' is mounted to 'DN_PROJECT_PATH'.
#  - 'DN_DEV_WORKSPACE' is the container ros workspace.
#  - 'DN_PROJECT_PATH' is symlinked to '$DN_DEV_WORKSPACE/src/$DN_PROJECT_PATH'.
COPY  . ${DN_PROJECT_PATH:?err}/
RUN <<EOF
  n2st::print_msg "Build/install ROS project..."
  cd "${DN_DEV_WORKSPACE:?'environment variable is not set'}"
  INSTALL_FROM_PATH="${DN_DEV_WORKSPACE:?err}/src"
  COLCON_BUILD_FLAGS=()
  #COLCON_BUILD_FLAGS+=( "ANY_COLCON_BUILD_FLAG" )
  #COLCON_BUILD_FLAGS+=("--packages-select" MY_ROS_PKG)
  source /dna-lib-container-tools/dn_project_core.build.aarch_aware_build_ros.bash \
            "${TARGETPLATFORM}" "${BUILDPLATFORM}" \
            "${INSTALL_FROM_PATH}" "${COLCON_BUILD_FLAGS[@]}" \
    || n2st::print_msg_error_and_exit "Failed to build repository ros pkgs!"
EOF
