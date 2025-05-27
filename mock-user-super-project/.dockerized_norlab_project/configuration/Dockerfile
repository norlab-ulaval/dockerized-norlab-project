
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS project-setup

ARG IS_TEAMCITY_RUN
ENV IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN:-false}

ARG DEBIAN_FRONTEND=noninteractive

# ====Dockerized-NorLab-project internal configuration step========================================

ARG DN_PROJECT_GIT_NAME
ENV DN_PROJECT_GIT_NAME=${DN_PROJECT_GIT_NAME:?'Build argument needs to be set and non-empty.'}
# Note: `DN_PROJECT_GIT_NAME` is required by the `/develop/ros2_rebuild_dev_workspace.bash` script
ARG DN_PROJECT_GIT_DOMAIN
ENV DN_PROJECT_GIT_DOMAIN=${DN_PROJECT_GIT_DOMAIN:?'Build argument needs to be set and non-empty.'}

ARG DN_PROJECT_ALIAS_PREFIX
ENV DN_PROJECT_ALIAS_PREFIX=${DN_PROJECT_ALIAS_PREFIX:?'Build argument needs to be set and non-empty.'}

ARG DN_PROJECT_USER
ENV DN_PROJECT_USER=${DN_PROJECT_USER:?'Build argument needs to be set and non-empty.'}
ARG DN_PROJECT_UID
ENV DN_PROJECT_UID=${DN_PROJECT_UID:?'Build argument needs to be set and non-empty.'}
ARG DN_PROJECT_GID
ENV DN_PROJECT_GID=${DN_PROJECT_GID:?'Build argument needs to be set and non-empty.'}

ENV DN_PROJECT_USER_HOME=/home/${DN_PROJECT_USER}
ENV DN_PROJECT_PATH=${DN_DEV_WORKSPACE:?'Build argument needs to be set and non-empty.'}/src/${DN_PROJECT_GIT_NAME}

COPY --from=context-dn-container-tools ./dn_project_core_init.bash .
RUN <<EOF
    source ./dn_project_core_init.bash
    rm ./dn_project_core_init.bash
    { test -d ${DN_PROJECT_USER_HOME} && test -d ${DN_PROJECT_PATH} ; } || exit 1
EOF


# ====User project custom steps====================================================================
FROM project-setup AS project-custom-docker-steps
WORKDIR ${DN_PROJECT_PATH:?err}

# ....Install requirement from file................................................................
# Doc › pip install flag: https://pip.pypa.io/en/stable/cli/pip_install/#options

# (Priority) ToDo: refactor to dn_project_core_init.bash (ref task RLRP-213 and NMO-548)
COPY .dockerized_norlab_project/configuration/project_requirements/python.requirements.txt /python.requirements.txt
COPY .dockerized_norlab_project/configuration/project_requirements/shell.requirements.bash /shell.requirements.bash

RUN <<EOF
  pip3 install --verbose \
    -r /python.requirements.txt \
    && rm -f /python.requirements.txt

  source /shell.requirements.bash \
    && rm -f /shell.requirements.bash
EOF


# ====Python required tools sanity check===========================================================
FROM project-custom-docker-steps AS python-required-tools-sanity-check
WORKDIR "/"

RUN <<EOF
    {
        python -c "import torch" && \
        python -c "import torchvision" && \
    } || exit 1
EOF

# ====Simulator====================================================================================
FROM project-custom-docker-steps AS simulator

# ----Install f1tenth_gym_ros bridge---------------------------------------------------------------

RUN <<EOF
    apt-get update
    apt-get update --fix-missing
    apt-get upgrade -y
    apt-get install -y \
        ros-${ROS_DISTRO:?err}-rmw-cyclonedds-cpp
    apt-get autoremove -y
    apt-get clean
    echo "Cyclon DDS performance recommendations (ref https://github.com/ros2/rmw_cyclonedds?tab=readme-ov-file)"
    echo "net.core.rmem_max=8388608\nnet.core.rmem_default=8388608\n" | sudo tee /etc/sysctl.d/60-cyclonedds.conf
EOF

FROM --platform=${TARGETPLATFORM} simulator AS build-ros
ARG TARGETPLATFORM
ARG BUILDPLATFORM

## (CRITICAL) Quick-hack ToDo: validate deleting 02-dn-source-ros-system-wide.sh (ref task NMO-540)
#RUN rm /etc/profile.d/02-dn-source-ros-system-wide.sh
#ENV BASH_ENV=""

# (NICE TO HAVE) ToDo: refactor as a DN library fct with
RUN <<EOF
    echo "sourcing /opt/ros/${ROS_DISTRO}/setup.bash"
    source /opt/ros/${ROS_DISTRO}/setup.bash
    echo "sourcing ${DN_DEV_WORKSPACE}/install/setup.bash"
    source ${DN_DEV_WORKSPACE}/install/setup.bash
    apt-get update --fix-missing

    rosdep update --rosdistro ${ROS_DISTRO} --include-eol-distros
    rosdep fix-permissions

    rosdep install \
            --ignore-packages-from-source \
            --from-path src  \
            --rosdistro ${ROS_DISTRO}  \
            -r \
            -q \
            -y

    colcon version-check
    COLCON_FLAGS=()
    if [[ ${TARGETPLATFORM:?err} != ${BUILDPLATFORM:?err} ]]; then
        echo -e "Builder is running in architecture virtualisation"
        COLCON_FLAGS+=("--executor" "sequential")
    else
        echo -e "Builder is running on native architecture"
        COLCON_FLAGS+=("--symlink-install")
    fi

    COLCON_FLAGS+=(
          "--cmake-clean-cache"
          "--cmake-args" "-DCMAKE_BUILD_TYPE=Release"
          "--event-handlers" "console_direct+"
       )
    echo -e "COLCON_FLAGS=("${COLCON_FLAGS[*]}")"
    colcon build ${COLCON_FLAGS[@]}
EOF



# ====Final========================================================================================
FROM build-ros AS project-env

RUN <<EOF
    echo "Add project specific aliases"
    (
      echo ""
      echo "# Project specific aliases (general)"
      echo "alias dnp${DN_PROJECT_ALIAS_PREFIX:?err}_cd='cd ${DN_PROJECT_PATH:?err}'"
      echo "alias dnp${DN_PROJECT_ALIAS_PREFIX:?err}_cdd='cd ${DN_PROJECT_PATH:?err}/src'"
      echo "alias dnp${DN_PROJECT_ALIAS_PREFIX:?err}_cdt='cd ${DN_PROJECT_PATH:?err}/tests'"
      echo "alias dnp${DN_PROJECT_ALIAS_PREFIX:?err}_cde='cd ${DN_PROJECT_PATH:?err}/external_data'"
      echo ""
    ) >> /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash
EOF

# ....Set runtime environment variable.............................................................
ENV HYDRA_FULL_ERROR=1
ENV PYTHONUNBUFFERED=1
ENV HYDRA_FULL_ERROR=1
ENV PYCHARM_DEBUG=1
#ENV PYTEST_DEBUG=1
ENV OC_CAUSE=1
# OC_CAUSE: Set omegaconf full error backtrace

FROM project-env AS dn-project-tmp-dev

# //// To refactor ////////////////////////////////////////////////////////////////////////////////
# (StandBy) ToDo: refactor >> transfer to Dockerized-NorLab project
# Temporary hack (might be resolved now)
# ref issues
#  - https://github.com/ipython/ipython/issues/14390
#  - https://github.com/ros2/launch/issues/765
RUN pip3 install 'pytest==8.0'

# \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ To refactor \\\\

# //// To refactor //////////////////////////////////////////////////////////////////////////////
# (CRITICAL) ToDo: refactor >> transfer to Dockerized-NorLab project (ref task NMO-521)
# PATH related environment variable
# Note: DN_PROJECT_PATH=/ros2_ws/src/RedLeader-research-codebase
ENV PATH="${DN_PROJECT_PATH}:${PATH}"
# PYTHONPATH related environment variable
ENV PYTHONPATH="${DN_PROJECT_PATH}/src:${DN_PROJECT_PATH}/tests:${DN_PROJECT_PATH}/external_data:${DN_PROJECT_PATH}/.dockerized_norlab_project/python_dev_tools:${PYTHONPATH}"
# (NICE TO HAVE) ToDo: add `PYTHONPATH=${DN_PROJECT_PATH:?err}:$PYTHONPATH` for pytest execution from command line of PyCharm project
# \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ To refactor \\\\

# //// To refactor ////////////////////////////////////////////////////////////////////////////////
# For Matplotlib default backend (QtAgg) when python script are launched from terminal
RUN mkdir -m 0700 -p "/tmp/runtime-root" && chown -R ${DN_PROJECT_USER} "/tmp/runtime-root"
ENV XDG_RUNTIME_DIR='/tmp/runtime-root'
# \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ To refactor \\\\

# ====DN-project final=============================================================================
FROM dn-project-tmp-dev AS final

# (Priority) ToDo: refactor project_entrypoints logic as a fct. Either in DN container-tools or in DN-project
# Note: project-develop container mounth those directories as a volume to skip rebuilding
#       each time they are modify (see docker-compose.project.run.*.yaml). However, project-deploy
#       and project release container will need them copied in the image for portability.
# Note: Files copied from project_entrypoints/ directory are required by Dockerized-NorLab
#       dn_entrypoint.init.bash and dn_entrypoint.attach.bash
WORKDIR /project_entrypoints
COPY .dockerized_norlab_project/configuration/project_entrypoints .
RUN <<EOF
    for each_file in ./dn_entrypoint.*.bash; do
      chmod +x "${each_file}"
    done

    for each_file in ./project-*/dn_entrypoint.*.bash; do
      chmod +x "${each_file}"
    done
EOF

WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}
CMD [ "bash" ]








