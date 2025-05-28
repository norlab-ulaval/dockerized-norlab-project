
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS project-setup
ARG DEBIAN_FRONTEND=noninteractive

# ====Dockerized-NorLab-project internal configuration step========================================
ARG DN_PROJECT_GIT_NAME
ENV DN_PROJECT_GIT_NAME=${DN_PROJECT_GIT_NAME:?'Build argument needs to be set and non-empty.'}
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
ARG IS_TEAMCITY_RUN
ENV IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN:-false}

# ....Set PATH related environment variable........................................................
ENV DN_PROJECT_PATH=${DN_DEV_WORKSPACE:?'Build argument needs to be set and non-empty.'}/src/${DN_PROJECT_GIT_NAME}
ENV PATH="${DN_PROJECT_PATH}:${PATH}"
ENV PYTHONPATH="${DN_PROJECT_PATH}/src:${DN_PROJECT_PATH}/tests:${DN_PROJECT_PATH}/external_data:${PYTHONPATH}"

# ....Copy init and setup files....................................................................
COPY --from=context-dn-container-tools ./dn_project_core_init.bash /dn_project_core_init.bash
COPY --from=context-dnp-container-tools ./dn_project_core_setup.bash /dn_project_core_setup.bash
COPY --from=context-dnp-container-tools ./dn_architecture_aware_build_ros.bash /dn_architecture_aware_build_ros.bash
COPY .dockerized_norlab_project/configuration/project_requirements/python.requirements.txt /python.requirements.txt
COPY .dockerized_norlab_project/configuration/project_requirements/shell.requirements.bash /shell.requirements.bash
COPY .dockerized_norlab_project/configuration/project_entrypoints /project_entrypoints

RUN <<EOF
  source /dn_project_core_init.bash || exit 1
  source /dn_project_core_setup.bash || exit 1
  rm -f /dn_project_core_setup.bash && rm -f /dn_project_core_init.bash
EOF

# For Matplotlib default backend (QtAgg) when python script are launched from terminal
ENV XDG_RUNTIME_DIR='/tmp/runtime-root'

# ====User project custom steps====================================================================
FROM project-setup AS project-custom-docker-steps

RUN <<EOF
  # Sanity check example
  {
    echo "..........................................." && \
    python -c "import torch" && \
    python -c "import torchvision" && \
    echo "..........................................." && \
    python -c "import hydra" && \
    python -c "from omegaconf import DictConfig, OmegaConf" ;
  } || exit 1

  # ROS custom install step example
  apt-get update && apt-get update --fix-missing && apt-get upgrade -y
  apt-get install -y \
      ros-${ROS_DISTRO:?err}-rmw-cyclonedds-cpp
  apt-get autoremove -y && apt-get clean
  echo "Cyclon DDS performance recommendations (ref https://github.com/ros2/rmw_cyclonedds?tab=readme-ov-file)"
  echo "net.core.rmem_max=8388608\nnet.core.rmem_default=8388608\n" | sudo tee /etc/sysctl.d/60-cyclonedds.conf
EOF


# //// To refactor ////////////////////////////////////////////////////////////////////////////////
# (Priority) ToDo: refactor >> transfer to Dockerized-NorLab project
# Temporary hack (might be resolved now)
# ref issues
#  - https://github.com/ipython/ipython/issues/14390
#  - https://github.com/ros2/launch/issues/765
RUN pip3 install 'pytest==8.0'
# \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ To refactor \\\\


# ====DN-project final=============================================================================
FROM --platform=${TARGETPLATFORM} project-custom-docker-steps AS final
ARG TARGETPLATFORM
ARG BUILDPLATFORM
WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}

RUN <<EOF
  source /dn_architecture_aware_build_ros.bash ${TARGETPLATFORM} ${BUILDPLATFORM} ${DN_DEV_WORKSPACE:?err}/src || exit 1
  rm -f /dn_architecture_aware_build_ros.bash
EOF

CMD [ "bash" ]








