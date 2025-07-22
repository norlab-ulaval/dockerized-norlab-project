#!/bin/bash
# =================================================================================================
# Dockerized-NorLab project-core architecture aware ros build script.
#
# Note: user have access to `dn_ros2_rebuild_dev_workspace.bash` at runtime
#
# Usage:
#   # In a Dockerfile
#   FROM --platform=${TARGETPLATFORM} project-setup AS project-custom-docker-steps
#   ARG TARGETPLATFORM
#   ARG BUILDPLATFORM
#   WORKDIR /dockerized-norlab/dockerized-norlab-images/container-tools/
#   RUN source dn_project_core.build.aarch_aware_build_ros.bash ${TARGETPLATFORM} ${BUILDPLATFORM} <from-path>
#
# Positional argument:
#   TARGETPLATFORM      The target platform env var generated build buildx
#   BUILDPLATFORM       The host platform env var generated build buildx
#   <from-path>         The path passed to `rosdep install --from-path`
#
# Globals:
#   Read ROS_DISTRO
#   Read DN_DEV_WORKSPACE
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================
set -e
pushd "$(pwd)" >/dev/null || exit 1

TARGETPLATFORM=$1
BUILDPLATFORM=$2
FROM_PATH=$3

# (CRITICAL) ToDo: unit-test

function dna::build_ros() {

  # ....Check pre-conditions.......................................................................
  {
    test -n "${ROS_DISTRO:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_DEV_WORKSPACE:?'Env variable need to be set and non-empty.'}" && \
    test -n "${TARGETPLATFORM:?'Env variable need to be set and non-empty.'}" && \
    test -n "${BUILDPLATFORM:?'Env variable need to be set and non-empty.'}" ;
  } || exit

  # ....Begin......................................................................................
  echo "sourcing /opt/ros/${ROS_DISTRO}/setup.bash"
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  echo "sourcing ${DN_DEV_WORKSPACE}/install/setup.bash"
  source "${DN_DEV_WORKSPACE}/install/setup.bash"

  #apt-get update --ignore-missing
  apt-get update

  rosdep update --rosdistro "${ROS_DISTRO}"  || return 1
  rosdep fix-permissions

  rosdep install \
          --ignore-packages-from-source \
          --from-path "${FROM_PATH}"  \
          --rosdistro "${ROS_DISTRO}"  \
          -q \
          -y \
         || return 1

  colcon version-check

  local colcon_flags=()
  if [[ "${TARGETPLATFORM:?err}" != "${BUILDPLATFORM:?err}" ]]; then
      echo -e "Builder is running in architecture virtualisation"
      colcon_flags+=("--executor" "sequential")
  else
      echo -e "Builder is running on native architecture"
      colcon_flags+=("--symlink-install")
  fi

  colcon_flags+=(
        "--cmake-clean-cache"
        "--cmake-args" "-DCMAKE_BUILD_TYPE=Release"
        "--event-handlers" "console_direct+"
     )
  echo -e "colcon_flags=(${colcon_flags[*]})"
  colcon build "${colcon_flags[@]}" || return 1

  return 0
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  dna_error_prefix="\033[1;31m[DNA error]\033[0m"
  echo -e "${dna_error_prefix} This script must be sourced!
        i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  dna::build_ros || exit 1
fi

# ====Teardown=====================================================================================
popd >/dev/null || exit 1
