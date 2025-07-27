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
# (CRITICAL) ToDo: unit-test

TARGETPLATFORM=$1
BUILDPLATFORM=$2
FROM_PATH=$3

function dna::build_ros() {
  n2st::print_msg "Execute architecture aware ROS2 build..."

  # ....Check pre-conditions.......................................................................
  {
    test -n "${ROS_DISTRO:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_DEV_WORKSPACE:?'Env variable need to be set and non-empty.'}" && \
    test -n "${TARGETPLATFORM:?'Env variable need to be set and non-empty.'}" && \
    test -n "${BUILDPLATFORM:?'Env variable need to be set and non-empty.'}" ;
  } || n2st::print_msg_error_and_return "Failed pre-condition check!"

  if [[ -d "${DN_DEV_WORKSPACE}" ]]; then
    cd "${DN_DEV_WORKSPACE}"
  else
    n2st::print_msg_error_and_return "Directory ${DN_DEV_WORKSPACE} is unrechable!"
  fi

  # ....Begin......................................................................................
  echo "sourcing /opt/ros/${ROS_DISTRO}/setup.bash"
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  echo "sourcing ${DN_DEV_WORKSPACE}/install/setup.bash"
  source "${DN_DEV_WORKSPACE}/install/setup.bash"

  # (CRITICAL) ToDo: Validate >> next line ↓↓
  apt-get update \
    && apt-get upgrade --assume-yes

  rosdep update --rosdistro "${ROS_DISTRO}" || n2st::print_msg_error_and_return "Failed rosdep update!"
  rosdep fix-permissions

  rosdep install \
          --ignore-packages-from-source \
          --from-path "${FROM_PATH}"  \
          --rosdistro "${ROS_DISTRO}"  \
          -q \
          -y \
       || n2st::print_msg_error_and_return "Failed rosdep install!"

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

  colcon build "${colcon_flags[@]}" || n2st::print_msg_error_and_return "Failed colcon build!"

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
  test -n "$( declare -f n2st::print_msg )" || { echo -e "\033[1;31m[N2ST error]\033[0m The N2ST lib is not loaded!" 1>&2 && exit 1; }
  dna::build_ros || n2st::print_msg_error_and_exit "dn_project_core.build.aarch_aware_build_ros.bash exited with error!"
fi

# ====Teardown=====================================================================================
popd >/dev/null || exit 1
