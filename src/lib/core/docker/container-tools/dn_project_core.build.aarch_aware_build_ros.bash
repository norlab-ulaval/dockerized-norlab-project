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
# (CRITICAL) ToDo: unit-test

TARGETPLATFORM=$1
BUILDPLATFORM=$2
FROM_PATH=$3
error_prefix="\033[1;31m[DN error]\033[0m"

function dna::build_ros() {

  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)


  # ....Check pre-conditions.......................................................................
  test -n "$( declare -f n2st::print_msg )" || { echo -e "${error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
  test -n "$( declare -f dn::source_ros2_underlay_only )" || { echo -e "${error_prefix} The DN lib is not loaded!" 1>&2 && exit 1; }

  {
    test -n "${ROS_DISTRO:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_DEV_WORKSPACE:?'Env variable need to be set and non-empty.'}" && \
    test -n "${TARGETPLATFORM:?'Env variable need to be set and non-empty.'}" && \
    test -n "${BUILDPLATFORM:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DEBIAN_FRONTEND:?'Env variable need to be set and non-empty.'}" && \
    [[ "${DEBIAN_FRONTEND}" == "noninteractive" ]];
  } || n2st::print_msg_error_and_exit "Failed pre-condition check!"

  if [[ -d "${DN_DEV_WORKSPACE}" ]]; then
    cd "${DN_DEV_WORKSPACE}"
  else
    n2st::print_msg_error_and_exit "Directory ${DN_DEV_WORKSPACE} is unrechable!"
  fi

  # ....Begin......................................................................................
  n2st::print_msg "Execute architecture aware ROS2 build..."
  echo "  TARGETPLATFORM: ${TARGETPLATFORM}"
  echo "  BUILDPLATFORM: ${BUILDPLATFORM}"
  echo "  FROM_PATH: ${FROM_PATH}"
  echo "  DEBIAN_FRONTEND: ${DEBIAN_FRONTEND}"
  echo

  apt-get update

  # ....Install Python development libraries for ROS2 build..........................................
  n2st::set_which_python3_version
  n2st::print_msg "Install python${PYTHON3_VERSION:?err} dev libs for ROS2 build..."
  apt-get install --assume-yes --no-install-recommends \
        python3-dev \
        libpython3-dev \
        "python${PYTHON3_VERSION}-dev" \
        "python${PYTHON3_VERSION}-doc" \
      || n2st::print_msg_error_and_exit "Failed to install python dev libs for ROS2 build!"

  n2st::print_msg "Execute rosdep update..."
  dn::source_ros2_underlay_only || exit 1
  rosdep update --rosdistro "${ROS_DISTRO}" || n2st::print_msg_error_and_exit "Failed rosdep update!"
  rosdep fix-permissions

  n2st::print_msg "Execute rosdep install..."
  rosdep install \
          --ignore-packages-from-source \
          --from-path "${FROM_PATH}"  \
          --rosdistro "${ROS_DISTRO}"  \
          -q \
          -y \
       || n2st::print_msg_error_and_exit "Failed rosdep install!"

  colcon version-check

  local python_version
  python_version=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
  for lib_path in "/usr/lib/x86_64-linux-gnu" "/usr/lib/aarch64-linux-gnu" "/usr/lib" "/usr/local/lib"; do
      if [[ -f "${lib_path}/libpython${python_version}.so" ]]; then
          PYTHON_LIBRARIES="${lib_path}/libpython${python_version}.so"
          break
      fi
  done
  PYTHON_INCLUDE_DIRS="$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())")"
  echo -e "Python env var passed to colcon build cmake args:\n
    Python libraries: ${PYTHON_LIBRARIES}
    Python include dirs: ${PYTHON_INCLUDE_DIRS}
    cmake version: $(cmake --version)\n"

  test -f "${PYTHON_LIBRARIES}" || n2st::print_msg_error_and_exit "${PYTHON_LIBRARIES} unreachable!"
  test -d "${PYTHON_INCLUDE_DIRS}" || n2st::print_msg_error_and_exit "${PYTHON_INCLUDE_DIRS} unreachable!"

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
        "--cmake-args"
            "-DCMAKE_BUILD_TYPE=Release"
            "-DPYTHON_LIBRARIES=${PYTHON_LIBRARIES}"
            "-DPYTHON_INCLUDE_DIRS=${PYTHON_INCLUDE_DIRS}"
        "--event-handlers" "console_direct+"
     )
  echo -e "colcon_flags=(${colcon_flags[*]})"

  n2st::print_msg "Execute colcon build..."
  colcon build "${colcon_flags[@]}" || n2st::print_msg_error_and_exit "Failed colcon build!"

  # ....Teardown...................................................................................
  n2st::print_msg "Configuration issue introspection..."
  dpkg --audit
  echo
  dpkg-architecture -l
  echo
  tree -aL 2 /etc/dpkg/dpkg.cfg.d
  echo
  cat /etc/dpkg/dpkg.cfg
  echo

  apt-get autoremove --assume-yes
  apt-get clean
  rm -rf /var/lib/apt/lists/*

  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "${error_prefix} This script must be sourced!
        i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  dna::build_ros || n2st::print_msg_error_and_exit "dn_project_core.build.aarch_aware_build_ros.bash exited with error!"
fi
