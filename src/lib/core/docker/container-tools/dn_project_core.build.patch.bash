#!/bin/bash
# =================================================================================================
# Add to dna::global_install_hack(), install step that should be executed on all DN-Project
# service variations: project-develop, project-deploy, project-ci-tests, project-slurm
#
# Usage:
#   source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_project_core.build.patch.bash
#
# Globals:
#   Read ROS_DISTRO
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================
set -e

function dna::global_install_hack() {
  # ....Setup....................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)

  # ....Check pre-conditions.......................................................................
  test -n "$( declare -f n2st::print_msg )" || { echo -e "\033[1;31m[DN error]\033[0m The N2ST lib is not loaded!" 1>&2 && exit 1; }

  {
    test -n "${ROS_DISTRO:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_DEV_WORKSPACE:?'Env variable need to be set and non-empty.'}" && \
    test -n "${TARGETPLATFORM:?'Env variable need to be set and non-empty.'}" && \
    test -n "${BUILDPLATFORM:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DEBIAN_FRONTEND:?'Env variable need to be set and non-empty.'}" && \
    [[ "${DEBIAN_FRONTEND}" == "noninteractive" ]];
  } || n2st::print_msg_error_and_exit "Failed pre-condition check!"

  # ....Begin......................................................................................
  n2st::print_msg "Execute global install patch..."

  # ///////////////////////////////////////////////////////////////////////////////////////////////
  # (StandBy) ToDo: maybe transfer to Dockerized-NorLab
  {
    apt-get update && \
    apt-get install --assume-yes --no-install-recommends "ros-${ROS_DISTRO:?err}-rmw-cyclonedds-cpp" ;
  } || n2st::print_msg_error_and_exit "Failed ros-${ROS_DISTRO:?err}-rmw-cyclonedds-cpp install!"

  # || n2st::print_msg_warning "Be advised, encountered ros-${ROS_DISTRO:?err}-rmw-cyclonedds-cpp install problem. Continue anyway."
  echo "Cyclon DDS performance recommendations (ref https://github.com/ros2/rmw_cyclonedds?tab=readme-ov-file)"
  # shellcheck disable=SC2028
  echo "net.core.rmem_max=8388608\nnet.core.rmem_default=8388608\n" | sudo tee /etc/sysctl.d/60-cyclonedds.conf || return 1

  # ///////////////////////////////////////////////////////////////////////////////////////////////
  # (STANDBY) ToDo: add the following Hydra requirements to Dockerized-NorLab
  # (STANDBY) ToDo: assess where to put hydra in DN since its a partial requirement for DNA

  # ....pytest related.............................................................................
  # https://github.com/Teemu/pytest-sugar
  pip3 install 'pytest-sugar'

  # ....Hydra related..............................................................................
  pip3 install 'hydra-core >= 1.3'

  # https://hydra.cc/docs/plugins/ray_launcher/
  pip3 install 'hydra-ray-launcher'

  # Hydra Experiment Sweeper
  # https://github.com/martenlienen/hydra-experiment-sweeper
  pip3 install 'hydra-experiment-sweeper'

  # CPU based parallel job
  # https://hydra.cc/docs/plugins/joblib_launcher/
  pip3 install 'hydra-joblib-launcher'

  # https://hydra.cc/docs/plugins/optuna_sweeper/
  #hydra-optuna-sweeper >= 1.2.0
  pip3 install 'hydra-optuna-sweeper >= 1.3.0.dev0'
  pip3 install 'sqlalchemy<2.0' # Temporary compatibility quickhack for hydra-optuna-sweeper

  # ///////////////////////////////////////////////////////////////////////////////////////////////

  # ....Hyperparameter optimization................................................................
  # (StandBy) ToDo: add to Dockerized-NorLab
  # https://github.com/optuna/optuna-dashboard
  pip3 install 'optuna-dashboard'
  #pip3 install 'bottle == 0.12.*' # Fix the optuna-dashboard loading screen stall problem
  # optional dependencies to make optuna-dashboard faster
  pip3 install 'optuna-fast-fanova'
  pip3 install 'gunicorn'

  # ///////////////////////////////////////////////////////////////////////////////////////////////

  # ....Others.....................................................................................

  # (StandBy) ToDo: add to Dockerized-NorLab
  # Single-command clean up for Python bytecode files in your favorite directories i.e. __pycache__; .*pyc;*.pytest_cache;
  # https://github.com/bittner/pyclean
  pip3 install pyclean

  # ///////////////////////////////////////////////////////////////////////////////////////////////

  # Package is not maintained and pytest-env version 1.6.0 (which is maintained) introduce a flag
  # that conflict with pytest-env (ref task NMO-834)
  pip3 uninstall --yes pytest-dotenv

  # ///////////////////////////////////////////////////////////////////////////////////////////////

  # NMO-789 fix: path pytest-rerunfailure bogus release
  pip3 install pytest-rerunfailures!=16.0

  # ///////////////////////////////////////////////////////////////////////////////////////////////

  # ....Teardown...................................................................................
  apt-get autoremove --assume-yes
  apt-get clean
  rm -rf /var/lib/apt/lists/*

  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}


# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  error_prefix="\033[1;31m[DN error]\033[0m"
  echo -e "${error_prefix} This script must be sourced!
        i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  dna::global_install_hack || n2st::print_msg_error_and_exit "dn_project_core.build.patch.bash exited with error!"
fi
