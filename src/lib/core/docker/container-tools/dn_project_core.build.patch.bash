#!/bin/bash
# =================================================================================================
# Add to dnp::global_install_hack(), install step that should be executed on all DN-Project
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
pushd "$(pwd)" >/dev/null || exit 1

function dnp::global_install_hack() {

  # ///////////////////////////////////////////////////////////////////////////////////////////////
  # (StandBy) ToDo: maybe transfer to Dockerized-NorLab project
#  apt-get update && apt-get update --fix-missing && apt-get upgrade -y
  apt-get install -y \
    ros-"${ROS_DISTRO:?err}"-rmw-cyclonedds-cpp
  apt-get autoremove -y && apt-get clean
  echo "Cyclon DDS performance recommendations (ref https://github.com/ros2/rmw_cyclonedds?tab=readme-ov-file)"
  echo "net.core.rmem_max=8388608\nnet.core.rmem_default=8388608\n" | sudo tee /etc/sysctl.d/60-cyclonedds.conf

  # ///////////////////////////////////////////////////////////////////////////////////////////////
  # (StandBy) ToDo: maybe transfer to Dockerized-NorLab project
  # Temporary hack (might be resolved now)
  # Ref issues
  #  - https://github.com/ipython/ipython/issues/14390
  #  - https://github.com/ros2/launch/issues/765
  pip3 install 'pytest==8.0'

  # ///////////////////////////////////////////////////////////////////////////////////////////////
  # (STANDBY) ToDo: add the following Hydra requirements to Dockerized-NorLab

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
  pip3 install 'bottle == 0.12.*' # Fix the optuna-dashboard loading screen stall problem
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

  # (Priority) ToDo: delete both when NMO-694 is resolve
  n2st::seek_and_modify_string_in_file "alias tree='tree -a -L 1'" "" /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash
  n2st::seek_and_modify_string_in_file "alias tree2='tree -a -L 2'" "" /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash

  # ///////////////////////////////////////////////////////////////////////////////////////////////

  # (Priority) ToDo: delete on task NMO-702 completion >> those lines ↓↓
  local dn_info_path="/dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash"
  n2st::seek_and_modify_string_in_file "docker-compose.project.run.<host-arch>.yaml" ".env.dnp" "$dn_info_path"
  n2st::seek_and_modify_string_in_file "services:" "path: .dockerized_norlab/configuration/.env.dnp" "$dn_info_path"
  n2st::seek_and_modify_string_in_file "  develop: # the service name" "Set environment variable DN_ACTIVATE_POWERLINE_PROMT to false" "$dn_info_path"
  sed -i '/.*environment:/,/- DN_ACTIVATE_POWERLINE_PROMT=false/d' "$dn_info_path"
  n2st::seek_and_modify_string_in_file "dn_attach" "dnp [up|exec]" "$dn_info_path"
  n2st::seek_and_modify_string_in_file "<the-running-container-name>" "bash" "$dn_info_path"

  # ///////////////////////////////////////////////////////////////////////////////////////////////

  return 0
}


# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  MSG_ERROR_FORMAT="\033[1;31m"
  MSG_END_FORMAT="\033[0m"
  echo -e "${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} This script must be sourced!
        i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  dnp::global_install_hack || exit 1
fi

# ====Teardown=====================================================================================
popd >/dev/null || exit 1
