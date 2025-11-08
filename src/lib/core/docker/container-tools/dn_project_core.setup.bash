#!/bin/bash
# =================================================================================================
# Dockerized-NorLab project-core image setup script i.e., user configuration install steps.
# Is executed by '.dockerized_norlab/configuration/Dockerfile.project-core-user' in a DN project image
#
# Usage:
#   source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_project_core.setup.bash
#
# Globals:
#   Read DN_PROJECT_ALIAS_PREFIX
#   Read DN_PROJECT_USER
#   Read DN_PROJECT_USER_HOME
#   Read DN_PROJECT_UID
#   Read DN_PROJECT_GID
#   Read DN_PROJECT_PATH
#   Read DN_DEV_WORKSPACE
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================
set -e

# (CRITICAL) ToDo: unit-test

function dna::execute_shell_requirement_script() {
  # Note: Run shell requirement before python ones so that user have an option to update python or pip

  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)

  # ....Check pre-conditions.......................................................................
  test -n "$( declare -f n2st::print_msg )" || { echo -e "\033[1;31m[DN error]\033[0m The N2ST lib is not loaded!" 1>&2 && exit 1; }

  # ....Begin......................................................................................
  n2st::print_msg "Execute shell.requirements-dna.bash script..."
  source /shell.requirements-dna.bash
  local exit_code=$?

  # ....Teardown...................................................................................
  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }

  apt-get autoremove --assume-yes
  apt-get clean
  rm -rf /var/lib/apt/lists/*

  rm -f /shell.requirements-dna.bash

  if [[ ${exit_code} -eq 0 ]]; then
    n2st::print_msg_done "shell.requirements-dna.bash completed successfully."
    return 0
  else
    n2st::print_msg_error "shell.requirements-dna.bash exited with error!"
    return 1
  fi
}

function dna::install_python_requirement() {
  # Doc: pip install flag: https://pip.pypa.io/en/stable/cli/pip_install/#options
  # Note: Run shell requirement before python ones so that user have an option to update python or pip

  # ....Check pre-conditions.......................................................................
  test -n "$( declare -f n2st::print_msg )" || { echo -e "\033[1;31m[DN error]\033[0m The N2ST lib is not loaded!" 1>&2 && exit 1; }

  # ....Begin......................................................................................
  n2st::print_msg "Execute pip install from python.requirements-dna.txt file..."
  pip3 install --verbose -r /python.requirements-dna.txt
  local exit_code=$?

  # ....Teardown...................................................................................
  rm -f /python.requirements-dna.txt

  if [[ ${exit_code} -eq 0 ]]; then
    n2st::print_msg_done "pip install from python.requirements-dna.txt completed successfully."
    return 0
  else
    n2st::print_msg_error "pip install from python.requirements-dna.txt exited with error!"
    return 1
  fi
}


function dna::setup_dockerized_norlab_project() {

  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)

  # ....Check pre-conditions.......................................................................
  test -n "$( declare -f n2st::print_msg )" || { echo -e "\033[1;31m[DN error]\033[0m The N2ST lib is not loaded!" 1>&2 && exit 1; }

  # Check environment variables
  {
    test -n "${ROS_DISTRO:?'Env variable need to be set and non-empty.'}" && \
    test -n "${TARGETPLATFORM:?'Env variable need to be set and non-empty.'}" && \
    test -n "${BUILDPLATFORM:?'Env variable need to be set and non-empty.'}" ;
    test -n "${DN_PROJECT_ALIAS_PREFIX:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_USER:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_USER_HOME:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_UID:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_GID:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_PATH:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_DEV_WORKSPACE:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_GIT_NAME:?'Env variable need to be set and non-empty.'}" ;
    test -n "${DEBIAN_FRONTEND:?'Env variable need to be set and non-empty.'}" && \
    [[ "${DEBIAN_FRONTEND}" == "noninteractive" ]];
  } || n2st::print_msg_error_and_exit "Failed pre-condition check!"

  # Check directories exist
  {
    test -d "${DN_PROJECT_USER_HOME}" && \
    test -d "${DN_PROJECT_PATH}" ;
  } || n2st::print_msg_error_and_exit "Failed directory check!"

  # ....User specific aliases......................................................................
  n2st::print_msg "Add project specific aliases..."
  (
    echo ""
    echo "# Project specific aliases (general)"
    echo "alias dn-${DN_PROJECT_ALIAS_PREFIX:?err}-cd='cd ${DN_PROJECT_PATH:?err}'"
    echo "alias dn-${DN_PROJECT_ALIAS_PREFIX:?err}-cds='cd ${DN_PROJECT_PATH:?err}/src'"
    echo "alias dn-${DN_PROJECT_ALIAS_PREFIX:?err}-cdt='cd ${DN_PROJECT_PATH:?err}/tests'"
    echo "alias dn-${DN_PROJECT_ALIAS_PREFIX:?err}-cda='cd ${DN_PROJECT_PATH:?err}/artifact'"
    echo "alias dn-${DN_PROJECT_ALIAS_PREFIX:?err}-cdd='cd ${DN_PROJECT_PATH:?err}/data'"
    echo ""
  ) >> /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash

  # ....Add usefull alias utility....................................................................
  (
    echo ""
    echo "# General utility"
    # shellcheck disable=SC2028
    echo "alias dn-show-python-path-split='printenv | grep PYTHONPATH | tr \":\" \"\\n\" | tr \"=\" \"\\n\"'"
    echo ""
  ) >> /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash

  # ....Entrypoint related setup...................................................................
  # Notes:
  #   - All files from 'configuration/entrypoints/' directory that follow the pattern
  #     'dn_entrypoint.*.callback.bash' are required by DN scripts 'dn_entrypoint.init.bash' and
  #     'dn_entrypoint.attach.bash'.
  #   - Be advised that 'project-develop' container mount this directory as a volume to prevent
  #     image rebuilding each time its content is modified (See the 'services.volumes' key in
  #     'docker-compose.run.*.yaml').
  #   - However, 'project-deploy' and 'project-release' containers copy this directory and its
  #     contents in the image at build time to ensure portability.
  #
  n2st::print_msg "Entrypoint related setup..."
  cd /entrypoints || return 1
  {
    test -d project-ci-tests/ && \
    test -d project-ci-tests/test_jobs && \
    test -f project-ci-tests/dn_entrypoint.init.callback.bash && \
    test -d project-slurm/ && \
    test -f project-slurm/dn_entrypoint.init.callback.bash && \
    test -d project-deploy/ && \
    test -f project-deploy/dn_entrypoint.attach.callback.bash && \
    test -f project-deploy/dn_entrypoint.init.callback.bash && \
    test -d project-develop/ && \
    test -f project-develop/dn_entrypoint.attach.callback.bash && \
    test -f project-develop/dn_entrypoint.init.callback.bash && \
    test -f dn_entrypoint.global.attach.callback.bash && \
    test -f dn_entrypoint.global.init.callback.bash ;
  } || n2st::print_msg_error_and_exit "Missing super project configuration file or directory in .dockerized_norlab/configuration/"

  for each_file in ./dn_entrypoint.*.bash; do
    chmod +x "${each_file}"
  done

  for each_file in ./project-*/dn_entrypoint.*.bash; do
    chmod +x "${each_file}"
  done

  chmod +x /dna-lib-container-tools/entrypoints/dn_entrypoint_gpu_checks.bash

  # ....Hack.......................................................................................
  # For Matplotlib default backend (QtAgg) when python script are launched from terminal
  mkdir -m 0700 -p "/tmp/runtime-root" && chown -R "${DN_PROJECT_USER}" "/tmp/runtime-root"

  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}


# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  error_prefix="\033[1;31m[DN error]\033[0m"
  echo -e "${error_prefix} This script must be sourced! i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  {
    dna::setup_dockerized_norlab_project && \
    dna::execute_shell_requirement_script && \
    dna::install_python_requirement;
  } || n2st::print_msg_error_and_exit "dn_project_core.setup.bash exited with error!"
fi

