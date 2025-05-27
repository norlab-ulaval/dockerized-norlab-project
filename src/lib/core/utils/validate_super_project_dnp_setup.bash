#!/bin/bash
# =================================================================================================
# Check that Dockerized-Norlab-Project components and dependencies are installed at the required
# location in the super-project
#
# Usage:
#   $ bash validate_super_project_dnp_setup.bash
#
# =================================================================================================

function dnp::validate_super_project_dnp_setup() {

  # Note: can handle both sourcing cases
  #   i.e. from within a script or from an interactive terminal session
  local _PATH_TO_SCRIPT="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  local _CWD="$(dirname "${_PATH_TO_SCRIPT}")"

  # ....Source project shell-scripts dependencies..................................................
  source "${_CWD}/load_super_project_config.bash" || exit 1
  cd "${SUPER_PROJECT_ROOT:?err}" || exit 1

  # ====Begin======================================================================================
  if [[ ! -d ".dockerized_norlab_project" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} '.dockerized_norlab_project' is not installed at super-project repository root as it should!" 1>&2
    exit 1
  fi

  if [[ ! -f ".dockerignore" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} '.dockerignore' is not installed at super-project repository root as it should!" 1>&2
    exit 1
  fi

  # (Priority) ToDo: add other DNP user side required dir/files

  echo -e "${MSG_DONE_FORMAT}[DNP done]${MSG_END_FORMAT} Super project ${SUPER_PROJECT_REPO_NAME:?err} setup is OK"
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
pushd "$(pwd)" >/dev/null || exit 1

dnp::validate_super_project_dnp_setup

# ====Teardown=====================================================================================
popd >/dev/null || exit 1
