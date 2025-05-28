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

  # ....Setup......................................................................................
  local SCRIPT_PATH
  local SCRIPT_PATH_PARENT

  # Note: can handle both sourcing cases
  #   i.e. from within a script or from an interactive terminal session
  SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  SCRIPT_PATH_PARENT="$(dirname "${SCRIPT_PATH}")"


  source "${SCRIPT_PATH_PARENT}/load_super_project_config.bash" || exit 1

  # ====Begin======================================================================================
  cd "${SUPER_PROJECT_ROOT:?err}" || exit 1

  # ....check super project directory structure....................................................
  if [[ ! -d ".dockerized_norlab_project" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} '.dockerized_norlab_project' is not installed at super-project repository root as it should!" 1>&2
    exit 1
  fi

  if [[ ! -f ".dockerignore" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} '.dockerignore' is not installed at super-project repository root as it should!" 1>&2
    exit 1
  fi

  # ....check .dockerized_norlab_project directory structure.......................................
  cd ".dockerized_norlab_project" || exit 1
  test -d "configuration/" || exit 1
  test -d "dn_container_env_variable/" || exit 1
  test -d "slurm_jobs/" || exit 1

  cd "configuration/" || exit 1

  test -d "project_requirements/" || exit 1
  test -d "project_entrypoints/" || exit 1
  test -f ".env" || exit 1
  test -f "Dockerfile" || exit 1

  test -d "project_entrypoints/project-ci-tests/" || exit 1
  test -d "project_entrypoints/project-ci-tests/test_jobs" || exit 1
  test -d "project_entrypoints/project-deploy/" || exit 1
  test -d "project_entrypoints/project-develop/" || exit 1

  echo -e "${MSG_DONE_FORMAT}[DNP done]${MSG_END_FORMAT} Super project ${SUPER_PROJECT_REPO_NAME:?err} setup is OK"
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
pushd "$(pwd)" >/dev/null || exit 1

dnp::validate_super_project_dnp_setup

# ====Teardown=====================================================================================
popd >/dev/null || exit 1
