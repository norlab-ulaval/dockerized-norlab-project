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

  # ....Find path to script........................................................................
  # Note: can handle both sourcing cases
  #   i.e. from within a script or from an interactive terminal session
  local SCRIPT_PATH
  local SCRIPT_PATH_PARENT
  SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  SCRIPT_PATH_PARENT="$(dirname "${SCRIPT_PATH}")"

  # ....Setup......................................................................................
  source "${SCRIPT_PATH_PARENT}/import_dnp_lib.bash" || exit 1
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

  if [[ ! -d "src/" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} The 'src' directory is not installed at super-project repository root as it should!" 1>&2
    exit 1
  fi

  if [[ $( git -C "${SUPER_PROJECT_ROOT}" rev-parse --is-inside-work-tree ) != true ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} The '.git' directory does not exist at super-project repository root as it should!
Dockerized-NorLab-Porject require that the super project be under version control using git." 1>&2
    exit 1
  else
    git status
  fi

#  echo "$( git rev-parse --show-toplevel )"
#  echo "$( git remote get-url origin )"
#  echo -e "\n${0}: breakpoint\n" && exit 1 # (CRITICAL) ToDo: on task end >> delete this line <--

  if [[ ! -d "tests/" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} The 'tests' directory is not installed at super-project repository root as it should!" 1>&2
    exit 1
  fi

  if [[ ! -d "external_data/" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} The 'external_data' directory is not installed at super-project repository root as it should!" 1>&2
    exit 1
  fi

  if [[ ! -d "artifact/" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} The 'artifact' directory is not installed at super-project repository root as it should!" 1>&2
    exit 1
  fi

  if [[ ! -d "artifact/optuna_storage/" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} The 'optuna_storage' directory is not installed in the super-project 'artifact/' directory as it should!" 1>&2
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
  test -f ".env.dnp" || exit 1
  test -f ".env" || exit 1
  test -f ".env.local" || exit 1
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
