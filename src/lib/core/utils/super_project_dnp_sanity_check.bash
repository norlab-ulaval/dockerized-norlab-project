#!/bin/bash
# =================================================================================================
# Check that Dockerized-Norlab-Project components and dependencies are installed at the required
# location in the super-project
#
# Usage:
#   $ [bash|source] super_project_dnp_sanity_check.bash
#
# Global:
#  read DNP_ROOT
#  read SUPER_PROJECT_ROOT
#  read SUPER_PROJECT_REPO_NAME
#
# =================================================================================================

function dnp::super_project_dnp_sanity_check() {
  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)

  if [[ -z "${DNP_ROOT}" ]]; then
    echo -e "\033[1;31m[DNP error]\033[0m DNP libs are not loaded, run import_dnp_lib.bash first!" 1>&2
    exit 1
  fi
  if [[ -z "${SUPER_PROJECT_ROOT}" ]] || [[ -z "${SUPER_PROJECT_REPO_NAME}" ]]; then
    n2st::print_msg_error_and_exit "Super project config are not loaded, run load_super_project_config.bash first!"
  fi

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

  if [[ ! -f ".dockerignore" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} '.dockerignore' is not installed at super-project repository root as it should!" 1>&2
    exit 1
  fi
  if ! grep --silent -E "^\!\*\*\/\.dockerized_norlab_project\/$" ".dockerignore"; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} The line '!**/.dockerized_norlab_project/' is not present in .dockerignore as it should be!" 1>&2
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
    git status >/dev/null || exit 1
  fi

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

  #  ....Teardown...................................................................................
  echo -e "${MSG_DONE_FORMAT}[DNP done]${MSG_END_FORMAT} Super project ${SUPER_PROJECT_REPO_NAME:?err} setup is OK"
  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  # ....Source project shell-scripts dependencies..................................................
  script_path="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  script_path_parent="$(dirname "${script_path}")"
  if [[ -z ${DNP_ROOT}  ]]; then
    source "${script_path_parent}/import_dnp_lib.bash" || exit 1
  fi
  if [[ -z ${SUPER_PROJECT_ROOT}  ]]; then
    source "${script_path_parent}/load_super_project_config.bash" || exit 1
  fi
  dnp::super_project_dnp_sanity_check || exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  :
fi
