#!/bin/bash
# =================================================================================================
# Load Dockerized-NorLab and Dockerized-NorLab-Project resources and dependencies
#
# Usage:
#   $ source load_dependencies.bash
#
# =================================================================================================

# Variable set for export
declare -x SUPER_PROJECT_ROOT

function dnp::source_project_shellscript_dependencies() {

  # ....N2ST lib configuration.......................................................................
  # Note: PROJECT_ENV_N2ST_FILE is required for enabling N2ST functionality
  SUPER_PROJECT_ROOT=$(git rev-parse --show-toplevel)
  PROJECT_REPO_NAME=$(basename "${SUPER_PROJECT_ROOT}" .git)
  PROJECT_ENV_N2ST_FILE=".env.${PROJECT_REPO_NAME}"

  # ....Setup......................................................................................
  # Note: Use local var approach for dir handling in lib import script has its more robust in case
  #       of nested error (instead of the pushd approach).
  local TMP_CWD
  TMP_CWD=$(pwd)

  # ....Pre-condition..............................................................................
  export SUPER_PROJECT_ROOT

  if [[ ! -f "${SUPER_PROJECT_ROOT}/${PROJECT_ENV_N2ST_FILE:?err}" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} '$PROJECT_ENV_N2ST_FILE' root directory unreachable!\n Current working directory is '$(pwd)'" 1>&2
    exit 1
  fi

  # ....Load project .env file for N2ST............................................................
  cd "${SUPER_PROJECT_ROOT}" || exit 1
  set -o allexport
  # shellcheck disable=SC1090
  source "${PROJECT_ENV_N2ST_FILE}" || exit 1
  set +o allexport


  # ....Load NBS...................................................................................
  cd "${NBS_PATH:?'Variable not set'}" || exit 1
  source "import_norlab_build_system_lib.bash" || exit 1

  # ....(Quickhack) Reload project .env file for N2ST..............................................
  cd "${SUPER_PROJECT_ROOT:?err}" || exit 1
  set -o allexport
  # shellcheck disable=SC1090
  source "${PROJECT_ENV_N2ST_FILE}" || exit 1
  set +o allexport

  # ....Load N2ST..................................................................................
  cd "${N2ST_PATH:?'Variable not set'}" || exit 1
  source "import_norlab_shell_script_tools_lib.bash" || exit 1

  # ....(Quickhack) Reload project .env file for N2ST..............................................
  # shellcheck disable=SC1090
  cd "${SUPER_PROJECT_ROOT:?err}" || exit 1
  set -o allexport
  source "${PROJECT_ENV_N2ST_FILE}" || exit 1
  set +o allexport

  # ....Load Dockerized-NorLab-Project .env file...................................................
  cd "${SUPER_PROJECT_ROOT:?err}" || exit 1
  set -o allexport
  source ".dockerized_norlab_project/configuration/.env" || exit 1
  set +o allexport

  #  ....Teardown...................................................................................
  cd "${TMP_CWD}" || { echo "Return to original dir error" 1>&2 && exit 1; }
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"

if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} This script must be sourced i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  dnp::source_project_shellscript_dependencies
fi

## For DEBUG
#{
#  echo -e "\n=======================================================\n" &&
#    printenv | grep -i -e DN_ -e N2ST_ -e NBS_ -e RLRC &&
#    echo -e "\n=======================================================\n"
#} && echo "WE ARE HERE" #<--
