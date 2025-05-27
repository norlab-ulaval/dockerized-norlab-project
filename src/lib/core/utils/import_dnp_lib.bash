#!/bin/bash
# =================================================================================================
# Load Dockerized-NorLab and Dockerized-NorLab-Project resources and dependencies
#
# Usage:
#   $ source import_dnp_lib.bash
#
# Globals:
#   write DNP_REPO_ROOT_PATH
#
# =================================================================================================
MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"

# ....Variable set for export......................................................................
declare -x DNP_REPO_ROOT_PATH

# =================================================================================================
# Function that load DNP lib and load all dependencies.
#
# Usage:
#     $ dnp::import_lib_and_dependencies
#
# Arguments:
#   none
# Outputs:
#   An error message to to stderr in case of failure
# Globals:
#   write DNP_REPO_ROOT_PATH
# Returns:
#   1 on faillure, 0 otherwise
# =================================================================================================
function dnp::import_lib_and_dependencies() {

  # ....Setup......................................................................................
  # Note: Use local var approach for dir handling in lib import script has its more robust in case
  #       of nested error (instead of the pushd approach).
  local TMP_CWD
  TMP_CWD=$(pwd)

  export DNP_REPO_ROOT_PATH=$(git rev-parse --show-toplevel)

  # ....Pre-condition..............................................................................
  # Test extracted path
  if [[ ! -d "${DNP_REPO_ROOT_PATH:?err}" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} dockerized-norlab-project is unreachable at '${DNP_REPO_ROOT_PATH}'!" 1>&2
    return 1
  fi

  # ....Load DNP .env file for N2ST................................................................
  cd "${DNP_REPO_ROOT_PATH}" || return 1
  set -o allexport
  # shellcheck disable=SC1090
  source ".env.dockerized-norlab-project" || return 1
  set +o allexport

  # ....Load NBS...................................................................................
  cd "${NBS_PATH:?'Variable not set'}" || return 1
  source "import_norlab_build_system_lib.bash" || return 1

  # ....(Quickhack) Reload project .env file for N2ST..............................................
  cd "${DNP_REPO_ROOT_PATH:?err}" || return 1
  set -o allexport
  # shellcheck disable=SC1090
  source ".env.dockerized-norlab-project" || return 1
  set +o allexport

  # ....Load N2ST..................................................................................
  cd "${N2ST_PATH:?'Variable not set'}" || return 1
  source "import_norlab_shell_script_tools_lib.bash" || return 1

  # ....(Quickhack) Reload project .env file for N2ST..............................................
  # shellcheck disable=SC1090
  cd "${DNP_REPO_ROOT_PATH:?err}" || return 1
  set -o allexport
  source ".env.dockerized-norlab-project" || return 1
  set +o allexport

  #  ....Teardown...................................................................................
  echo -e "${MSG_DONE_FORMAT}[DNP]${MSG_END_FORMAT} librairies loaded"
  cd "${TMP_CWD}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} This script must be sourced i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  dnp::import_lib_and_dependencies || exit 1
fi

## For DEBUG
#{
#  echo -e "\n=======================================================\n" &&
#    printenv | grep -i -e DN_ -e N2ST_ -e NBS_ -e RLRC &&
#    echo -e "\n=======================================================\n"
#} && echo "WE ARE HERE" #<--
