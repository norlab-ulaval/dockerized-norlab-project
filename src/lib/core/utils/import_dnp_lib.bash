#!/bin/bash
# =================================================================================================
# Load Dockerized-NorLab and Dockerized-NorLab-Project resources and dependencies
#
# Usage:
#   $ source import_dnp_lib.bash
#
# =================================================================================================
MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"

# ....Variable set for export......................................................................

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
# Returns:
#   1 on faillure, 0 otherwise
# =================================================================================================
function dnp::import_lib_and_dependencies() {
  local TMP_CWD
  local SCRIPT_PATH
  local SCRIPT_PATH_PARENT
  local TMP_DNP_ROOT

  # ....Setup......................................................................................
  # Note: Use local var approach for dir handling in lib import script has its more robust in case
  #       of nested error (instead of the pushd approach).
  TMP_CWD=$(pwd)

  SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  TMP_DNP_ROOT="$(dirname "${SCRIPT_PATH}")"
  while [[ $(basename "${TMP_DNP_ROOT}") != "dockerized-norlab-project" ]]; do
    TMP_DNP_ROOT="$( dirname "$TMP_DNP_ROOT" )"
  done

  # ....Pre-condition..............................................................................
  # Test extracted path
  if [[ ! -d "${TMP_DNP_ROOT:?err}" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} dockerized-norlab-project is unreachable at '${TMP_DNP_ROOT}'!" 1>&2
    return 1
  fi

  # ....Load DNP .env file for N2ST................................................................
  cd "${TMP_DNP_ROOT}" || return 1
  set -o allexport
  # shellcheck disable=SC1090
  source ".env.dockerized-norlab-project" || return 1
  set +o allexport

  # ....Load NBS...................................................................................
  cd "${NBS_PATH:?'Variable not set'}" || return 1
  source "import_norlab_build_system_lib.bash" || return 1

  # ....(Quickhack) Reload project .env file for N2ST..............................................
  cd "${TMP_DNP_ROOT:?err}" || return 1
  set -o allexport
  # shellcheck disable=SC1090
  source ".env.dockerized-norlab-project" || return 1
  set +o allexport

  # ....Load N2ST..................................................................................
  cd "${N2ST_PATH:?'Variable not set'}" || return 1
  source "import_norlab_shell_script_tools_lib.bash" || return 1

  # ....(Quickhack) Reload project .env file for N2ST..............................................
  # shellcheck disable=SC1090
  cd "${TMP_DNP_ROOT:?err}" || return 1
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
