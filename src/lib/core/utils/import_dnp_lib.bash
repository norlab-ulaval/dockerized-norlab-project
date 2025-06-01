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
# Function that load DNP lib and all dependencies.
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

  # ....Setup......................................................................................
  local TMP_CWD
  TMP_CWD=$(pwd)

  # ....Find path to script........................................................................
  # Note: can handle both sourcing cases
  #   i.e. from within a script or from an interactive terminal session
  local SCRIPT_PATH
  local TARGET_ROOT
  SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  TARGET_ROOT="$(dirname "${SCRIPT_PATH}")"

  # ....Find path to target parent directory.......................................................
  while [[ $(basename "${TARGET_ROOT}") != "dockerized-norlab-project" ]]; do
    TARGET_ROOT="$( dirname "$TARGET_ROOT" )"
  done

  # ....Pre-condition..............................................................................
  # Test extracted path
  if [[ ! -d "${TARGET_ROOT:?err}" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} dockerized-norlab-project is unreachable at '${TARGET_ROOT}'!" 1>&2
    return 1
  fi

  # ....Load DNP .env file for N2ST................................................................
  source "${TARGET_ROOT}/load_repo_dotenv.bash"

  # ....Load NBS...................................................................................
  cd "${NBS_PATH:?'Variable not set'}" || return 1
  source "import_norlab_build_system_lib.bash" || return 1

  # ....(Quickhack) Reload project .env file for N2ST..............................................
  source "${TARGET_ROOT}/load_repo_dotenv.bash"

  # ....Load N2ST..................................................................................
  cd "${N2ST_PATH:?'Variable not set'}" || return 1
  source "import_norlab_shell_script_tools_lib.bash" || return 1

  # ....(Quickhack) Reload project .env file for N2ST..............................................
  source "${TARGET_ROOT}/load_repo_dotenv.bash"

  # ....Teardown...................................................................................
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
