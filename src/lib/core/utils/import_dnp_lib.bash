#!/bin/bash
# =================================================================================================
# Load Dockerized-NorLab and Dockerized-NorLab-Project resources and dependencies
#
# Usage:
#   $ source import_dnp_lib.bash
#
# Global
#   read/write DNP_ROOT
#
# =================================================================================================
MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"
MSG_DONE_FORMAT="\033[1;32m"

# ....Variable set for export......................................................................

# =================================================================================================
# Function that load DNP lib and all dependencies.
#
# Usage:
#     $ dnp::import_lib_and_dependencies
#
# Arguments:
#   none
# Global:
#   read DNP_ROOT
# Outputs:
#   An error message to to stderr in case of failure
# Returns:
#   1 on faillure, 0 otherwise
# =================================================================================================
function dnp::import_lib_and_dependencies() {

  # ....Setup......................................................................................
  local TMP_CWD
  TMP_CWD=$(pwd)
  local _debug

  # ....cli..........................................................................................
  while [ $# -gt 0 ]; do
    case $1 in
      --debug)
        _debug="true"
        shift # Remove argument (--debug)
        ;;
      *) # Default case
        break
        ;;
    esac
  done

  # ....Find path to script........................................................................
  dnp::find_dnp_root_path || return 1

  # ....Pre-condition..............................................................................
  # Test extracted path
  if [[ ! -d "${DNP_ROOT:?err}" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} dockerized-norlab-project is unreachable at '${DNP_ROOT}'!" 1>&2
    return 1
  fi

  # ....Load DNP .env file for N2ST................................................................
  source "${DNP_ROOT}/load_repo_dotenv.bash"

  # ....Load NBS...................................................................................
  cd "${NBS_PATH:?'Variable not set'}" || return 1
  source "import_norlab_build_system_lib.bash" || return 1

  # ....(Quickhack) Reload project .env file for N2ST..............................................
  source "${DNP_ROOT}/load_repo_dotenv.bash"

  # ....Load N2ST..................................................................................
  cd "${N2ST_PATH:?'Variable not set'}" || return 1
  source "import_norlab_shell_script_tools_lib.bash" || return 1

  # ....(Quickhack) Reload project .env file for N2ST..............................................
  source "${DNP_ROOT}/load_repo_dotenv.bash"

  # ....Teardown...................................................................................
  if [[ "${DNP_DEBUG}" == "true" ]] || [[ "${_debug}" == "true" ]]; then
    export DNP_DEBUG=true
    echo -e "${MSG_DONE_FORMAT}[DNP]${MSG_END_FORMAT} librairies loaded"
  fi
  cd "${TMP_CWD}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}

# =================================================================================================
# Function to find the DNP root path. It seek for the .env.dockerized-norlab-project
# file which should be at the project root by moving up the directory tree from cwd.
#
# Usage:
#     $ dnp::find_dnp_root_path
#
# Arguments:
#   none
# Outputs:
#   An error message to to stderr in case of failure
# Globals:
#   write DNP_ROOT
# Returns:
#   1 on faillure, 0 otherwise
# =================================================================================================
dnp::find_dnp_root_path() {

    # ....Find path to script........................................................................
    # Note: can handle both sourcing cases
    #   i.e. from within a script or from an interactive terminal session
    local SCRIPT_PATH
    SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]:-'.'}")"
    DNP_ROOT="$(dirname "${SCRIPT_PATH}")"

    local max_iterations=10  # Safety limit to prevent infinite loops
    local iterations_count=0

    while [[ "$DNP_ROOT" != "/" && $iterations_count -lt $max_iterations ]]; do

      # Move up to parent directory path
      DNP_ROOT="$( dirname "$DNP_ROOT" )"
      ((iterations_count++))

      # Note: the .env.dockerized-norlab-project check instead of dir dockerized-norlab-project
      #  check is for case where the repo root was clone with a different name, e.g., in teamcity
      if [[ -f "${DNP_ROOT}/.env.dockerized-norlab-project" ]]; then
        echo -e "${MSG_DONE_FORMAT}[DNP]${MSG_END_FORMAT} Found .env.dockerized-norlab-project in: $DNP_ROOT"
        export DNP_ROOT
        return 0
      elif [[ "${DNP_DEBUG}" == "true" ]]; then
        echo "Level ${iterations_count} › DNP_ROOT=$DNP_ROOT"
      fi

    done

    # If we get here, the directory was not found
    echo -e "${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} dockerized-norlab-project root directory not found in any parent directory" >&2
    return 1
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} This script must be sourced i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  dnp::import_lib_and_dependencies "$@" || exit 1
fi
