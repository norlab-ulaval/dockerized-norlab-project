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
declare -x DNP_ROOT

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
  local tmp_cwd
  tmp_cwd=$(pwd)
  local debug_flag

  # ....cli..........................................................................................
  while [ $# -gt 0 ]; do
    case $1 in
      --debug)
        debug_flag="true"
        shift # Remove argument (--debug)
        ;;
      *) # Default case
        break
        ;;
    esac
  done

  # ....Find path to dockerized-norlab-project root................................................
  dnp::find_dnp_root_path || return 1

  # ....Test extracted path........................................................................
  # Note: We validate the repository expected root by matching DNP config and .git config.
  #       Its a more robust alternative to checking the root dir name because the repository
  #       root might be arbitrary different from the repository name for various reason,
  #       e.g., teamcity CI src code pull, user cloned in a different dir, project renamed.

  if [[ ! -d "${DNP_ROOT:?err}" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} dockerized-norlab-project is unreachable at '${DNP_ROOT}'!" 1>&2
    return 1
  fi

  local git_project_path
  git_project_path="$( cd "${DNP_ROOT}" && git rev-parse --show-toplevel )"
  if [[ "${DNP_ROOT:?err}" != "${git_project_path}" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} found project root '${DNP_ROOT}' does not match the repository .git config '${git_project_path}'!" 1>&2
    return 1
  fi

  # ....Load DNP .env file for N2ST................................................................
  source "${DNP_ROOT}/load_repo_main_dotenv.bash"

  # ....Load NBS...................................................................................
  cd "${NBS_PATH:?'Variable not set'}" || return 1
  source "import_norlab_build_system_lib.bash" || return 1

  # ....(Quickhack) Reload project .env file for N2ST..............................................
  source "${DNP_ROOT}/load_repo_main_dotenv.bash"

  # ....Load N2ST..................................................................................
  cd "${N2ST_PATH:?'Variable not set'}" || return 1
  source "import_norlab_shell_script_tools_lib.bash" || return 1

  # ....(Quickhack) Reload project .env file for N2ST..............................................
  source "${DNP_ROOT}/load_repo_main_dotenv.bash"

  # ....Teardown...................................................................................
  if [[ "${DNP_DEBUG}" == "true" ]] || [[ "${debug_flag}" == "true" ]]; then
    export DNP_DEBUG=true
    echo -e "${MSG_DONE_FORMAT}[DNP]${MSG_END_FORMAT} librairies loaded"
  fi
  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}

# =================================================================================================
# Function to find the DNP root path. It seek for the .env.dockerized-norlab-project
# file which should be at the project root by moving up the directory tree from cwd.
#
# Note: Seek the DNP main dotenv instead of the hardcoded repo name to handle the case of
#       teamcity CI src code pull which name the clone root using a different hash number
#       on each build.
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
#   read DNP_DEBUG (optional)
# Returns:
#   1 on faillure, 0 otherwise
# =================================================================================================
function dnp::find_dnp_root_path() {

    # ....Find path to script........................................................................
    # Note: can handle both sourcing cases
    #   i.e. from within a script or from an interactive terminal session
    local script_path
    script_path="$(realpath "${BASH_SOURCE[0]:-'.'}")"
    dnp_root="$(dirname "${script_path}")"

    local max_iterations=10  # Safety limit to prevent infinite loops
    local iterations_count=0

    while [[ "$dnp_root" != "/" && $iterations_count -lt $max_iterations ]]; do
      # Check if DNP main dotenv exists in the current directory
      if [[ -f "${dnp_root}/.env.dockerized-norlab-project" ]]; then
        if [[ "${DNP_DEBUG}" == "true" ]] || [[ "${debug_flag}" == "true" ]]; then
          echo -e "${MSG_DONE_FORMAT}[DNP]${MSG_END_FORMAT} Found .env.dockerized-norlab-project in: $dnp_root"
        fi
        export DNP_ROOT="${dnp_root}"
        return 0
      elif [[ "${DNP_DEBUG}" == "true" ]]; then
        echo "Level ${iterations_count} › dnp_root=$dnp_root"
      fi

      # Move up to parent directory path
      dnp_root="$( dirname "$dnp_root" )"
      ((iterations_count++))
    done

    # If we get here, the repository root was not found
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
