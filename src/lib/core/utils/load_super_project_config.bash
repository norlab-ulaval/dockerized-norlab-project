#!/bin/bash
# =================================================================================================
# Load Dockerized-NorLab and Dockerized-NorLab-Project resources and dependencies
#
# Usage:
#   $ source import_dnp_lib.bash
#
# Globals:
#   read DNP_ROOT
#   write SUPER_PROJECT_ROOT, SUPER_PROJECT_REPO_NAME
#
# =================================================================================================
MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"

# ....Variable set for export......................................................................
declare -x SUPER_PROJECT_ROOT
declare -x SUPER_PROJECT_REPO_NAME

# =================================================================================================
# Function that load DNP lib, find the super project root and load all dependencies.
#
# Usage:
#     $ dnp::load_super_project_configurations
#
# Arguments:
#   none
# Outputs:
#   An error message to to stderr in case of failure
# Globals:
#   read/write SUPER_PROJECT_ROOT
#   write SUPER_PROJECT_REPO_NAME
# Returns:
#   1 on faillure, 0 otherwise
# =================================================================================================
function dnp::load_super_project_configurations() {

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


  # ....Find super project path and name...........................................................
  dnp::find_dnp_super_project_dir || return 1

  SUPER_PROJECT_REPO_NAME=$(basename "${SUPER_PROJECT_ROOT}")

  export SUPER_PROJECT_ROOT
  export SUPER_PROJECT_REPO_NAME

  # ....Test extracted path........................................................................
  local SUPER_PROJECT_META_DNP_DOTENV=".env.${SUPER_PROJECT_REPO_NAME}"
  if [[ ! -f "${SUPER_PROJECT_ROOT}/.dockerized_norlab_project/${SUPER_PROJECT_META_DNP_DOTENV:?err}" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} can't find '.dockerized_norlab_project/${SUPER_PROJECT_META_DNP_DOTENV}' in ${SUPER_PROJECT_ROOT}!" 1>&2
    return 1
  fi

  # ....Load Dockerized-NorLab-Project .env file...................................................
  cd "${SUPER_PROJECT_ROOT:?err}" || return 1
  set -o allexport
  source ".dockerized_norlab_project/${SUPER_PROJECT_META_DNP_DOTENV}" || return 1
  set +o allexport

  # ....Load build time DNP dotenv file for docker-compose.........................................
  set -o allexport
  cd "${SUPER_PROJECT_ROOT:?err}" || return 1
  source ".dockerized_norlab_project/configuration/.env.dnp" || return 1
  source "${DNP_ROOT:?err}/src/lib/core/docker/.env.dnp-internal" || return 1
  set +o allexport

  # ....Load run time DNP dotenv file for docker-compose...........................................
  cd "${SUPER_PROJECT_ROOT:?err}" || return 1
  set -o allexport
  source ".dockerized_norlab_project/configuration/.env" || return 1
  source ".dockerized_norlab_project/configuration/.env.local" || return 1
  set +o allexport


  #  ....Teardown...................................................................................
  if [[ "${DNP_DEBUG}" == "true" ]] || [[ "${_debug}" == "true" ]]; then
    export DNP_DEBUG=true
    echo -e "${MSG_DONE_FORMAT}[DNP]${MSG_END_FORMAT} ${SUPER_PROJECT_REPO_NAME} project configurations loaded"
  fi

  cd "${TMP_CWD}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}


# =================================================================================================
# Function to find the DNP user side project path. It seek for the .dockerized_norlab_project
# directory which should be at the project root by moving up the directory tree from cwd.
#
# Usage:
#     $ dnp::find_dnp_super_project_dir
#
# Arguments:
#   none
# Outputs:
#   An error message to to stderr in case of failure
# Globals:
#   write SUPER_PROJECT_ROOT
# Returns:
#   1 on faillure, 0 otherwise
# =================================================================================================
dnp::find_dnp_super_project_dir() {

    local current_dir=$(pwd)
    local max_iterations=10  # Safety limit to prevent infinite loops
    local iterations_count=0

    while [[ "$current_dir" != "/" && $iterations_count -lt $max_iterations ]]; do
        # Check if .dockerized_norlab_project exists in the current directory
        if [[ -d "$current_dir/.dockerized_norlab_project" ]]; then
            echo "Found .dockerized_norlab_project in: $current_dir"
            export SUPER_PROJECT_ROOT="$( pwd )"
            return 0
        fi

        # Move up to parent directory
        cd ..
        current_dir=$(pwd)
        ((iterations_count++))
    done

    # If we get here, the directory was not found
    echo -e "${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} .dockerized_norlab_project directory not found in any parent directory" >&2
    return 1
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} This script must be sourced i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  dnp::load_super_project_configurations "$@" || { echo -e "${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} failled to load DNP user project configurations" 1>&2 && exit 1; }
fi

