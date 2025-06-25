#!/bin/bash
# =================================================================================================
# Load Dockerized-NorLab and Dockerized-NorLab Project resources and dependencies
#
# Usage:
#   $ source import_dna_lib.bash
#
# Global
#   read/write DNA_ROOT
#
# =================================================================================================
dna_error_prefix="\033[1;31m[DNA error]\033[0m"
dna_done_prefix="\033[1;32m[DNA done]\033[0m"


# ....Variable set for export......................................................................
declare -x DNA_ROOT

# =================================================================================================
# Function that load DNA lib and all dependencies.
#
# Usage:
#     $ dna::import_lib_and_dependencies
#
# Arguments:
#   none
# Global:
#   read DNA_ROOT
# Outputs:
#   An error message to to stderr in case of failure
# Returns:
#   1 on faillure, 0 otherwise
# =================================================================================================
function dna::import_lib_and_dependencies() {

  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)
  local debug_flag=false

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
  dna::find_dna_root_path || return 1

  # ....Test extracted path........................................................................
  # Note: We validate the repository expected root by matching DNA config and .git config.
  #       Its a more robust alternative to checking the root dir name because the repository
  #       root might be arbitrary different from the repository name for various reason,
  #       e.g., teamcity CI src code pull, user cloned in a different dir, project renamed.

  if [[ ! -d "${DNA_ROOT:?err}" ]]; then
    echo -e "\n${dna_error_prefix} dockerized-norlab-project is unreachable at '${DNA_ROOT}'!" 1>&2
    return 1
  fi

  local git_project_path
  git_project_path="$( cd "${DNA_ROOT}" && git rev-parse --show-toplevel )"
  if [[ "${DNA_ROOT:?err}" != "${git_project_path}" ]]; then
    echo -e "\n${dna_error_prefix} found project root '${DNA_ROOT}' does not match the repository .git config '${git_project_path}'!" 1>&2
    return 1
  fi

  # ....Load DNA .env file for N2ST................................................................
  source "${DNA_ROOT}/load_repo_main_dotenv.bash"

  # ....Load NBS...................................................................................
#  cd "${NBS_PATH:?'Variable not set'}" || return 1
  source "${NBS_PATH:?'Variable not set'}/import_norlab_build_system_lib.bash" || return 1

  # (Quickhack) Reload project .env file for N2ST
  source "${DNA_ROOT}/load_repo_main_dotenv.bash"

  # ....Load N2ST..................................................................................
  # Note: load n2st after nbs to make sure that the n2st functions version are not those of
  # the nbs n2st submodule.
#  cd "${N2ST_PATH:?'Variable not set'}" || return 1
  source "${N2ST_PATH:?'Variable not set'}/import_norlab_shell_script_tools_lib.bash" || return 1

  # (Quickhack) Reload project .env file for N2ST
  source "${DNA_ROOT}/load_repo_main_dotenv.bash" || return 1

  # ....Load DNA utils.............................................................................
  source "${DNA_LIB_PATH:?err}/core/utils/execute_compose.bash" || return 1
  source "${DNA_LIB_PATH:?err}/core/utils/ui.bash" || return 1
  source "${DNA_LIB_PATH:?err}/core/utils/online.bash" || return 1

  # ....Export loaded functions....................................................................
  for func in $(compgen -A function | grep -e dna:: -e nbs:: -e n2st::); do
    # shellcheck disable=SC2163
    export -f "${func}"
  done

  # ....Teardown...................................................................................
  if [[ "${DNA_DEBUG}" == "true" ]] || [[ "${debug_flag}" == "true" ]]; then
    export DNA_DEBUG=true
    echo -e "${dna_done_prefix} librairies loaded"
  fi
  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}

# =================================================================================================
# Function to find the DNA root path. It seek for the .env.dockerized-norlab-project
# file which should be at the project root by moving up the directory tree from cwd.
#
# Note: Seek the DNA main dotenv instead of the hardcoded repo name to handle the case of
#       teamcity CI src code pull which name the clone root using a different hash number
#       on each build.
#
# Usage:
#     $ dna::find_dna_root_path
#
# Arguments:
#   none
# Outputs:
#   An error message to to stderr in case of failure
# Globals:
#   write DNA_ROOT
#   read DNA_DEBUG (optional)
# Returns:
#   1 on faillure, 0 otherwise
# =================================================================================================
function dna::find_dna_root_path() {

    # ....Find path to script......................................................................
    # Note: can handle both sourcing cases
    #   i.e. from within a script or from an interactive terminal session
    local script_path
    local dna_root
    # Check if running interactively
    if [[ $- == *i* ]]; then
      # Case: running in an interactive session
      dna_root=$(realpath .)
    else
      # Case: running in an non-interactive session
      script_path="$(realpath -q "${BASH_SOURCE[0]:-.}")"
      dna_root="$(dirname "${script_path}")"
    fi


    # ....Find path to dna root....................................................................
    local max_iterations=10  # Safety limit to prevent infinite loops
    local iterations_count=0

    while [[ "$dna_root" != "/" && $iterations_count -lt $max_iterations ]]; do
      # Check if DNA main dotenv exists in the current directory
      if [[ -f "${dna_root}/.env.dockerized-norlab-project" ]]; then
        if [[ "${DNA_DEBUG}" == "true" ]] || [[ "${debug_flag}" == "true" ]]; then
          echo -e "${dna_done_prefix} Found .env.dockerized-norlab-project in: $dna_root"
        fi
        export DNA_ROOT="${dna_root}"
        return 0
      elif [[ "${DNA_DEBUG}" == "true" ]]; then
        echo "Level ${iterations_count} › dna_root=$dna_root"
      fi

      # Move up to parent directory path
      dna_root="$( dirname "$dna_root" )"
      ((iterations_count++))
    done

    # If we get here, the repository root was not found
    echo -e "${dna_error_prefix} dockerized-norlab-project root directory not found in any parent directory" >&2
    return 1
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "${dna_error_prefix} This script must be sourced i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  dna::import_lib_and_dependencies "$@" || exit 1
fi
