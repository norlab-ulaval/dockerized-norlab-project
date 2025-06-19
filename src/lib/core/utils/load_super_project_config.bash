#!/bin/bash
# =================================================================================================
# Load Dockerized-NorLab and Dockerized-NorLab-Project resources and dependencies
#
# Usage:
#   $ source import_dnp_lib.bash
#
# Globals:
#   read DNP_ROOT
#   write SUPER_PROJECT_ROOT
#   write SUPER_PROJECT_REPO_NAME
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


  # ....Find super project path and name...........................................................
  dnp::cd_to_dnp_super_project_root || exit 1
  # Note: fail fast on super project not found

  if [[ ! -d "${SUPER_PROJECT_ROOT:?err}/.git" ]]; then
      n2st::print_msg_error_and_exit "Can't find git directory at project root. DNP project are required to be under git version control."
  fi

  local super_project_git_remote_url
  super_project_git_remote_url=$( cd "${SUPER_PROJECT_ROOT:?err}" && git remote get-url origin )
  SUPER_PROJECT_REPO_NAME="$( basename "${super_project_git_remote_url}" .git )"
  export SUPER_PROJECT_REPO_NAME

  local super_project_meta_dnp_dotenv=".env.${SUPER_PROJECT_REPO_NAME}"

  # ....Test extracted path........................................................................
  # Note: We validate the repository expected root by matching DNP config and .git config.
  #       Its a more robust alternative to checking the root dir name because the repository
  #       root might be arbitrary different from the repository name for various reason,
  #       e.g., teamcity CI src code pull, user cloned in a different dir, project renamed.

  if [[ ! -f "${SUPER_PROJECT_ROOT:?err}/.dockerized_norlab_project/${super_project_meta_dnp_dotenv:?err}" ]]; then
    n2st::print_msg_error "can't find '.dockerized_norlab_project/.env.<SUPER_PROJECT_REPO_NAME>' in ${SUPER_PROJECT_ROOT}!" 1>&2
    return 1
  fi

  local git_project_path
  git_project_path="$( cd "${SUPER_PROJECT_ROOT}" && git rev-parse --show-toplevel )"
  if [[ "${SUPER_PROJECT_ROOT:?err}" != "${git_project_path}" ]]; then
    n2st::print_msg_error "found project root '${SUPER_PROJECT_ROOT}' does not match the repository .git config '${git_project_path}'!" 1>&2
    return 1
  fi

  # ....Load super project DNP meta config dotenv file.............................................
  cd "${SUPER_PROJECT_ROOT:?err}" || return 1
  set -o allexport
  source ".dockerized_norlab_project/${super_project_meta_dnp_dotenv}" || return 1
  set +o allexport


  # ....Load build time DNP dotenv file for docker-compose.........................................
  set -o allexport
  cd "${SUPER_PROJECT_ROOT:?err}" || return 1
  source ".dockerized_norlab_project/configuration/.env.dnp" || return 1
  source "${DNP_ROOT:?err}/src/lib/core/docker/.env.dnp-internal" || return 1
  set +o allexport

  if [[ "${super_project_git_remote_url}" != "${DN_PROJECT_GIT_REMOTE_URL:?err}" ]]; then
    n2st::print_msg_error "super project ${SUPER_PROJECT_REPO_NAME} DNP configuration in .dockerized_norlab_project/configuration.env.dnp DN_PROJECT_GIT_REMOTE_URL=${DN_PROJECT_GIT_REMOTE_URL} does not match the repository .git config url '${super_project_git_remote_url}'!" 1>&2
    return 1
  fi


  # ....Load run time DNP dotenv file for docker-compose...........................................
  cd "${SUPER_PROJECT_ROOT:?err}" || return 1
  set -o allexport
  source ".dockerized_norlab_project/configuration/.env" || return 1
  source ".dockerized_norlab_project/configuration/.env.local" || return 1
  set +o allexport


  #  ....Teardown...................................................................................
  if [[ "${DNP_DEBUG}" == "true" ]] || [[ "${debug_flag}" == "true" ]]; then
    export DNP_DEBUG=true
    n2st::print_msg_done "${SUPER_PROJECT_REPO_NAME} project configurations loaded"
  fi

  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}


# =================================================================================================
# Function to find the DNP user side project path. It seek for the .dockerized_norlab_project
# directory which should be at the project root by moving up the directory tree from cwd.
#
# Usage:
#     $ dnp::cd_to_dnp_super_project_root
#
# Arguments:
#   none
# Outputs:
#   An error message to to stderr in case of failure
# Globals:
#   write SUPER_PROJECT_ROOT
#   read DNP_DEBUG (optional)
# Returns:
#   1 on faillure, 0 otherwise
# =================================================================================================
function dnp::cd_to_dnp_super_project_root() {

    local current_working_dir
    local initial_working_dir
    declare -a current_working_dir_trace=()
    initial_working_dir=$(pwd)
    current_working_dir="$initial_working_dir"


    local max_iterations=10  # Safety limit to prevent infinite loops
    local iterations_count=0

    while [[ "$current_working_dir" != "/" && $iterations_count -lt $max_iterations ]]; do
        # Check if .dockerized_norlab_project exists in the current directory
        if [[ -d "$current_working_dir/.dockerized_norlab_project" ]]; then
            n2st::print_msg "Found .dockerized_norlab_project in: $current_working_dir"
            export SUPER_PROJECT_ROOT="${current_working_dir}"
            return 0
        elif [[ "${DNP_DEBUG}" == "true" ]]; then
          n2st::print_msg "Level ${iterations_count} › current_working_dir=$current_working_dir"
        fi

        # Move up to parent directory
        cd ..
        current_working_dir=$(pwd)
        current_working_dir_trace+=("$current_working_dir")
        ((iterations_count++))
    done

    # If we get here, the directory was not found
    n2st::print_msg_error "${MSG_ERROR_FORMAT}${MSG_DIMMED_FORMAT}.dockerized_norlab_project${MSG_END_FORMAT}${MSG_ERROR_FORMAT} directory not found in any parent directory.\n\n$(
echo -e "        cwd -> $initial_working_dir"
for each in "${current_working_dir_trace[@]}" ; do
  echo -e "               $each"
done
    )\n
Check if the project in which your curently executing a DNP command as been initialize with ${MSG_DIMMED_FORMAT}dnp init${MSG_END_FORMAT}${MSG_ERROR_FORMAT}.
Execute ${MSG_DIMMED_FORMAT}dnp project sanity${MSG_END_FORMAT}${MSG_ERROR_FORMAT} if your unsure.
${MSG_END_FORMAT}" >&2
    return 1
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} This script must be sourced i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # Check if N2ST is loaded
  n2st::print_msg "test" 2>/dev/null >/dev/null || { echo -e "${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} The N2ST lib is not loaded!" ; exit 1 ; }

  dnp::load_super_project_configurations "$@" || { n2st::print_msg_error "failled to load DNP user project configurations" 1>&2 && exit 1; }
fi

