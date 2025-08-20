#!/bin/bash
# =================================================================================================
# Load all environment variable define in .env.dockerized-norlab-project
#
# Usage:
#   $ source load_repo_main_dotenv.bash
#
# =================================================================================================
dna_error_prefix="\033[1;31m[dna error]\033[0m"
dna_done_prefix="\033[1;32m[dna done]\033[0m"

function dna::load_repository_environment_variables() {

  # ....Setup......................................................................................
  local debug_flag=false
  local tmp_cwd
  tmp_cwd=$(pwd)
  local script_path
  local target_path

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


  # ....Find path to script........................................................................
  if [[ -z ${DNA_ROOT} ]]; then
    # Note: can handle both sourcing cases
    #   i.e. from within a script or from an interactive terminal session
    # Check if running interactively
    if [[ $- == *i* ]]; then
      # Case: running in an interactive session
      target_path=$(realpath .)
    else
      # Case: running in an non-interactive session
      script_path="$(realpath -q "${BASH_SOURCE[0]:-.}")"
      target_path="$(dirname "${script_path}")"
    fi

    if [[ ${debug_flag} == true ]]; then
      echo "
      >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
      BASH_SOURCE: ${BASH_SOURCE[*]}

      tmp_cwd: ${tmp_cwd}
      script_path: ${script_path}
      target_path: ${target_path}

      realpath: $(realpath .)
      \$0: $0
      <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
      "
    fi
  else
    target_path="${DNA_ROOT}"
  fi

  # ....Check pre-condition........................................................................
  if ! command -v git >/dev/null 2>&1; then
    n2st::print_msg_error "'git' is not installed.\nExiting now" && return 1
  fi

  cd "${target_path:?err}" || return 1
  if [[ ! -d .git ]]; then
    n2st::print_msg_error "dockerized-norlab-project is missing it's .git directory. Check cloned/copied directory at ${target_path}!\nExiting now" && return 1
  fi

  if [[ ! -f .env.dockerized-norlab-project ]]; then
    echo -e "\n[\033[1;31mDN error\033[0m] Can't find ${DNA_HUMAN_NAME} repository root!" 1>&2
    return 1
  fi

  # ....load environment variables in current shell................................................
  set -o allexport
  source ".env.dockerized-norlab-project" || return 1
  set +o allexport

  local repo_dotenv_local="${DNA_LIB_PATH:?err}/.env.dockerized-norlab-project.local"
  if [[ -f "${repo_dotenv_local}" ]]; then
    # Note: Dotenv file '.env.dockerized-norlab-project.local', if it exist, should be sourced
    #       after '.env.dockerized-norlab-project'.
    if [[ $(grep -c '^[[:space:]]*[A-Z_][A-Z0-9_]*=' "${repo_dotenv_local}" 2>/dev/null) -gt 0 ]]; then
        n2st::print_msg_warning "Be advised, sourcing dna internal ${MSG_EMPH_FORMAT}local${MSG_END_FORMAT} dotenv file ${MSG_DIMMED_FORMAT}${repo_dotenv_local}${MSG_END_FORMAT}."
    fi
    set -o allexport
    # shellcheck disable=SC1090
    source "${repo_dotenv_local}" || return 1
    set +o allexport
  fi

  # ....Teardown...................................................................................
  if [[ "${DNA_DEBUG}" == "true" ]] || [[ "${debug_flag}" == "true" ]]; then
    export DNA_DEBUG=true
    echo -e "${dna_done_prefix} .env.dockerized-norlab-project loaded"
    # Debug flags
    set -v # echo lines as they are read
    export BUILDKIT_PROGRESS=plain
  fi

  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "${dna_error_prefix} This script must be sourced i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  dna::load_repository_environment_variables "$@" || exit 1
fi
