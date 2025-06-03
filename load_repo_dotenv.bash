#!/bin/bash
# =================================================================================================
# Load all environment variable define in .env.dockerized-norlab-project
#
# Usage:
#   $ source load_repo_dotenv.bash
#
# =================================================================================================
MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"
MSG_DONE_FORMAT="\033[1;32m"

function dnp::load_repository_environment_variables() {
  # ....Setup......................................................................................
  local TMP_CWD
  TMP_CWD=$( pwd )
  local debug_flag="false"

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
  # Note: can handle both sourcing cases
  #   i.e. from within a script or from an interactive terminal session
  local SCRIPT_PATH
  local TARGET_PATH
  SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  TARGET_PATH="$(dirname "${SCRIPT_PATH}")"
  cd "${TARGET_PATH:?err}" || return 1

  # ....load environment variables in current shell................................................
  set -o allexport
  source ".env.dockerized-norlab-project" || return 1
  set +o allexport

  # ....Teardown...................................................................................
  if [[ "${DNP_DEBUG}" == "true" ]] || [[ "${debug_flag}" == "true" ]]; then
    export DNP_DEBUG=true
    echo -e "${MSG_DONE_FORMAT}[DNP]${MSG_END_FORMAT} .env.dockerized-norlab-project loaded"
    # Debug flags
    set -v # echo lines as they are read
    export BUILDKIT_PROGRESS=plain
  fi

  cd "${TMP_CWD}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} This script must be sourced i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  dnp::load_repository_environment_variables "$@" || exit 1
fi
