#!/bin/bash
# ===============================================================================================
# Setup dockerized-norlab-project-mock for testing
#
# Usage:
#   $ bash setup_mock.bash
#
# Notes:
#   See utilities/tmp/README.md for details on role of `dockerized-norlab-project-mock`
#
# Global:
#   read DNP_ROOT
#   read N2ST_PATH
#
# =================================================================================================
pushd "$(pwd)" >/dev/null || exit 1

function dnp::setup_mock() {
  n2st::print_formated_script_header "setup_mock.bash" "${MSG_LINE_CHAR_UTIL}"

  test -n "${DNP_ROOT:?err}" || n2st::print_msg_error_and_exit "Env variable DNP_ROOT need to be set and non-empty."
  test -d "${DNP_ROOT}/utilities/tmp" || n2st::print_msg_error_and_exit "The directory ${DNP_ROOT}/utilities/tmp is unreachable"

  if [[ -d "${DNP_ROOT}/utilities/tmp/dockerized-norlab-project-mock" ]]; then
      # Delete git cloned repo
      rm -rf "${DNP_ROOT}/utilities/tmp/dockerized-norlab-project-mock"
  fi
  if [[ ! -d "${DNP_ROOT}/utilities/tmp/dockerized-norlab-project-mock" ]]; then
    mkdir "${DNP_ROOT}/utilities/tmp/dockerized-norlab-project-mock"
  fi
  git clone https://github.com/norlab-ulaval/dockerized-norlab-project-mock.git \
    "${DNP_ROOT}/utilities/tmp/dockerized-norlab-project-mock" \
    || n2st::print_msg_error_and_exit "Could not clone dockerized-norlab-project-mock"

  # ....Sanity check...............................................................................
  test -d "${DNP_ROOT}/utilities/tmp" || n2st::print_msg_error_and_exit "The directory ${DNP_ROOT}/utilities/tmp is unreachable"
  test -d "${DNP_ROOT}/utilities/tmp/dockerized-norlab-project-mock/.git" \
  || { \
    tree -a -L 2 "${DNP_ROOT}/utilities/tmp" &&
    n2st::print_msg_error_and_exit "The directory ${DNP_ROOT}/utilities/tmp is unreachable" ;
    }

  n2st::print_formated_script_footer "setup_mock.bash" "${MSG_LINE_CHAR_UTIL}"
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  cd "${N2ST_PATH:?'Variable not set'}" || exit 1
  source "import_norlab_shell_script_tools_lib.bash" || exit 1

  dnp::setup_mock
else
  # This script is being sourced, ie: __name__="__source__"
  MSG_ERROR_FORMAT="\033[1;31m"
  MSG_END_FORMAT="\033[0m"
  echo -e "${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} This script must executed with bash! i.e.: $ bash $( basename "$0" )" 1>&2
  exit 1
fi

# ====Teardown=====================================================================================
popd >/dev/null || exit 1
