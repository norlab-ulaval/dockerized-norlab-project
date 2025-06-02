#!/bin/bash
# ===============================================================================================
# Teardown dockerized-norlab-project-mock for testing
#
# Usage:
#   $ bash teardown_mock.bash
#
# Notes:
#   See utilities/tmp/README.md for details on role of `dockerized-norlab-project-mock`
#
# =================================================================================================

# ....Load N2ST..................................................................................
cd "${N2ST_PATH:?'Variable not set'}" || exit 1
source "import_norlab_shell_script_tools_lib.bash" || exit 1

function dnp::teardown_mock() {
  n2st::print_formated_script_header "teardown_mock.bash" "${MSG_LINE_CHAR_UTIL}"

  test -n "${DNP_ROOT:?err}" || n2st::print_msg_error_and_exit "Env variable DNP_ROOT need to be set and non-empty."
  test -d "${DNP_ROOT}/utilities/tmp/dockerized-norlab-project-mock" \
    || n2st::print_msg_error_and_exit "The directory ${DNP_ROOT}/utilities/tmp/dockerized-norlab-project-mock is unreachable"

  # Delete git cloned repo
  rm -rf "${DNP_ROOT}/utilities/tmp/dockerized-norlab-project-mock"

  # Setup placeholder
  mkdir "${DNP_ROOT}/utilities/tmp/dockerized-norlab-project-mock"

  # ....Sanity check...............................................................................
  test -d "${DNP_ROOT}/utilities" || n2st::print_msg_error_and_exit "The directory ${DNP_ROOT}/utilities is unreachable"
  test ! -d "${DNP_ROOT}/utilities/tmp/dockerized-norlab-project-mock/.git" \
  || { \
    tree -a -L 2 "${DNP_ROOT}/utilities" &&
    n2st::print_msg_error_and_exit "The directory ${DNP_ROOT}/utilities is unreachable" ;
    }

  n2st::print_formated_script_footer "teardown_mock.bash" "${MSG_LINE_CHAR_UTIL}"
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  dnp::teardown_mock
else
  # This script is being sourced, ie: __name__="__source__"
  MSG_ERROR_FORMAT="\033[1;31m"
  MSG_END_FORMAT="\033[0m"
  echo -e "${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} This script must executed with bash! i.e.: $ bash $( basename "$0" )" 1>&2
  exit 1
fi


