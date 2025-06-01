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
# Globals:
#   Read DNP_ROOT
#
# =================================================================================================
function dnp::teardown_mock() {
  set -e

  n2st::print_formated_script_header "teardown_mock.bash" "${MSG_LINE_CHAR_UTIL}"

  test -n "${DNP_ROOT:?err}" || n2st::print_msg_error_and_exit "Env variable DNP_ROOT need to be set and non-empty."
  test -d "${DNP_ROOT}/utilities/tmp/dockerized-norlab-project-mock" \
    || n2st::print_msg_error_and_exit "The directory ${DNP_ROOT}/utilities/tmp/dockerized-norlab-project-mock is unreachable"

  # Delete git cloned repo
  rm -rf "${DNP_ROOT}/utilities/tmp/dockerized-norlab-project-mock"

  # Setup placeholder
  mkdir "${DNP_ROOT}/utilities/tmp/dockerized-norlab-project-mock"

  # ....Saniti check...............................................................................
  test -d "${DNP_ROOT}/utilities" || n2st::print_msg_error_and_exit "The directory ${DNP_ROOT}/utilities is unreachable"
  test ! -d "${DNP_ROOT}/utilities/tmp/dockerized-norlab-project-mock/.git" \
  || { \
    tree -a -L 2 "${DNP_ROOT}/utilities" &&
    n2st::print_msg_error_and_exit "The directory ${DNP_ROOT}/utilities is unreachable" ;
    }

  n2st::print_formated_script_footer "teardown_mock.bash" "${MSG_LINE_CHAR_UTIL}"
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dnp::teardown_mock
