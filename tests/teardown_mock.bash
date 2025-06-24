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
# Global:
#   read DNA_ROOT
#   read N2ST_PATH
#
# =================================================================================================
pushd "$(pwd)" >/dev/null || exit 1

function dna::teardown_mock() {
  n2st::print_formated_script_header "teardown_mock.bash" "${MSG_LINE_CHAR_UTIL}"

  test -n "${DNA_ROOT:?err}" || n2st::print_msg_error_and_exit "Env variable DNA_ROOT need to be set and non-empty."
  test -d "${DNA_ROOT}/utilities/tmp/dockerized-norlab-project-mock" \
    || n2st::print_msg_error_and_exit "The directory ${DNA_ROOT}/utilities/tmp/dockerized-norlab-project-mock is unreachable"

  # Delete git cloned repo
  rm -rf "${DNA_ROOT}/utilities/tmp/dockerized-norlab-project-mock"

  # Setup placeholder
  mkdir "${DNA_ROOT}/utilities/tmp/dockerized-norlab-project-mock"

  # ....Sanity check...............................................................................
  test -d "${DNA_ROOT}/utilities" || n2st::print_msg_error_and_exit "The directory ${DNA_ROOT}/utilities is unreachable"
  test ! -d "${DNA_ROOT}/utilities/tmp/dockerized-norlab-project-mock/.git" \
  || { \
    tree -a -L 2 "${DNA_ROOT}/utilities/tmp" &&
    n2st::print_msg_error_and_exit "Something went wrong with the deletion of the cloned repository ${DNA_ROOT}/utilities/tmp/dockerized-norlab-project-mock/" ;
    }

  n2st::print_formated_script_footer "teardown_mock.bash" "${MSG_LINE_CHAR_UTIL}"
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  cd "${N2ST_PATH:?'Variable not set'}" || exit 1
  source "import_norlab_shell_script_tools_lib.bash" || exit 1

  dna::teardown_mock

else
  # This script is being sourced, ie: __name__="__source__"
  dna_error_prefix="\033[1;31m[DNA error]\033[0m"
  echo -e "${dna_error_prefix} This script must executed with bash! i.e.: $ bash $( basename "$0" )" 1>&2
  exit 1
fi

# ====Teardown=====================================================================================
popd >/dev/null || exit 1


