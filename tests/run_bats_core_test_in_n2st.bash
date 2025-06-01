#!/bin/bash
# =================================================================================================
# Execute 'dockerized-norlab-project' repo shell script tests via 'norlab-shell-script-tools' library
#
# Usage:
#   $ bash run_bats_core_test_in_n2st.bash ['<test-directory>[/<this-bats-test-file.bats>]' ['<image-distro>']]
#
# Arguments:
#   - ['<test-directory>']     The directory from which to start test, default to 'tests'
#   - ['<test-directory>/<this-bats-test-file.bats>']  A specific bats file to run, default will
#                                                      run all bats file in the test directory
#
# Globals: none
#
# =================================================================================================
PARAMS="$@"

if [[ -z $PARAMS ]]; then
  # Set to default bats tests directory if none specified
  PARAMS="tests/tests_bats/"
fi

function n2st::run_n2st_testsing_tools(){
  local TMP_CWD
  TMP_CWD=$(pwd)

  # ....Load environment variables from file.......................................................
  source "$(git rev-parse --show-toplevel)/load_repo_dotenv.bash"

  # ....Load N2ST..................................................................................
  cd "${N2ST_PATH:?'Variable not set'}" || return 1
  source "import_norlab_shell_script_tools_lib.bash" || return 1

  # ....Setup dockerized-norlab-project-mock.......................................................
  bash "${DNP_ROOT:?err}/tests/setup_mock.bash"

  # ....Execute N2ST run_bats_tests_in_docker.bash.................................................
  cd "${DNP_ROOT:?err}" || return 1

  # shellcheck disable=SC2086
  bash "${N2ST_PATH:?err}/tests/bats_testing_tools/run_bats_tests_in_docker.bash" $PARAMS

  # ....Teardown dockerized-norlab-project-mock....................................................
  bash "${DNP_ROOT:?err}/tests/teardown_mock.bash"

  # ....Teardown...................................................................................
  cd "$TMP_CWD" || return 1
  return 0
  }

n2st::run_n2st_testsing_tools || exit 1

