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

set -e            # exit on error
set -o nounset    # exit on unbound variable
set -o pipefail   # exit if errors within pipes

if [[ -z $PARAMS ]]; then
  # Set to default bats tests directory if none specified
  PARAMS="tests/tests_bats/"
fi

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_dotenv.bash" || exit 1
bash "${DNP_ROOT:?err}/tests/setup_mock.bash"

function dnp::teardown() {
  EXIT_CODE=$?
  cd "${DNP_ROOT:?err}"
  bash tests/teardown_mock.bash
  exit ${EXIT_CODE:1}
}
trap dnp::teardown EXIT

# ....Execute N2ST run_bats_tests_in_docker.bash.................................................
cd "${DNP_ROOT:?err}" || return 1

bash "${N2ST_PATH:?err}/tests/bats_testing_tools/run_bats_tests_in_docker.bash" $PARAMS

# ....Teardown.....................................................................................
# Handle by the trap command
