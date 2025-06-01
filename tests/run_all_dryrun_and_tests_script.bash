#!/bin/bash
# =================================================================================================
# Run all tests in tests_dryrun_and_tests_scripts/ directory
#
# Usage:
#   $ bash run_all_dryrun_and_tests_script.bash
#
# =================================================================================================
set -e            # exit on error
set -o nounset    # exit on unbound variable
set -o pipefail   # exit if errors within pipes

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_dotenv.bash"
bash "${DNP_ROOT:?err}/tests/setup_mock.bash"

function dnp::execute_on_exit() {
  cd "${DNP_ROOT:?err}"
  bash tests/teardown_mock.bash
  exit ${EXIT_CODE:1}
}
trap dnp::execute_on_exit EXIT

# ....Begin........................................................................................
TEST_DIR="${DNP_ROOT:?err}/tests/tests_dryrun_and_tests_scripts"
source "${NBS_PATH:?err}/src/utility_scripts/nbs_run_all_test_and_dryrun_in_directory.bash" "${TEST_DIR}"
EXIT_CODE=$?

# ....Teardown.....................................................................................
# Handle by the trap command
