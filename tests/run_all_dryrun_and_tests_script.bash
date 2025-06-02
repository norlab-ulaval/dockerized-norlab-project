#!/bin/bash
# =================================================================================================
# Run all tests in tests_dryrun_and_tests_scripts/ directory
#
# Usage:
#   $ bash run_all_dryrun_and_tests_script.bash
#
# =================================================================================================

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

# ....Begin........................................................................................
(
  set -e            # exit on error
  set -o nounset    # exit on unbound variable
  set -o pipefail   # exit if errors within pipes

  TEST_DIR="${DNP_ROOT:?err}/tests/tests_dryrun_and_tests_scripts"
  source "${NBS_PATH:?err}/src/utility_scripts/nbs_run_all_test_and_dryrun_in_directory.bash" "${TEST_DIR}"
)


# ....Teardown.....................................................................................
# Handle by the trap command
