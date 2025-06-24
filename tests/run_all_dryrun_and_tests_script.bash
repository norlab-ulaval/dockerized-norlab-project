#!/bin/bash
# =================================================================================================
# Run all tests in tests_dryrun_and_tests_scripts/ directory
#
# Usage:
#   $ bash run_all_dryrun_and_tests_script.bash
#
# =================================================================================================
test_dir="tests/tests_dryrun_and_tests_scripts"

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1

# ....Begin........................................................................................
source "${NBS_PATH:?err}/src/utility_scripts/nbs_run_all_test_and_dryrun_in_directory.bash" "${DNP_ROOT:?err}/${test_dir}"
exit $?
