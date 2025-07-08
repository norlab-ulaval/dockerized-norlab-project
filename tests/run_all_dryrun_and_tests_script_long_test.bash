#!/bin/bash
# =================================================================================================
# Run all tests in tests_dryrun_and_tests_scripts/ and tests_dryrun_and_tests_scripts_long_tests/
#  directories
#
# Usage:
#   $ bash run_all_dryrun_and_tests_script_long_test.bash
#
# =================================================================================================
test_dir="tests/tests_dryrun_and_tests_scripts"
test_dir2="tests/tests_dryrun_and_tests_scripts_long_tests"

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1

# ....Begin........................................................................................
source "${NBS_PATH:?err}/src/utility_scripts/nbs_run_all_test_and_dryrun_in_directory.bash" "${DNA_ROOT:?err}/${test_dir}"
EXIT_CODE1=$?

source "${NBS_PATH:?err}/src/utility_scripts/nbs_run_all_test_and_dryrun_in_directory.bash" "${DNA_ROOT:?err}/${test_dir2}"
EXIT_CODE_2=$?

exit $(( EXIT_CODE1 + EXIT_CODE_2 ))
