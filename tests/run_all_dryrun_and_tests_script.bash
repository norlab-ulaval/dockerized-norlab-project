#!/bin/bash
# =================================================================================================
# Run all tests in tests_dryrun_and_tests_scripts/ directory
#
# Usage:
#   $ bash run_all_dryrun_and_tests_script.bash
#
# =================================================================================================

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1

# ....Begin........................................................................................
(
  set -e            # exit on error
  set -o nounset    # exit on unbound variable
  set -o pipefail   # exit if errors within pipes

  test_dir="${DNP_ROOT:?err}/tests/tests_dryrun_and_tests_scripts"
  source "${NBS_PATH:?err}/src/utility_scripts/nbs_run_all_test_and_dryrun_in_directory.bash" "${test_dir}"
)
