#!/bin/bash
# =================================================================================================
# Run all tests in tests_dryrun_and_tests_scripts/ directory
#
# Usage:
#   $ bash run_all_dryrun_and_tests_script.bash
#
# =================================================================================================
SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]:-'.'}")"
SCRIPT_PATH_PARENT="$(dirname "${SCRIPT_PATH}")"
TEST_DIR="${SCRIPT_PATH_PARENT}/tests_dryrun_and_tests_scripts"

source "$(git rev-parse --show-toplevel)/src/lib/core/utils/import_dnp_lib.bash" || exit 1

set -e            # exit on error
set -o nounset    # exit on unbound variable
set -o pipefail   # exit if errors within pipes

source "${NBS_PATH:?err}/src/utility_scripts/nbs_run_all_test_and_dryrun_in_directory.bash" "${TEST_DIR}"

