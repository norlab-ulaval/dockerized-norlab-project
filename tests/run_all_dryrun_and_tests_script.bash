#!/bin/bash
# =================================================================================================
# Run all tests in tests_dryrun_and_tests_scripts/ directory
#
# Usage:
#   $ bash run_all_dryrun_and_tests_script.bash
#
# =================================================================================================
_PATH_TO_SCRIPT="$(realpath "$0")"
SCRIPT_DIR_PATH="$(dirname "${_PATH_TO_SCRIPT}")"
TEST_DIR="$SCRIPT_DIR_PATH/tests_dryrun_and_tests_scripts"

SUPER_PROJECT_ROOT=$(git rev-parse --show-toplevel)
source "${SUPER_PROJECT_ROOT}"/.dockerized_norlab_project/utilities/load_dependencies.bash

set -e            # exit on error
set -o nounset    # exit on unbound variable
set -o pipefail   # exit if errors within pipes

source "${NBS_PATH}/src/utility_scripts/nbs_run_all_test_and_dryrun_in_directory.bash" "${TEST_DIR}"

