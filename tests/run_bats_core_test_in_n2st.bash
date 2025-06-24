#!/bin/bash
# =================================================================================================
# Execute 'dockerized-norlab-project' repository .bats shell script tests using 'norlab-shell-script-tools'
# library. Under the hood, it will copy or optionaly mount the repository source code in a docker
# container and execute every .bats tests located in the '<test-directory>'. The script can be
# executed from anywhere as long as its inside the 'dockerized-norlab-project' repository.
#
# Usage:
#  $ bash run_bats_core_test_in_n2st.bash [--mount-src-code-as-a-volume] [--help]
#                                         ['<test-directory>[/<bats-test-file-name.bats>]' ['<image-distro>']]
#
# Arguments:
#   --mount-src-code-as-a-volume      Mount the source code at run time instead of copying it at build time.
#                                     Comromise in isolation to the benefit of increase velocity.
#                                     Usefull for project dealing with large files but require
#                                     handling temporary files and directory manualy via bats-file.
#   -h | --help                       Show the N2ST script run_bats_tests_in_docker.bash help message
#
# Positional argument:
#   '<test-directory>'                The directory from which to start test (default to 'tests')
#   '<bats-test-file-name.bats>'      A specific bats file to run, default will run all bats file
#                                      in the test directory
#   '<image-distro>'                  ubuntu or alpine (default ubuntu)
#
# Globals:
#   read DNA_ROOT
#   read N2ST_PATH    Default to "./utilities/norlab-shell-script-tools"
#
# =================================================================================================
params=( "$@" )

if [[ -z ${params[0]} ]]; then
  # Set to default bats tests directory if none specified
  params=("tests/tests_bats/")
fi

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
bash "${DNA_ROOT:?err}/tests/setup_mock.bash"

function dna::bats_tests_teardown_callback() {
  exit_code=$?
  cd "${DNA_ROOT:?err}" || exit 1
  bash tests/teardown_mock.bash
  exit ${exit_code:1}
}
trap dna::bats_tests_teardown_callback EXIT

# ....Execute N2ST run_bats_tests_in_docker.bash.................................................
cd "${DNA_ROOT:?err}" || exit 1

bash "${N2ST_PATH:?err}/tests/bats_testing_tools/run_bats_tests_in_docker.bash" "${params[@]}"

# ....Teardown.....................................................................................
# Handle by the trap command

