#!/usr/bin/env bats
# =================================================================================================
# Usage in docker container
#   $ REPO_ROOT=$(pwd) && RUN_TESTS_IN_DIR='tests'
#   $ docker run -it --rm -v "$REPO_ROOT:/code" bats/bats:latest "$RUN_TESTS_IN_DIR"
#
#   Note: "/code" is the working directory in the bats official image
#
# bats-core ref:
#   - https://bats-core.readthedocs.io/en/stable/tutorial.html
#   - https://bats-core.readthedocs.io/en/stable/writing-tests.html
#   - https://opensource.com/article/19/2/testing-bash-bats
#       ↳ https://github.com/dmlond/how_to_bats/blob/master/test/build.bats
#
# Helper library:
#   - https://github.com/bats-core/bats-assert
#   - https://github.com/bats-core/bats-support
#   - https://github.com/bats-core/bats-file
#
# =================================================================================================

bats_path=/usr/lib/bats
error_prefix="[\033[1;31mN2ST ERROR\033[0m]"
if [[ -d ${bats_path} ]]; then
  # ....Bats-core recommended helper functions.....................................................
  load "${bats_path}/bats-support/load"
  load "${bats_path}/bats-assert/load"
  load "${bats_path}/bats-file/load"
  # ....Optional...................................................................................
  #load "${bats_path}/bats-detik/load" # <- Kubernetes support
  # ....N2ST library helper function...............................................................
  load "${SRC_CODE_PATH:?err}/${N2ST_BATS_TESTING_TOOLS_RELATIVE_PATH:?err}/bats_helper_functions"
  load "${SRC_CODE_PATH}/tests/tests_bats/bats_testing_tools/bats_helper_functions_local"
else
  echo -e "\n${error_prefix} $0 path to bats-core helper library unreachable at \"${bats_path}\"!"
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi

# ====Tests file configuration=====================================================================

TESTED_FILE="version.bash"
TESTED_FILE_PATH="src/lib/commands"

# ....Setup........................................................................................
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
  export MOCK_PROJECT_PATH="${BATS_DOCKER_WORKDIR}/utilities/tmp/dockerized-norlab-project-mock"

  # Create temporary directory for tests
  export MOCK_DNP_DIR=$(temp_make)

  # Create mock functions directory in the temporary directory
  mkdir -p "${MOCK_DNP_DIR}/src/lib/core/utils/"

  # Create a mock import_dnp_lib.bash that sets up the environment
  cat > "${MOCK_DNP_DIR}/src/lib/core/utils/import_dnp_lib.bash" << 'EOF'
#!/bin/bash
# Mock import_dnp_lib.bash

# ....Setup........................................................................................

# Set message formatting variables
export MSG_DIMMED_FORMAT=""
export MSG_END_FORMAT=""

# Set up environment variables
export DNP_SPLASH_NAME_FULL="Dockerized-NorLab (DN)"
export DNP_SPLASH_NAME_SMALL="Dockerized-NorLab"
export DNP_ROOT="${MOCK_DNP_DIR}"
export DNP_LIB_PATH="${MOCK_DNP_DIR}/src/lib"

# ....Mock dependencies loading test functions.....................................................
function dnp::import_lib_and_dependencies() {
  return 0
}

function n2st::print_msg() {
  return 0
}

# ....Mock ui.bash functions.......................................................................
function dnp::command_help_menu() {
  echo "Mock dnp::command_help_menu called with args: $*"
  return 0
}

# ....Export mock functions........................................................................
for func in $(compgen -A function | grep -e dnp:: -e n2st::); do
  export -f "$func"
done

# ....Teardown.....................................................................................
# Print a message to indicate that the mock import_dnp_lib.bash has been loaded
echo "[DNP done] Mock import_dnp_lib.bash and its librairies loaded"
EOF
}

setup() {
  # Create necessary directories in the temporary directory
  mkdir -p "${MOCK_DNP_DIR}/src/lib/commands"
  mkdir -p "${MOCK_DNP_DIR}/src/lib/core/utils"

  # Copy the version.bash file to the temporary directory
  cp "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" "${MOCK_DNP_DIR}/src/lib/commands/"

  source "${MOCK_DNP_DIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1

  # Change to the temporary directory
  cd "${MOCK_DNP_DIR}" || exit 1
}

# ....Teardown.....................................................................................
teardown() {
  bats_print_run_env_variable_on_error
}

teardown_file() {
  # Clean up temporary directory
  temp_del "${MOCK_DNP_DIR}"
}

# ====Test cases==================================================================================

@test "dnp::version_command with no arguments and version.txt exists › expect version display" {
  # Create a mock version.txt file
  mkdir -p "${MOCK_DNP_DIR}"
  echo "1.0.0" > "${MOCK_DNP_DIR}/version.txt"

  # Test case: When version command is called without arguments and version.txt exists, it should display the version
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/version.bash && dnp::version_command"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Dockerized-NorLab-Project version: 1.0.0"
}

@test "dnp::version_command with --help › expect help menu" {
  # Test case: When version command is called with --help, it should show the help menu
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/version.bash && dnp::version_command --help"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dnp::command_help_menu called with args:"
}

@test "dnp::version_command with -h › expect help menu" {
  # Test case: When version command is called with -h, it should show the help menu
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/version.bash && dnp::version_command -h"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dnp::command_help_menu called with args:"
}

@test "dnp::version_command with no arguments and version.txt does not exist › expect error" {
  # Ensure version.txt does not exist
  rm -f "${MOCK_DNP_DIR}/version.txt"

  # Test case: When version command is called without arguments and version.txt does not exist, it should show an error
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/version.bash && dnp::version_command"

  # Should fail
  assert_failure

  # Should output the error message
  assert_output --partial "Error: version.txt not found."
  assert_output --partial "Could not determine Dockerized-NorLab-Project version."
}

@test "dnp::version_command with unknown option › expect option ignored" {
  # Create a mock version.txt file
  mkdir -p "${MOCK_DNP_DIR}"
  echo "1.0.0" > "${MOCK_DNP_DIR}/version.txt"

  # Test case: When version command is called with an unknown option, it should ignore it and display the version
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/version.bash && dnp::version_command --unknown-option"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Dockerized-NorLab-Project version: 1.0.0"
}
