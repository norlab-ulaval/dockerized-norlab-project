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

TESTED_FILE="up.bash"
TESTED_FILE_PATH="src/lib/commands"

# ....Setup........................................................................................
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
  export MOCK_PROJECT_PATH="${BATS_DOCKER_WORKDIR}/utilities/tmp/dockerized-norlab-project-mock"

  # Create temporary directory for tests
  export MOCK_DNA_DIR=$(temp_make)

  # Create mock functions directory in the temporary directory
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/execute/"
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/utils/"

  # Create mock functions for dependencies
  cat > "${MOCK_DNA_DIR}/src/lib/core/utils/load_super_project_config.bash" << 'EOF'
#!/bin/bash
# Mock load_super_project_config.bash
echo "Mock load_super_project_config.bash loaded"
return 0
EOF

  cat > "${MOCK_DNA_DIR}/src/lib/core/execute/up_and_attach.bash" << 'EOF'
#!/bin/bash
# Mock up_and_attach.bash
function dna::up_and_attach() {
  echo "Mock dna::up_and_attach called with args: $*"
  return 0
}
EOF

  # Create a mock import_dna_lib.bash that sets up the environment
  cat > "${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash" << 'EOF'
#!/bin/bash
# Mock import_dna_lib.bash

# ....Setup........................................................................................

# Set message formatting variables
export MSG_DIMMED_FORMAT=""
export MSG_END_FORMAT=""
export MSG_LINE_CHAR_BUILDER_LVL1="-"

# Set up environment variables
export DNA_SPLASH_NAME_FULL="Dockerized-NorLab (DN)"
export DNA_SPLASH_NAME_SMALL="Dockerized-NorLab"
export DNA_ROOT="${MOCK_DNA_DIR}"
export DNA_LIB_PATH="${MOCK_DNA_DIR}/src/lib"
export DNA_LIB_EXEC_PATH="${MOCK_DNA_DIR}/src/lib/core/execute"
export DNA_PROMPT_NAME="Dockerized-NorLab Project"
export DNA_SPLASH_NAME_FULL="Dockerized-NorLab Project"
export DNA_SPLASH_NAME_SMALL="Dockerized-NorLab Project"
export DNA_GIT_REMOTE_URL="https://github.com/norlab-ulaval/dockerized-norlab-project.git"

# ....Mock dependencies loading test functions.....................................................
function dna::import_lib_and_dependencies() {
  return 0
}

# ....Mock ui.bash functions.......................................................................
function dna::command_help_menu() {
  echo "Mock dna::command_help_menu called with args: $*"
  return 0
}

function dna::illegal_command_msg() {
  echo "Mock dna::illegal_command_msg called with args: $*"
  return 1
}

function dna::unknown_subcommand_msg() {
  echo "Mock dna::unknown_subcommand_msg called with args: $*"
  return 1
}

# ....Mock N2ST functions..........................................................................
function n2st::norlab_splash() {
  echo "Mock n2st::norlab_splash called with args: $*"
  return 0
}

function n2st::print_msg() {
  echo "Mock n2st::print_msg called with args: $*"
  return 0
}

# ....Export mock functions........................................................................
for func in $(compgen -A function | grep -e dna:: -e n2st::); do
  # shellcheck disable=SC2163
  export -f "${func}"
done

# ....Teardown.....................................................................................
# Print a message to indicate that the mock import_dna_lib.bash has been loaded
echo "[DNA done] Mock import_dna_lib.bash and its librairies loaded"
EOF
}

setup() {
  # Create necessary directories in the temporary directory
  mkdir -p "${MOCK_DNA_DIR}/src/lib/commands"
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/utils"
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/execute"

  # Copy the up.bash file to the temporary directory
  cp "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" "${MOCK_DNA_DIR}/src/lib/commands/"

  source "${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash" || exit 1

  # Change to the temporary directory
  cd "${MOCK_DNA_DIR}" || exit 1
}

# ....Teardown.....................................................................................
teardown() {
  bats_print_run_env_variable_on_error
}

teardown_file() {
  # Clean up temporary directory
  temp_del "${MOCK_DNA_DIR}"
}

# ====Test cases==================================================================================

@test "dna::up_command with no arguments › expect default behavior" {
  # Test case: When up command is called without arguments, it should start and attach to the default service
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/up.bash && dna::up_command"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::norlab_splash called with args: Dockerized-NorLab Project https://github.com/norlab-ulaval/dockerized-norlab-project.git small"
  assert_output --partial "Mock dna::up_and_attach called with args:"
  assert_output --partial "Mock n2st::print_msg called with args: Detached."
}

@test "dna::up_command with --help › expect help menu" {
  # Test case: When up command is called with --help, it should show the help menu
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/up.bash && dna::up_command --help"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}

@test "dna::up_command with -h › expect help menu" {
  # Test case: When up command is called with -h, it should show the help menu
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/up.bash && dna::up_command -h"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}

@test "dna::up_command with custom command › expect command passed to up_and_attach" {
  # Test case: When up command is called with a custom command, it should pass it to up_and_attach
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/up.bash && dna::up_command bash -c 'echo hello'"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dna::up_and_attach called with args: --service develop bash -c echo hello"
}

@test "dna::up_command with --arbitrary-flag option › expect service passed to up_and_attach" {
  # Test case: When up command is called with --arbitrary-flag option, it should pass it to up_and_attach
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/up.bash && dna::up_command --arbitrary-flag flag-option"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dna::up_and_attach called with args: --service develop --arbitrary-flag flag-option"
}

@test "dna::up_command with --arbitrary-flag option and custom command › expect both passed to up_and_attach" {
  # Test case: When up command is called with --arbitrary-flag option and custom command, it should pass both to up_and_attach
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/up.bash && dna::up_command --arbitrary-flag flag-option bash -c 'echo hello'"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dna::up_and_attach called with args: --service develop --arbitrary-flag flag-option bash -c echo hello"
}

# ====New test cases for service selection====================================================

@test "dna::up_command with develop service › expect develop service passed to up_and_attach" {
  # Test case: When up command is called with develop service, it should pass it to up_and_attach
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/up.bash && dna::up_command develop"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dna::up_and_attach called with args: --service develop"
}

@test "dna::up_command with deploy service › expect deploy service passed to up_and_attach" {
  # Test case: When up command is called with deploy service, it should pass it to up_and_attach
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/up.bash && dna::up_command deploy"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dna::up_and_attach called with args: --service deploy"
}

@test "dna::up_command with develop service and command › expect both passed to up_and_attach" {
  # Test case: When up command is called with develop service and command, it should pass both to up_and_attach
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/up.bash && dna::up_command develop -- bash -c 'echo hello'"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dna::up_and_attach called with args: --service develop -- bash -c echo hello"
}

@test "dna::up_command with deploy service and command › expect both passed to up_and_attach" {
  # Test case: When up command is called with deploy service and command, it should pass both to up_and_attach
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/up.bash && dna::up_command deploy -- bash -c 'echo hello'"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dna::up_and_attach called with args: --service deploy -- bash -c echo hello"
}

# ====New test cases for error handling=======================================================

@test "dna::up_command with multiple services › expect error" {
  # Test case: When up command is called with multiple services, it should show an error
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/up.bash && dna::up_command develop deploy"

  # Should fail
  assert_failure

  # Should output the error message
  assert_output --partial "Mock dna::illegal_command_msg called with args: up"
  assert_output --partial "Only one SERVICE can be specified"
}

@test "dna::up_command with --no-up flag › expect error" {
  # Test case: When up command is called with --no-up flag, it should show an error as it's internal
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/up.bash && dna::up_command --no-up"

  # Should fail
  assert_failure

  # Should output the error message
  assert_output --partial "Mock dna::illegal_command_msg called with args: up --no-up Its a dna internal flag"
}

