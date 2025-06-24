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

TESTED_FILE="exec.bash"
TESTED_FILE_PATH="src/lib/commands"

# ....Setup........................................................................................
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
  export MOCK_PROJECT_PATH="${BATS_DOCKER_WORKDIR}/utilities/tmp/dockerized-norlab-project-mock"

  # Create temporary directory for tests
  export MOCK_DNP_DIR=$(temp_make)

  # Create mock functions directory in the temporary directory
  mkdir -p "${MOCK_DNP_DIR}/src/lib/core/execute/"
  mkdir -p "${MOCK_DNP_DIR}/src/lib/core/utils/"

  # Create mock functions for dependencies
  cat > "${MOCK_DNP_DIR}/src/lib/core/utils/load_super_project_config.bash" << 'EOF'
#!/bin/bash
# Mock load_super_project_config.bash
echo "Mock load_super_project_config.bash loaded"
return 0
EOF

  cat > "${MOCK_DNP_DIR}/src/lib/core/execute/up_and_attach.bash" << 'EOF'
#!/bin/bash
# Mock up_and_attach.bash
function dnp::up_and_attach() {
  echo "Mock dnp::up_and_attach called with args: $*"
  return 0
}
EOF

  # Create a mock import_dnp_lib.bash that sets up the environment
  cat > "${MOCK_DNP_DIR}/src/lib/core/utils/import_dnp_lib.bash" << 'EOF'
#!/bin/bash
# Mock import_dnp_lib.bash

# ....Setup........................................................................................

# Set message formatting variables
export MSG_DIMMED_FORMAT=""
export MSG_END_FORMAT=""
export MSG_LINE_CHAR_BUILDER_LVL1="-"

# Set up environment variables
export DNP_SPLASH_NAME_FULL="Dockerized-NorLab (DN)"
export DNP_SPLASH_NAME_SMALL="Dockerized-NorLab"
export DNP_ROOT="${MOCK_DNP_DIR}"
export DNP_LIB_PATH="${MOCK_DNP_DIR}/src/lib"
export DNP_LIB_EXEC_PATH="${MOCK_DNP_DIR}/src/lib/core/execute"
export DN_CONTAINER_NAME="mock-container"
export DNP_PROMPT_NAME="Dockerized-NorLab-Project"
export DNP_SPLASH_NAME_FULL="Dockerized-NorLab-Project"
export DNP_SPLASH_NAME_SMALL="Dockerized-NorLab-Project"
export DNP_GIT_REMOTE_URL="https://github.com/norlab-ulaval/dockerized-norlab-project.git"

# ....Mock dependencies loading test functions.....................................................
function dnp::import_lib_and_dependencies() {
  return 0
}

# ....Mock ui.bash functions.......................................................................
function dnp::command_help_menu() {
  echo "Mock dnp::command_help_menu called with args: $*"
  return 0
}

function dnp::illegal_command_msg() {
  echo "Mock dnp::illegal_command_msg called with args: $*"
  return 1
}

function dnp::unknown_subcommand_msg() {
  echo "Mock dnp::unknown_subcommand_msg called with args: $*"
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
for func in $(compgen -A function | grep -e dnp:: -e n2st::); do
  # shellcheck disable=SC2163
  export -f "${func}"
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
  mkdir -p "${MOCK_DNP_DIR}/src/lib/core/execute"

  # Copy the exec.bash file to the temporary directory
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

@test "dnp::exec_command with no arguments › expect default behavior" {
  # Test case: When exec command is called without arguments, it should execute with the default service (develop)
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/exec.bash && dnp::exec_command"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::norlab_splash called with args: Dockerized-NorLab-Project https://github.com/norlab-ulaval/dockerized-norlab-project.git small"
  assert_output --partial "Mock dnp::up_and_attach called with args: --no-up --service develop"
  assert_output --partial "Mock n2st::print_msg called with args: Detached."
}

@test "dnp::exec_command with --help › expect help menu" {
  # Test case: When exec command is called with --help, it should show the help menu
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/exec.bash && dnp::exec_command --help"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dnp::command_help_menu called with args:"
}

@test "dnp::exec_command with -h › expect help menu" {
  # Test case: When exec command is called with -h, it should show the help menu
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/exec.bash && dnp::exec_command -h"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dnp::command_help_menu called with args:"
}

@test "dnp::exec_command with develop service › expect develop service passed to up_and_attach" {
  # Test case: When exec command is called with develop service, it should pass it to up_and_attach
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/exec.bash && dnp::exec_command develop"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dnp::up_and_attach called with args: --no-up --service develop"
}

@test "dnp::exec_command with deploy service › expect deploy service passed to up_and_attach" {
  # Test case: When exec command is called with deploy service, it should pass it to up_and_attach
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/exec.bash && dnp::exec_command deploy"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dnp::up_and_attach called with args: --no-up --service deploy"
}

@test "dnp::exec_command with invalid service › expect service treated as command" {
  # Test case: When exec command is called with an invalid service, it treats it as a command to execute
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/exec.bash && dnp::exec_command invalid-service"

  # Should succeed (exec doesn't validate services, treats them as commands)
  assert_success

  # Should output the expected message with invalid-service passed as command
  assert_output --partial "Mock dnp::up_and_attach called with args: --no-up --service develop invalid-service"
}

@test "dnp::exec_command with multiple services › expect error" {
  # Test case: When exec command is called with multiple services, it should show an error
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/exec.bash && dnp::exec_command develop deploy"

  # Should fail
  assert_failure

  # Should output the error message
  assert_output --partial "Mock dnp::illegal_command_msg called with args: exec"
  assert_output --partial "Only one SERVICE can be specified"
}

@test "dnp::exec_command with --detach option › expect detach flag passed to up_and_attach" {
  # Test case: When exec command is called with --detach option, it should pass it to up_and_attach
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/exec.bash && dnp::exec_command --detach"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dnp::up_and_attach called with args: --no-up --detach"
}

@test "dnp::exec_command with --dry-run option › expect dry-run and detach flags passed to up_and_attach" {
  # Test case: When exec command is called with --dry-run option, it should pass both --dry-run and --detach to up_and_attach
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/exec.bash && dnp::exec_command --dry-run"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dnp::up_and_attach called with args: --no-up --dry-run --detach"
}

@test "dnp::exec_command with --no-TTY option › expect no-TTY flag passed to up_and_attach" {
  # Test case: When exec command is called with --no-TTY option, it should pass it to up_and_attach
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/exec.bash && dnp::exec_command --no-TTY"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dnp::up_and_attach called with args: --no-up --no-TTY"
}

@test "dnp::exec_command with -T option › expect -T flag passed to up_and_attach" {
  # Test case: When exec command is called with -T option, it should pass it to up_and_attach
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/exec.bash && dnp::exec_command -T"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dnp::up_and_attach called with args: --no-up -T"
}

@test "dnp::exec_command with --env option › expect env option passed to up_and_attach" {
  # Test case: When exec command is called with --env option, it should pass it to up_and_attach
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/exec.bash && dnp::exec_command --env VAR=value"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dnp::up_and_attach called with args: --no-up --env VAR=value"
}

@test "dnp::exec_command with -e option › expect -e option passed to up_and_attach" {
  # Test case: When exec command is called with -e option, it should pass it to up_and_attach
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/exec.bash && dnp::exec_command -e VAR=value"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dnp::up_and_attach called with args: --no-up -e VAR=value"
}

@test "dnp::exec_command with --workdir option › expect workdir option passed to up_and_attach" {
  # Test case: When exec command is called with --workdir option, it should pass it to up_and_attach
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/exec.bash && dnp::exec_command --workdir /path/to/dir"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dnp::up_and_attach called with args: --no-up --workdir /path/to/dir"
}

@test "dnp::exec_command with -w option › expect -w option passed to up_and_attach" {
  # Test case: When exec command is called with -w option, it should pass it to up_and_attach
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/exec.bash && dnp::exec_command -w /path/to/dir"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dnp::up_and_attach called with args: --no-up -w /path/to/dir"
}

@test "dnp::exec_command with -- and command › expect command passed to up_and_attach" {
  # Test case: When exec command is called with -- followed by a command, it should pass the command to up_and_attach
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/exec.bash && dnp::exec_command -- bash -c 'echo hello'"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dnp::up_and_attach called with args: --no-up --service develop -- bash -c echo hello"
}

@test "dnp::exec_command with multiple options and command › expect all options and command passed to up_and_attach" {
  # Test case: When exec command is called with multiple options and a command, it should pass all to up_and_attach
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/exec.bash && dnp::exec_command deploy --workdir /path --detach -- bash -c 'echo hello'"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dnp::up_and_attach called with args: --no-up --workdir /path --detach --service deploy -- bash -c echo hello"
}

@test "dnp::exec_command with invalid option › expect option treated as command" {
  # Test case: When exec command is called with an invalid option, it treats it as a command to execute
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/exec.bash && dnp::exec_command --invalid-option"

  # Should succeed (exec doesn't validate options, treats them as commands)
  assert_success

  # Should output the expected message with invalid-option passed as command
  assert_output --partial "Mock dnp::up_and_attach called with args: --no-up --service develop --invalid-option"
}
