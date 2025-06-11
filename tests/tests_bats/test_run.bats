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

TESTED_FILE="run.bash"
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

  cat > "${MOCK_DNP_DIR}/src/lib/core/execute/run.ci_tests.bash" << 'EOF'
#!/bin/bash
# Mock run.ci_tests.bash
function dnp::run_ci_tests() {
  echo "Mock dnp::run_ci_tests called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNP_DIR}/src/lib/core/execute/run.slurm.bash" << 'EOF'
#!/bin/bash
# Mock run.slurm.bash
function dnp::run_slurm() {
  echo "Mock dnp::run_slurm called with args: $*"
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
export DNP_ROOT="${MOCK_DNP_DIR}"
export DNP_LIB_PATH="${MOCK_DNP_DIR}/src/lib"
export DNP_LIB_EXEC_PATH="${MOCK_DNP_DIR}/src/lib/core/execute"

# ....Mock dependencies loading test functions.....................................................
function dnp::import_lib_and_dependencies() {
  return 0
}

# ....Mock ui.bash functions.......................................................................
function dnp::command_help_menu() {
  echo "Mock dnp::command_help_menu called with args: $*"
  return 0
}

# ....Mock N2ST functions..........................................................................
function n2st::print_msg() {
  echo "Mock n2st::print_msg called with args: $*"
  return 0
}

function n2st::print_msg_error() {
  echo "Mock n2st::print_msg_error called with args: $*"
  return 0
}

# ....Mock dnp build function......................................................................
function dnp() {
  if [[ "$1" == "build" && "$2" == "--ci-tests" ]]; then
    echo "Mock dnp build --ci-tests called"
    return 0
  fi
  echo "Mock dnp called with args: $*"
  return 0
}

# ....Export mock functions........................................................................
for func in $(compgen -A function | grep -e dnp:: -e n2st::); do
  export -f "$func"
done

# ....Export dnp function..........................................................................
export -f dnp

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

  # Copy the run.bash file to the temporary directory
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

@test "dnp::run_command with no arguments › expect error message and help menu" {
  # Test case: When run command is called without arguments, it should show an error message and help menu
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command"

  # Should succeed (the function itself doesn't fail, it just shows an error message)
  assert_success

  # Should output the expected error message and help menu
  assert_output --partial "Mock n2st::print_msg_error called with args: No run service specified."
  assert_output --partial "Mock dnp::command_help_menu called with args:"
}

@test "dnp::run_command with --help › expect help menu" {
  # Test case: When run command is called with --help, it should show the help menu
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command --help"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dnp::command_help_menu called with args:"
}

@test "dnp::run_command with -h › expect help menu" {
  # Test case: When run command is called with -h, it should show the help menu
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command -h"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dnp::command_help_menu called with args:"
}

@test "dnp::run_command with ci-tests › expect ci-tests to be called" {
  # Test case: When run command is called with ci-tests, it should call dnp::run_ci_tests
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command ci-tests"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Running CI tests..."
  assert_output --partial "Mock dnp build --ci-tests called"
  assert_output --partial "Mock dnp::run_ci_tests called with args:"
}

@test "dnp::run_command with ci-tests and additional arguments › expect arguments passed to run_ci_tests" {
  # Test case: When run command is called with ci-tests and additional arguments, it should pass them to dnp::run_ci_tests
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command ci-tests --arg1 --arg2"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Running CI tests..."
  assert_output --partial "Mock dnp build --ci-tests called"
  assert_output --partial "Mock dnp::run_ci_tests called with args: --arg1 --arg2"
}

@test "dnp::run_command with ci-tests -- --help › expect help flag passed to run_ci_tests" {
  # Test case: When run command is called with ci-tests -- --help, it should call dnp::run_ci_tests with --help
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command ci-tests -- --help"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Mock dnp::run_ci_tests called with args: --help"
}

@test "dnp::run_command with slurm › expect slurm to be called" {
  # Test case: When run command is called with slurm, it should call dnp::run_slurm
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command slurm"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Running slurm containers..."
  assert_output --partial "Mock dnp::run_slurm called with args:"
}

@test "dnp::run_command with slurm and job id › expect job id passed to run_slurm" {
  # Test case: When run command is called with slurm and a job id, it should pass it to dnp::run_slurm
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command slurm 12345"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Running slurm containers..."
  assert_output --partial "Mock dnp::run_slurm called with args: 12345"
}

@test "dnp::run_command with slurm -- --help › expect help flag passed to run_slurm" {
  # Test case: When run command is called with slurm -- --help, it should call dnp::run_slurm with --help
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command slurm -- --help"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Running slurm containers..."
  assert_output --partial "Mock dnp::run_slurm called with args: --help"
}

@test "dnp::run_command with unknown service › expect error message and help menu" {
  # Test case: When run command is called with an unknown service, it should show an error message and help menu
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command unknown-service"

  # Should succeed (the function itself doesn't fail, it just shows an error message)
  assert_success

  # Should output the expected error message and help menu
  assert_output --partial "Mock n2st::print_msg_error called with args: No run service specified."
  assert_output --partial "Mock dnp::command_help_menu called with args:"
}
