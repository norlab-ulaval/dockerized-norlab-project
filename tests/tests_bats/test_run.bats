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

  # Create mock run.any.bash
  cat > "${MOCK_DNP_DIR}/src/lib/core/execute/run.any.bash" << 'EOF'
#!/bin/bash
# Mock run.any.bash
function dnp::run_any() {
  echo "Mock dnp::run_any called with args: $*"
  return 0
}
EOF

  # Create mock up_and_attach.bash
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

# ....Mock N2ST functions..........................................................................
function n2st::norlab_splash() {
  echo "Mock n2st::norlab_splash called with args: $*"
  return 0
}

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
  if [[ "$1" == "build" && "$2" == "ci-tests" ]]; then
    echo "Mock dnp build ci-tests called"
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

  # Should fail (the function returns 1 when no service is specified)
  assert_failure

  # Should output the expected error message and help menu
  assert_output --partial "Mock n2st::print_msg_error called with args: Service is either unknown or not specified."
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
  assert_output --partial "Mock dnp build ci-tests called"
  assert_output --partial "Mock dnp::run_ci_tests called with args:"
}

@test "dnp::run_command with ci-tests and additional arguments › expect arguments passed to run_ci_tests" {
  # Test case: When run command is called with ci-tests and additional arguments, it should pass them to dnp::run_ci_tests
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command ci-tests --arg1 --arg2"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Running CI tests..."
  assert_output --partial "Mock dnp build ci-tests called"
  assert_output --partial "Mock dnp::run_ci_tests called with args: --arg1 --arg2"
}

@test "dnp::run_command with ci-tests -- --help › expect help flag passed to run_ci_tests" {
  # Test case: When run command is called with ci-tests -- --help, it should call dnp::run_ci_tests with -- --help
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command ci-tests -- --help"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Mock dnp::run_ci_tests called with args: -- --help"
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
  # Test case: When run command is called with slurm -- --help, it should call dnp::run_slurm with -- --help
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command slurm -- --help"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Running slurm containers..."
  assert_output --partial "Mock dnp::run_slurm called with args: -- --help"
}

@test "dnp::run_command with develop › expect develop service to be called" {
  # Test case: When run command is called with develop, it should call dnp::run_any with project-develop service
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command develop"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Running develop containers..."
  assert_output --partial "Mock dnp::run_any called with args: --service project-develop"
}

@test "dnp::run_command with develop and command › expect command passed to run_any" {
  # Test case: When run command is called with develop and a command, it should pass it to dnp::run_any
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command develop -- ls -la"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Running develop containers..."
  assert_output --partial "Mock dnp::run_any called with args: --service project-develop -- ls -la"
}

@test "dnp::run_command with develop and multiple options › expect all options passed to run_any" {
  # Test case: When run command is called with develop and multiple options, it should pass them all to dnp::run_any
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command -e VAR1=value1 -e VAR2=value2 -w /workdir -T -v /host:/container --detach develop -- ls -la"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Running develop containers..."
  assert_output --partial "Mock dnp::run_any called with args: --service project-develop -e VAR1=value1 -e VAR2=value2 -w /workdir -T -v /host:/container --detach -- ls -la"
}

@test "dnp::run_command with deploy › expect deploy service to be called" {
  # Test case: When run command is called with deploy, it should call dnp::run_any with project-deploy service
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command deploy"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Running deploy containers..."
  assert_output --partial "Mock dnp::run_any called with args: --service project-deploy"
}

@test "dnp::run_command with deploy and command › expect command passed to run_any" {
  # Test case: When run command is called with deploy and a command, it should pass it to dnp::run_any
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command deploy -- ls -la"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Running deploy containers..."
  assert_output --partial "Mock dnp::run_any called with args: --service project-deploy -- ls -la"
}

@test "dnp::run_command with deploy and options › expect options passed to run_any" {
  # Test case: When run command is called with deploy and options, it should pass them to dnp::run_any
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command -e VAR=value -w /workdir deploy"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Running deploy containers..."
  assert_output --partial "Mock dnp::run_any called with args: --service project-deploy -e VAR=value -w /workdir"
}

@test "dnp::run_command with unknown service › expect error message and help menu" {
  # Test case: When run command is called with an unknown service, it should show an error message and help menu
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command unknown-service"

  # Should fail (the function returns 1 for unknown service)
  assert_failure

  # Should output the expected error message and help menu
  assert_output --partial "Mock n2st::print_msg_error called with args: Service is either unknown or not specified."
  assert_output --partial "Mock dnp::command_help_menu called with args:"
}

# ====New test cases for service-specific help options============================================

@test "dnp::run_command with --help-develop › expect develop help menu" {
  # Test case: When run command is called with --help-develop, it should show the develop help menu
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command --help-develop"

  # Should succeed
  assert_success

  # Should output the develop help menu
  assert_output --partial "Mock dnp::command_help_menu called with args:"
}

@test "dnp::run_command with --help-deploy › expect deploy help menu" {
  # Test case: When run command is called with --help-deploy, it should show the deploy help menu
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command --help-deploy"

  # Should succeed
  assert_success

  # Should output the deploy help menu
  assert_output --partial "Mock dnp::command_help_menu called with args:"
}

@test "dnp::run_command with --help-slurm › expect slurm help menu" {
  # Test case: When run command is called with --help-slurm, it should show the slurm help menu
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command --help-slurm"

  # Should succeed
  assert_success

  # Should output the slurm help menu
  assert_output --partial "Mock dnp::command_help_menu called with args:"
}

@test "dnp::run_command with --help-ci-tests › expect ci-tests help menu" {
  # Test case: When run command is called with --help-ci-tests, it should show the ci-tests help menu
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command --help-ci-tests"

  # Should succeed
  assert_success

  # Should output the ci-tests help menu
  assert_output --partial "Mock dnp::command_help_menu called with args:"
}

# ====New test cases for multiple services error handling=========================================

@test "dnp::run_command with multiple services › expect error message" {
  # Test case: When run command is called with multiple services, it should show an error message
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command develop deploy"

  # Should fail (the function returns 1 for multiple services)
  assert_failure

  # Should output the expected error message
  assert_output --partial "Mock dnp::illegal_command_msg called with args: run develop deploy Only one SERVICE can be specified."
}

@test "dnp::run_command with ci-tests and slurm › expect error message" {
  # Test case: When run command is called with ci-tests and slurm, it should show an error message
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command ci-tests slurm"

  # Should fail (the function returns 1 for multiple services)
  assert_failure

  # Should output the expected error message
  assert_output --partial "Mock dnp::illegal_command_msg called with args: run ci-tests slurm Only one SERVICE can be specified."
}

# ====New test cases for individual option testing===============================================

@test "dnp::run_command with --detach option and develop › expect detach flag passed" {
  # Test case: When run command is called with --detach option, it should pass the flag to the service
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command --detach develop"

  # Should succeed
  assert_success

  # Should output the expected messages with detach flag
  assert_output --partial "Mock n2st::print_msg called with args: Running develop containers..."
  assert_output --partial "Mock dnp::run_any called with args: --service project-develop --detach"
}

@test "dnp::run_command with --dry-run option and develop › expect dry-run and detach flags passed" {
  # Test case: When run command is called with --dry-run option, it should pass both --dry-run and --detach flags
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command --dry-run develop"

  # Should succeed
  assert_success

  # Should output the expected messages with dry-run and detach flags
  assert_output --partial "Mock n2st::print_msg called with args: Running develop containers..."
  assert_output --partial "Mock dnp::run_any called with args: --service project-develop --dry-run --detach"
}

@test "dnp::run_command with -T option and develop › expect no-TTY flag passed" {
  # Test case: When run command is called with -T option, it should pass the flag to the service
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command -T develop"

  # Should succeed
  assert_success

  # Should output the expected messages with -T flag
  assert_output --partial "Mock n2st::print_msg called with args: Running develop containers..."
  assert_output --partial "Mock dnp::run_any called with args: --service project-develop -T"
}

@test "dnp::run_command with --no-TTY option and develop › expect no-TTY flag passed" {
  # Test case: When run command is called with --no-TTY option, it should pass the flag to the service
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command --no-TTY develop"

  # Should succeed
  assert_success

  # Should output the expected messages with --no-TTY flag
  assert_output --partial "Mock n2st::print_msg called with args: Running develop containers..."
  assert_output --partial "Mock dnp::run_any called with args: --service project-develop --no-TTY"
}

# ====New test cases for parameter options====================================================

@test "dnp::run_command with -e option and develop › expect env flag and value passed" {
  # Test case: When run command is called with -e option, it should pass the flag and value to the service
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command -e TEST_VAR=value develop"

  # Should succeed
  assert_success

  # Should output the expected messages with -e flag and value
  assert_output --partial "Mock n2st::print_msg called with args: Running develop containers..."
  assert_output --partial "Mock dnp::run_any called with args: --service project-develop -e TEST_VAR=value"
}

@test "dnp::run_command with --env option and develop › expect env flag and value passed" {
  # Test case: When run command is called with --env option, it should pass the flag and value to the service
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command --env TEST_VAR=value develop"

  # Should succeed
  assert_success

  # Should output the expected messages with --env flag and value
  assert_output --partial "Mock n2st::print_msg called with args: Running develop containers..."
  assert_output --partial "Mock dnp::run_any called with args: --service project-develop --env TEST_VAR=value"
}

@test "dnp::run_command with -w option and develop › expect workdir flag and value passed" {
  # Test case: When run command is called with -w option, it should pass the flag and value to the service
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command -w /test/workdir develop"

  # Should succeed
  assert_success

  # Should output the expected messages with -w flag and value
  assert_output --partial "Mock n2st::print_msg called with args: Running develop containers..."
  assert_output --partial "Mock dnp::run_any called with args: --service project-develop -w /test/workdir"
}

@test "dnp::run_command with --workdir option and develop › expect workdir flag and value passed" {
  # Test case: When run command is called with --workdir option, it should pass the flag and value to the service
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command --workdir /test/workdir develop"

  # Should succeed
  assert_success

  # Should output the expected messages with --workdir flag and value
  assert_output --partial "Mock n2st::print_msg called with args: Running develop containers..."
  assert_output --partial "Mock dnp::run_any called with args: --service project-develop --workdir /test/workdir"
}

@test "dnp::run_command with -v option and develop › expect volume flag and value passed" {
  # Test case: When run command is called with -v option, it should pass the flag and value to the service
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command -v /host:/container develop"

  # Should succeed
  assert_success

  # Should output the expected messages with -v flag and value
  assert_output --partial "Mock n2st::print_msg called with args: Running develop containers..."
  assert_output --partial "Mock dnp::run_any called with args: --service project-develop -v /host:/container"
}

@test "dnp::run_command with --volume option and develop › expect volume flag and value passed" {
  # Test case: When run command is called with --volume option, it should pass the flag and value to the service
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command --volume /host:/container develop"

  # Should succeed
  assert_success

  # Should output the expected messages with --volume flag and value
  assert_output --partial "Mock n2st::print_msg called with args: Running develop containers..."
  assert_output --partial "Mock dnp::run_any called with args: --service project-develop --volume /host:/container"
}

# ====New test cases for option combinations with different services=========================

@test "dnp::run_command with options and ci-tests › expect options passed to run_ci_tests" {
  # Test case: When run command is called with options and ci-tests, it should pass options to dnp::run_ci_tests
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command -e TEST_VAR=value --detach ci-tests"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Running CI tests..."
  assert_output --partial "Mock dnp build ci-tests called"
  assert_output --partial "Mock dnp::run_ci_tests called with args: -e TEST_VAR=value --detach"
}

@test "dnp::run_command with options and slurm › expect options passed to run_slurm" {
  # Test case: When run command is called with options and slurm, it should pass options to dnp::run_slurm
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command -w /workdir slurm job123"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Running slurm containers..."
  assert_output --partial "Mock dnp::run_slurm called with args: -w /workdir job123"
}

@test "dnp::run_command with complex option combination and deploy › expect all options passed" {
  # Test case: When run command is called with complex options and deploy, it should pass all options correctly
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/run.bash && dnp::run_command -e VAR1=val1 -e VAR2=val2 -w /workdir -v /host1:/cont1 -v /host2:/cont2 --detach -T deploy -- bash -c 'echo test'"

  # Should succeed
  assert_success

  # Should output the expected messages with all options
  assert_output --partial "Mock n2st::print_msg called with args: Running deploy containers..."
  assert_output --partial "Mock dnp::run_any called with args: --service project-deploy -e VAR1=val1 -e VAR2=val2 -w /workdir -v /host1:/cont1 -v /host2:/cont2 --detach -T -- bash -c echo test"
}
