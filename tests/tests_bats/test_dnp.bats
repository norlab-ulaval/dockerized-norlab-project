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

TESTED_FILE="dnp"
TESTED_FILE_PATH="src/bin"

# ....Setup........................................................................................
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
  # This is the path to the mock super project (the user side)
  export MOCK_PROJECT_PATH="${BATS_DOCKER_WORKDIR}/utilities/tmp/dockerized-norlab-project-mock"

  # Create temporary directory for tests
  export MOCK_DNP_DIR=$(temp_make)

  # Create mock functions directory in the temporary directory
  mkdir -p "${MOCK_DNP_DIR}/src/lib/commands/"
  mkdir -p "${MOCK_DNP_DIR}/src/lib/core/utils/"

  # Create mock functions for each command
  cat > "${MOCK_DNP_DIR}/src/lib/commands/init.bash" << 'EOF'
function dnp::init_command() {
  echo "Mock dnp::init_command called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNP_DIR}/src/lib/commands/build.bash" << 'EOF'
function dnp::build_command() {
  echo "Mock dnp::build_command called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNP_DIR}/src/lib/commands/up.bash" << 'EOF'
function dnp::up_command() {
  echo "Mock dnp::up_command called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNP_DIR}/src/lib/commands/down.bash" << 'EOF'
function dnp::down_command() {
  echo "Mock dnp::down_command called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNP_DIR}/src/lib/commands/run.bash" << 'EOF'
function dnp::run_command() {
  echo "Mock dnp::run_command called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNP_DIR}/src/lib/commands/config.bash" << 'EOF'
function dnp::config_command() {
  echo "Mock dnp::config_command called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNP_DIR}/src/lib/commands/version.bash" << 'EOF'
function dnp::version_command() {
  echo "Mock dnp::version_command called with args: $*"
  return 0
}
EOF


  cat > "${MOCK_DNP_DIR}/src/lib/commands/project.bash" << 'EOF'
DOCUMENTATION_BUFFER_PROJECT="
# =================================================================================================
# Super project DNP configuration related command
#
# Usage:
# Commands:
# =================================================================================================
"

function dnp::project_validate_command() {
  echo "Mock dnp::project_validate_command called with args: $*"
  return 0
}

function dnp::project_sanity_command() {
  echo "Mock dnp::project_sanity_command called with args: $*"
  return 0
}

function dnp::project_dotenv_command() {
  echo "Mock dnp::project_dotenv_command called with args: $*"
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

# Set up environment variables
export DNP_ROOT="${MOCK_DNP_DIR}"
export DNP_LIB_PATH="${DNP_ROOT}/src/lib"

# ....Mock dependencies loading test functions.....................................................
function dnp::import_lib_and_dependencies() {
  return 0
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

function n2st::echo_centering_str() {
  echo "Mock n2st::echo_centering_str called with args: $*"
  return 0
}

function n2st::draw_horizontal_line_across_the_terminal_window() {
  echo "Mock n2st::draw_horizontal_line_across_the_terminal_window called with args: $*"
  return 0
}

# ....Load DNP lib functions.......................................................................
source "${DNP_LIB_PATH:?err}/core/utils/ui.bash" || exit 1

# ....Export loaded functions......................................................................
for func in $(compgen -A function | grep -e dnp:: -e nbs:: -e n2st::); do
  export -f "$func"
done

# ....Teardown.....................................................................................
# Print a message to indicate that the mock import_dnp_lib.bash has been loaded
echo "[DNP done] Mock import_dnp_lib.bash and its librairies loaded"
EOF
}

setup() {

  # Create the src/bin directory in the mock project
  mkdir -p "${MOCK_DNP_DIR}/src/bin"
  mkdir -p "${MOCK_DNP_DIR}/lib/core/utils/"

  # Copy the dnp script to the src/bin directory in the mock project
  cp "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" "${MOCK_DNP_DIR}/src/bin/"
  cp "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/ui.bash" "${MOCK_DNP_DIR}/src/lib/core/utils/"

  # Make the dnp script executable
  chmod +x "${MOCK_DNP_DIR}/src/bin/dnp"

  # Change cwd to the mock super project directory
  cd "${MOCK_PROJECT_PATH}" || exit 1
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

@test "dnp with no arguments › expect help message and no failure" {

  # Test case: When dnp is called without arguments, it should show help and exit
  run bash "${MOCK_DNP_DIR}"/src/bin/dnp

  # Should succeed
  assert_success

  # Should output help message
  assert_output --partial "Usage:"
  assert_output --partial "Commands:"
}

@test "dnp init command › expect init function to be called" {
  # Test case: When dnp is called with 'init' command, it should call the init function
  run bash "${MOCK_DNP_DIR}"/src/bin/dnp init

  # Should succeed
  assert_success

  # Should call the init function
  assert_output --partial "Mock dnp::init_command called with args:"
}

@test "dnp build command › expect build function to be called" {
  # Test case: When dnp is called with 'build' command, it should call the build function
  run bash "${MOCK_DNP_DIR}"/src/bin/dnp build

  # Should succeed
  assert_success

  # Should call the build function
  assert_output --partial "Mock dnp::build_command called with args:"
}

@test "dnp up command › expect up function to be called" {
  # Test case: When dnp is called with 'up' command, it should call the up function
  run bash "${MOCK_DNP_DIR}"/src/bin/dnp up

  # Should succeed
  assert_success

  # Should call the up function
  assert_output --partial "Mock dnp::up_command called with args:"
}

@test "dnp down command › expect down function to be called" {
  # Test case: When dnp is called with 'down' command, it should call the down function
  run bash "${MOCK_DNP_DIR}"/src/bin/dnp down

  # Should succeed
  assert_success

  # Should call the down function
  assert_output --partial "Mock dnp::down_command called with args:"
}

@test "dnp run command › expect run function to be called" {
  # Test case: When dnp is called with 'run' command, it should call the run function
  run bash "${MOCK_DNP_DIR}"/src/bin/dnp run

  # Should succeed
  assert_success

  # Should call the run function
  assert_output --partial "Mock dnp::run_command called with args:"
}

@test "dnp config command › expect config function to be called" {
  # Test case: When dnp is called with 'config' command, it should call the config function
  run bash "${MOCK_DNP_DIR}"/src/bin/dnp config

  # Should succeed
  assert_success

  # Should call the config function
  assert_output --partial "Mock dnp::config_command called with args:"
}

@test "dnp version command › expect version function to be called" {
  # Test case: When dnp is called with 'version' command, it should call the version function
  run bash "${MOCK_DNP_DIR}"/src/bin/dnp version

  # Should succeed
  assert_success

  # Should call the version function
  assert_output --partial "Mock dnp::version_command called with args:"
}

@test "dnp help command › expect help message and success" {
  # Test case: When dnp is called with 'help' command, it should show help and exit successfully
  run bash "${MOCK_DNP_DIR}"/src/bin/dnp help

  # Should succeed with exit code 0
  assert_success

  # Should output help message
  assert_output --partial "Usage:"
  assert_output --partial "Commands:"
}

@test "dnp with unknown command › expect error message and failure" {
  # Test case: When dnp is called with an unknown command, it should show error and help, and exit with error
  run bash "${MOCK_DNP_DIR}"/src/bin/dnp unknown_command

  # Should fail with exit code 1
  assert_failure 1

  # Should output error message
  assert_output --partial "Unknown command dnp unknown_command"
}

@test "dnp project validate command › expect project_validate function to be called" {
  # Test case: When dnp is called with 'project validate' command, it should call the project_validate function
  run bash "${MOCK_DNP_DIR}"/src/bin/dnp project validate

  # Should succeed
  assert_success

  # Should call the project_validate function
  assert_output --partial "Mock dnp::project_validate_command called with args:"
}

@test "dnp project sanity command › expect project_sanity function to be called" {
  # Test case: When dnp is called with 'project sanity' command, it should call the project_sanity function
  run bash "${MOCK_DNP_DIR}"/src/bin/dnp project sanity

  # Should succeed
  assert_success

  # Should call the project_sanity function
  assert_output --partial "Mock dnp::project_sanity_command called with args:"
}

@test "dnp project dotenv command › expect project_dotenv function to be called" {
  # Test case: When dnp is called with 'project dotenv' command, it should call the project_dotenv function
  run bash "${MOCK_DNP_DIR}"/src/bin/dnp project dotenv

  # Should succeed
  assert_success

  # Should call the project_dotenv function
  assert_output --partial "Mock dnp::project_dotenv_command called with args:"
}

@test "dnp project help command › expect help message and no failllure" {
  # Test case: When dnp is called with 'project help' command, it should show help and exit with error
  run bash "${MOCK_DNP_DIR}"/src/bin/dnp project help

  # Should succeed
  assert_success

  # Should output help message for project command
  assert_output --partial "Usage:"
  assert_output --partial "Commands:"
}

@test "dnp project with unknown subcommand › expect error message and failure" {
  # Test case: When dnp is called with an unknown project subcommand, it should show error and help, and exit with error
  run bash "${MOCK_DNP_DIR}"/src/bin/dnp project unknown_subcommand

  # Should fail with exit code 1
  assert_failure 1

  # Should output error message
  assert_output --partial "Unknown command dnp project unknown_subcommand"
}
