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

TESTED_FILE="dna"
TESTED_FILE_PATH="src/bin"

# ....Setup........................................................................................
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
  # This is the path to the mock super project (the user side)
  export MOCK_PROJECT_PATH="${BATS_DOCKER_WORKDIR}/utilities/tmp/dockerized-norlab-project-mock"

  # Create temporary directory for tests
  export MOCK_DNA_DIR=$(temp_make)

  # Create mock functions directory in the temporary directory
  mkdir -p "${MOCK_DNA_DIR}/src/lib/commands/"
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/utils/"

  # Create mock functions for each command
  cat > "${MOCK_DNA_DIR}/src/lib/commands/init.bash" << 'EOF'
function dna::init_command() {
  echo "Mock dna::init_command called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNA_DIR}/src/lib/commands/build.bash" << 'EOF'
function dna::build_command() {
  echo "Mock dna::build_command called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNA_DIR}/src/lib/commands/up.bash" << 'EOF'
function dna::up_command() {
  echo "Mock dna::up_command called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNA_DIR}/src/lib/commands/down.bash" << 'EOF'
function dna::down_command() {
  echo "Mock dna::down_command called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNA_DIR}/src/lib/commands/run.bash" << 'EOF'
function dna::run_command() {
  echo "Mock dna::run_command called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNA_DIR}/src/lib/commands/config.bash" << 'EOF'
function dna::config_command() {
  echo "Mock dna::config_command called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNA_DIR}/src/lib/commands/version.bash" << 'EOF'
function dna::version_command() {
  echo "Mock dna::version_command called with args: $*"
  return 0
}
EOF


  cat > "${MOCK_DNA_DIR}/src/lib/commands/project.bash" << 'EOF'
DOCUMENTATION_BUFFER_PROJECT="
# =================================================================================================
# Super project DNA configuration related command
#
# Usage:
# Commands:
# =================================================================================================
"

function dna::project_validate_command() {
  echo "Mock dna::project_validate_command called with args: $*"
  return 0
}

function dna::project_sanity_command() {
  echo "Mock dna::project_sanity_command called with args: $*"
  return 0
}

function dna::project_dotenv_command() {
  echo "Mock dna::project_dotenv_command called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNA_DIR}/src/lib/commands/exec.bash" << 'EOF'
function dna::exec_command() {
  echo "Mock dna::exec_command called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNA_DIR}/src/lib/commands/attach.bash" << 'EOF'
function dna::attach_command() {
  echo "Mock dna::attach_command called with args: $*"
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

# Set up environment variables
export DNA_SPLASH_NAME_FULL="Dockerized-NorLab (DN)"
export DNA_SPLASH_NAME_SMALL="Dockerized-NorLab"
export DNA_ROOT="${MOCK_DNA_DIR}"
export DNA_LIB_PATH="${DNA_ROOT}/src/lib"
export DNA_PROMPT_NAME="Dockerized-NorLab Project"
export DNA_SPLASH_NAME_FULL="Dockerized-NorLab Project"
export DNA_SPLASH_NAME_SMALL="Dockerized-NorLab Project"
export DNA_GIT_REMOTE_URL="https://github.com/norlab-ulaval/dockerized-norlab-project.git"

# ....Mock dependencies loading test functions.....................................................
function dna::import_lib_and_dependencies() {
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

# ....Load DNA lib functions.......................................................................
source "${DNA_LIB_PATH:?err}/core/utils/ui.bash" || exit 1

# ....Export loaded functions......................................................................
for func in $(compgen -A function | grep -e dna:: -e nbs:: -e n2st::); do
  # shellcheck disable=SC2163
  export -f "${func}"
done

# ....Teardown.....................................................................................
# Print a message to indicate that the mock import_dna_lib.bash has been loaded
echo "[DNA done] Mock import_dna_lib.bash and its librairies loaded"
EOF
}

setup() {

  # Create the src/bin directory in the mock project
  mkdir -p "${MOCK_DNA_DIR}/src/bin"
  mkdir -p "${MOCK_DNA_DIR}/lib/core/utils/"

  # Copy the dna script to the src/bin directory in the mock project
  cp "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" "${MOCK_DNA_DIR}/src/bin/"
  cp "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/ui.bash" "${MOCK_DNA_DIR}/src/lib/core/utils/"
  cp "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/.env.cli_format_and_style" "${MOCK_DNA_DIR}/src/lib/core/utils/"

  # Make the dna script executable
  chmod +x "${MOCK_DNA_DIR}/src/bin/dna"

  # Change cwd to the mock super project directory
  cd "${MOCK_PROJECT_PATH}" || exit 1
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

@test "dna with no arguments › expect help message and no failure" {

  # Test case: When dna is called without arguments, it should show help and exit
  run bash "${MOCK_DNA_DIR}"/src/bin/dna

  # Should succeed
  assert_success

  # Should output help message
  assert_output --partial "Usage:"
  assert_output --partial "Commands:"
}

@test "dna init command › expect init function to be called" {
  # Test case: When dna is called with 'init' command, it should call the init function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna init

  # Should succeed
  assert_success

  # Should call the init function
  assert_output --partial "Mock dna::init_command called with args:"
}

@test "dna build command › expect build function to be called" {
  # Test case: When dna is called with 'build' command, it should call the build function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna build

  # Should succeed
  assert_success

  # Should call the build function
  assert_output --partial "Mock dna::build_command called with args:"
}

@test "dna up command › expect up function to be called" {
  # Test case: When dna is called with 'up' command, it should call the up function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna up

  # Should succeed
  assert_success

  # Should call the up function
  assert_output --partial "Mock dna::up_command called with args:"
}

@test "dna down command › expect down function to be called" {
  # Test case: When dna is called with 'down' command, it should call the down function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna down

  # Should succeed
  assert_success

  # Should call the down function
  assert_output --partial "Mock dna::down_command called with args:"
}

@test "dna run command › expect run function to be called" {
  # Test case: When dna is called with 'run' command, it should call the run function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna run

  # Should succeed
  assert_success

  # Should call the run function
  assert_output --partial "Mock dna::run_command called with args:"
}

@test "dna config command › expect config function to be called" {
  # Test case: When dna is called with 'config' command, it should call the config function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna config

  # Should succeed
  assert_success

  # Should call the config function
  assert_output --partial "Mock dna::config_command called with args:"
}

@test "dna version command › expect version function to be called" {
  # Test case: When dna is called with 'version' command, it should call the version function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna version

  # Should succeed
  assert_success

  # Should call the version function
  assert_output --partial "Mock dna::version_command called with args:"
}

@test "dna help command › expect help message and success" {
  # Test case: When dna is called with 'help' command, it should show help and exit successfully
  run bash "${MOCK_DNA_DIR}"/src/bin/dna help

  # Should succeed with exit code 0
  assert_success

  # Should output help message
  assert_output --partial "Usage:"
  assert_output --partial "Commands:"
}

@test "dna with unknown command › expect error message and failure" {
  # Test case: When dna is called with an unknown command, it should show error and help, and exit with error
  run bash "${MOCK_DNA_DIR}"/src/bin/dna unknown_command

  # Should fail with exit code 1
  assert_failure 1

  # Should output error message
  assert_output --partial "Unknown command dna unknown_command"
}

@test "dna project validate command › expect project_validate function to be called" {
  # Test case: When dna is called with 'project validate' command, it should call the project_validate function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna project validate

  # Should succeed
  assert_success

  # Should call the project_validate function
  assert_output --partial "Mock dna::project_validate_command called with args:"
}

@test "dna project sanity command › expect project_sanity function to be called" {
  # Test case: When dna is called with 'project sanity' command, it should call the project_sanity function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna project sanity

  # Should succeed
  assert_success

  # Should call the project_sanity function
  assert_output --partial "Mock dna::project_sanity_command called with args:"
}

@test "dna project dotenv command › expect project_dotenv function to be called" {
  # Test case: When dna is called with 'project dotenv' command, it should call the project_dotenv function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna project dotenv

  # Should succeed
  assert_success

  # Should call the project_dotenv function
  assert_output --partial "Mock dna::project_dotenv_command called with args:"
}

@test "dna project help command › expect help message and no failllure" {
  # Test case: When dna is called with 'project help' command, it should show help and exit with error
  run bash "${MOCK_DNA_DIR}"/src/bin/dna project help

  # Should succeed
  assert_success

  # Should output help message for project command
  assert_output --partial "Usage:"
  assert_output --partial "Commands:"
}

@test "dna project with unknown subcommand › expect error message and failure" {
  # Test case: When dna is called with an unknown project subcommand, it should show error and help, and exit with error
  run bash "${MOCK_DNA_DIR}"/src/bin/dna project unknown_subcommand

  # Should fail with exit code 1
  assert_failure 1

  # Should output error message
  assert_output --partial "Unknown command dna project unknown_subcommand"
}

@test "dna exec command › expect exec function to be called" {
  # Test case: When dna is called with 'exec' command, it should call the exec function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna exec

  # Should succeed
  assert_success

  # Should call the exec function
  assert_output --partial "Mock dna::exec_command called with args:"
}

@test "dna exec command with options › expect options passed to exec function" {
  # Test case: When dna is called with 'exec' command and options, it should pass the options to the exec function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna exec --service project-custom --detach

  # Should succeed
  assert_success

  # Should call the exec function with the options
  assert_output --partial "Mock dna::exec_command called with args: --service project-custom --detach"
}

@test "dna exec command with command › expect command passed to exec function" {
  # Test case: When dna is called with 'exec' command and a command, it should pass the command to the exec function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna exec -- bash -c 'echo hello'

  # Should succeed
  assert_success

  # Should call the exec function with the command
  assert_output --partial "Mock dna::exec_command called with args: -- bash -c echo hello"
}

@test "dna attach command › expect attach function to be called" {
  # Test case: When dna is called with 'attach' command, it should call the attach function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna attach

  # Should succeed
  assert_success

  # Should call the attach function
  assert_output --partial "Mock dna::attach_command called with args:"
}

@test "dna attach command with service option › expect service passed to attach function" {
  # Test case: When dna is called with 'attach' command and service option, it should pass the service to the attach function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna attach --service project-custom

  # Should succeed
  assert_success

  # Should call the attach function with the service
  assert_output --partial "Mock dna::attach_command called with args: --service project-custom"
}

@test "dna exec command with multiple options › expect all options passed to exec function" {
  # Test case: When dna is called with 'exec' command and multiple options, it should pass all options to the exec function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna exec --service project-custom --workdir /path --env VAR=value --no-TTY

  # Should succeed
  assert_success

  # Should call the exec function with all options
  assert_output --partial "Mock dna::exec_command called with args: --service project-custom --workdir /path --env VAR=value --no-TTY"
}

@test "dna exec command with short options › expect short options passed to exec function" {
  # Test case: When dna is called with 'exec' command and short options, it should pass them to the exec function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna exec -e VAR=value -w /path -T

  # Should succeed
  assert_success

  # Should call the exec function with the short options
  assert_output --partial "Mock dna::exec_command called with args: -e VAR=value -w /path -T"
}

@test "dna exec command with options and command › expect options and command passed to exec function" {
  # Test case: When dna is called with 'exec' command, options, and a command, it should pass all to the exec function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna exec --service project-custom --detach -- ls -la

  # Should succeed
  assert_success

  # Should call the exec function with the options and command
  assert_output --partial "Mock dna::exec_command called with args: --service project-custom --detach -- ls -la"
}

@test "dna exec command with --help option › expect help option passed to exec function" {
  # Test case: When dna is called with 'exec' command and --help option, it should pass it to the exec function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna exec --help

  # Should succeed
  assert_success

  # Should call the exec function with the help option
  assert_output --partial "Mock dna::exec_command called with args: --help"
}

@test "dna attach command with --help option › expect help option passed to attach function" {
  # Test case: When dna is called with 'attach' command and --help option, it should pass it to the attach function
  run bash "${MOCK_DNA_DIR}"/src/bin/dna attach --help

  # Should succeed
  assert_success

  # Should call the attach function with the help option
  assert_output --partial "Mock dna::attach_command called with args: --help"
}
