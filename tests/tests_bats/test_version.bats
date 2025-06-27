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
  export MOCK_DNA_DIR=$(temp_make)

  # Create mock functions directory in the temporary directory
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/utils/"

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
export DNA_LIB_PATH="${MOCK_DNA_DIR}/src/lib"
export DNA_HUMAN_NAME="Dockerized-NorLab project application"
export DNA_VERSION="1.0.0"
export DNA_CONFIG_SCHEME_VERSION="1"
export N2ST_VERSION="2.0.0"
export NBS_VERSION="3.0.0"
export IMAGE_ARCH_AND_OS="linux/amd64"

# ....Mock dependencies loading test functions.....................................................
function dna::import_lib_and_dependencies() {
  return 0
}

function n2st::print_msg() {
  return 0
}

# ....Mock ui.bash functions.......................................................................
function dna::command_help_menu() {
  echo "Mock dna::command_help_menu called with args: $*"
  return 0
}

function dna::unknown_option_msg() {
  echo "Mock dna::unknown_option_msg called with args: $*"
  return 1
}

# ....Mock N2ST functions..........................................................................
function n2st::print_msg_error_and_exit() {
  echo "Mock n2st::print_msg_error_and_exit called with args: $*" >&2
  exit 1
}

function n2st::set_which_architecture_and_os() {
  # This function sets IMAGE_ARCH_AND_OS, which we already set above
  return 0
}

# ....Mock git commands............................................................................
function git() {
  case "$1" in
    "branch")
      if [[ "$2" == "--show-current" ]]; then
        echo "main"
      fi
      ;;
    "rev-parse")
      if [[ "$2" == "HEAD" ]]; then
        echo "abc123def456"
      fi
      ;;
    *)
      command git "$@"
      ;;
  esac
}

# ....Export mock functions........................................................................
for func in $(compgen -A function | grep -e dna:: -e n2st::); do
  # shellcheck disable=SC2163
  export -f "${func}"
done

# Export git mock function
export -f git

# ....Teardown.....................................................................................
# Print a message to indicate that the mock import_dna_lib.bash has been loaded
echo "[DNA done] Mock import_dna_lib.bash and its librairies loaded"
EOF
}

setup() {
  # Create necessary directories in the temporary directory
  mkdir -p "${MOCK_DNA_DIR}/src/lib/commands"
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/utils"

  # Copy the version.bash file to the temporary directory
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

@test "dna::version_command with no arguments › expect default version display" {
  # Test case: When version command is called without arguments, it should display the default version format
  # Expected behavior: Shows "DNA_HUMAN_NAME version DNA_VERSION"
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/version.bash && dna::version_command"

  # Should succeed
  assert_success

  # Should output the expected message in default format
  assert_output --partial "${DNA_HUMAN_NAME} version ${DNA_VERSION}"
}

@test "dna::version_command with --help › expect help menu" {
  # Test case: When version command is called with --help, it should show the help menu
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/version.bash && dna::version_command --help"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}

@test "dna::version_command with -h › expect help menu" {
  # Test case: When version command is called with -h, it should show the help menu
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/version.bash && dna::version_command -h"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}

@test "dna::version_command with --short › expect short version display" {
  # Test case: When version command is called with --short, it should display only the version number
  # Expected behavior: Shows only "DNA_VERSION"
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/version.bash && dna::version_command --short"

  # Should succeed
  assert_success

  # Should output only the version number
  assert_output "${DNA_VERSION}"
}

@test "dna::version_command with -s › expect short version display" {
  # Test case: When version command is called with -s, it should display only the version number
  # Expected behavior: Shows only "DNA_VERSION"
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/version.bash && dna::version_command -s"

  # Should succeed
  assert_success

  # Should output only the version number
  assert_output "${DNA_VERSION}"
}

@test "dna::version_command with --all › expect detailed version display" {
  # Test case: When version command is called with --all, it should display detailed version information
  # Expected behavior: Shows comprehensive version info including submodules, git info, and architecture
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/version.bash && dna::version_command --all"

  # Should succeed
  assert_success

  # Should output detailed version information
  assert_output --partial "${DNA_HUMAN_NAME}:"
  assert_output --partial "Version: ${DNA_VERSION}"
  assert_output --partial "Config scheme version: ${DNA_CONFIG_SCHEME_VERSION}"
  assert_output --partial "norlab-shell-script-tools: ${N2ST_VERSION}"
  assert_output --partial "norlab-build-system: ${NBS_VERSION}"
  assert_output --partial "Current branch: main"
  assert_output --partial "Current commit: abc123def456"
  assert_output --partial "Host architecture and OS: ${IMAGE_ARCH_AND_OS}"
}

@test "dna::version_command with -a › expect detailed version display" {
  # Test case: When version command is called with -a, it should display detailed version information
  # Expected behavior: Shows comprehensive version info including submodules, git info, and architecture
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/version.bash && dna::version_command -a"

  # Should succeed
  assert_success

  # Should output detailed version information
  assert_output --partial "${DNA_HUMAN_NAME}:"
  assert_output --partial "Version: ${DNA_VERSION}"
  assert_output --partial "Config scheme version: ${DNA_CONFIG_SCHEME_VERSION}"
  assert_output --partial "norlab-shell-script-tools: ${N2ST_VERSION}"
  assert_output --partial "norlab-build-system: ${NBS_VERSION}"
  assert_output --partial "Current branch: main"
  assert_output --partial "Current commit: abc123def456"
  assert_output --partial "Host architecture and OS: ${IMAGE_ARCH_AND_OS}"
}

@test "dna::version_command with unknown option › expect error" {
  # Test case: When version command is called with an unknown option, it should show an error
  # Expected behavior: Should call dna::unknown_option_msg and return failure
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/version.bash && dna::version_command --unknown-option"

  # Should fail
  assert_failure

  # Should output the unknown option message
  assert_output --partial "Mock dna::unknown_option_msg called with args: version --unknown-option"
}
