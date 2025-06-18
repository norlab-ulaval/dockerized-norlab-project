#!/usr/bin/env bats
# =================================================================================================
# Unit tests for load.bash command
#
# Test cases:
# - Basic load command functionality
# - Argument validation
# - Save directory validation
# - Metadata file validation
# - Docker image load functionality
# - Post-load actions for develop vs deploy services
# - Error handling
#
# =================================================================================================

bats_path=/usr/lib/bats
error_prefix="[\033[1;31mN2ST ERROR\033[0m]"
if [[ -d ${bats_path} ]]; then
  # ....Bats-core recommended helper functions.....................................................
  load "${bats_path}/bats-support/load"
  load "${bats_path}/bats-assert/load"
  load "${bats_path}/bats-file/load"
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

TESTED_FILE="load.bash"
TESTED_FILE_PATH="src/lib/commands"

# ....Setup........................................................................................
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR

  # Create temporary directory for tests
  export MOCK_DNP_DIR=$(temp_make)
  export MOCK_SAVE_DIR=$(temp_make)

  # Create mock functions directory in the temporary directory
  mkdir -p "${MOCK_DNP_DIR}/src/lib/core/utils/"
  mkdir -p "${MOCK_DNP_DIR}/src/lib/commands/"

  # Create a mock import_dnp_lib.bash that sets up the environment
  cat > "${MOCK_DNP_DIR}/src/lib/core/utils/import_dnp_lib.bash" << 'EOF'
#!/bin/bash
# Mock import_dnp_lib.bash

# Set message formatting variables
export MSG_DIMMED_FORMAT=""
export MSG_END_FORMAT=""
export MSG_LINE_CHAR_BUILDER_LVL1="-"

# Set up environment variables
export DNP_ROOT="${MOCK_DNP_DIR}"
export DNP_LIB_PATH="${MOCK_DNP_DIR}/src/lib"

# ....Mock dependencies loading test functions.....................................................
function dnp::import_lib_and_dependencies() {
  return 0
}

function n2st::print_msg() {
  echo "Mock n2st::print_msg: $*"
  return 0
}

function n2st::print_msg_error() {
  echo "Mock n2st::print_msg_error: $*"
  return 0
}

function n2st::print_msg_done() {
  echo "Mock n2st::print_msg_done: $*"
  return 0
}

function n2st::print_msg_warning() {
  echo "Mock n2st::print_msg_warning: $*"
  return 0
}

function n2st::print_formated_script_header() {
  echo "Mock n2st::print_formated_script_header: $*"
  return 0
}

function n2st::print_formated_script_footer() {
  echo "Mock n2st::print_formated_script_footer: $*"
  return 0
}

function dnp::command_help_menu() {
  echo "Mock dnp::command_help_menu called with args: $*"
  return 0
}

function dnp::illegal_command_msg() {
  echo "Mock dnp::illegal_command_msg called with args: $*"
  return 1
}

# ....Export mock functions........................................................................
for func in $(compgen -A function | grep -e dnp:: -e n2st::); do
  export -f "$func"
done

echo "[DNP done] Mock import_dnp_lib.bash and its libraries loaded"
EOF

  # Create mock save directories for testing
  export MOCK_DEPLOY_SAVE_DIR="${MOCK_SAVE_DIR}/dnp-save-deploy-test-project-202312151430"
  export MOCK_DEVELOP_SAVE_DIR="${MOCK_SAVE_DIR}/dnp-save-develop-test-project-202312151430"

  mkdir -p "${MOCK_DEPLOY_SAVE_DIR}"
  mkdir -p "${MOCK_DEVELOP_SAVE_DIR}"
  mkdir -p "${MOCK_DEPLOY_SAVE_DIR}/test-project"

  # Create mock metadata files
  cat > "${MOCK_DEPLOY_SAVE_DIR}/meta.txt" << 'EOF'
# DNP Save Metadata
#   Generated on: Fri Dec 15 14:30:00 UTC 2023
#   From host:
#     Name: MacBook-Pro-M3-Karen
#     Architecture and OS: darwin/arm64

# Configuration
DNP_CONFIG_SCHEME_VERSION=1.0
DN_PROJECT_GIT_REMOTE_URL=https://github.com/test/test-project.git
DN_PROJECT_ALIAS_PREFIX=test

# Project Information
SUPER_PROJECT_REPO_NAME=test-project
SERVICE=deploy
IMAGE_NAME=test-image-deploy.latest

# Git Information
BRANCH=main
COMMIT=abc123def456

# Save Information
SAVE_DATE=Fri Dec 15 14:30:00 UTC 2023
SAVE_TIMESTAMP=202312151430
TAR_FILENAME=test-image-deploy.latest.tar
EOF

  cat > "${MOCK_DEVELOP_SAVE_DIR}/meta.txt" << 'EOF'
# DNP Save Metadata
#   Generated on: Fri Dec 15 14:30:00 UTC 2023
#   From host:
#     Name: MacBook-Pro-M3-Karen
#     Architecture and OS: darwin/arm64

# Configuration
DNP_CONFIG_SCHEME_VERSION=1.0
DN_PROJECT_GIT_REMOTE_URL=https://github.com/test/test-project.git
DN_PROJECT_ALIAS_PREFIX=test

# Project Information
SUPER_PROJECT_REPO_NAME=test-project
SERVICE=develop
IMAGE_NAME=test-image-develop.latest

# Git Information
BRANCH=main
COMMIT=abc123def456

# Save Information
SAVE_DATE=Fri Dec 15 14:30:00 UTC 2023
SAVE_TIMESTAMP=202312151430
TAR_FILENAME=test-image-develop.latest.tar
EOF

#  # Create mock metadata files
#  cat > "${MOCK_DEPLOY_SAVE_DIR}/meta.txt" << 'EOF'
## DNP Save Metadata
#SERVICE=deploy
#IMAGE_NAME=test-image-deploy.latest
#SUPER_PROJECT_REPO_NAME=test-project
#DN_PROJECT_ALIAS_PREFIX=test
#EOF

#  cat > "${MOCK_DEVELOP_SAVE_DIR}/meta.txt" << 'EOF'
## DNP Save Metadata
#SERVICE=develop
#IMAGE_NAME=test-image-develop.latest
#SUPER_PROJECT_REPO_NAME=test-project
#DN_PROJECT_ALIAS_PREFIX=test
#EOF

  # Create mock tar files
  touch "${MOCK_DEPLOY_SAVE_DIR}/test-image-deploy.latest.tar"
  touch "${MOCK_DEVELOP_SAVE_DIR}/test-image-develop.latest.tar"
}

setup() {
  # Create necessary directories in the temporary directory
  mkdir -p "${MOCK_DNP_DIR}/src/lib/commands"
  mkdir -p "${MOCK_DNP_DIR}/src/lib/core/utils"

  # Copy the load.bash file to the temporary directory
  cp "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" "${MOCK_DNP_DIR}/src/lib/commands/"

  source "${MOCK_DNP_DIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1

  # Change to the temporary directory
  cd "${MOCK_DNP_DIR}" || exit 1

  # Mock docker command
  function docker() {
    case "$1" in
      "image")
        case "$2" in
          "load")
            echo "Mock docker image load called with args: $*"
            return 0
            ;;
          *)
            echo "Mock docker image command: $*"
            return 0
            ;;
        esac
        ;;
      *)
        echo "Mock docker command: $*"
        return 0
        ;;
    esac
  }
  export -f docker

  # Note: Using real find, grep, cut, cd, pwd, command, and basename commands instead of mocking them
  # This allows us to test actual command behavior and file operations

  # Create a mock alias for testing alias functionality
  function dnp-test-cd() {
    echo "Mock alias dnp-test-cd executed"
    return 0
  }
  export -f dnp-test-cd
}

# ....Teardown.....................................................................................
teardown() {
  bats_print_run_env_variable_on_error
}

teardown_file() {
  # Clean up temporary directories
  temp_del "${MOCK_DNP_DIR}"
  temp_del "${MOCK_SAVE_DIR}"
}

# ====Test cases==================================================================================

@test "dnp::load_command with no arguments › expect error" {
  # Test case: When load command is called without arguments, it should show error
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/load.bash && dnp::load_command"

  # Should fail
  assert_failure

  # Should output error message
  assert_output --partial "SAVE_DIR_PATH argument is required"
}

@test "dnp::load_command with non-existent directory › expect error" {
  # Test case: When load command is called with non-existent directory, it should show error
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/load.bash && dnp::load_command /non/existent/dir"

  # Should fail
  assert_failure

  # Should output error message
  assert_output --partial "Save directory does not exist"
}

@test "dnp::load_command with directory missing meta.txt › expect error" {
  # Test case: When load command is called with directory missing meta.txt, it should show error
  local test_dir="${MOCK_SAVE_DIR}/no_meta"
  mkdir -p "${test_dir}"

  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/load.bash && dnp::load_command ${test_dir}"

  # Should fail
  assert_failure

  # Should output error message
  assert_output --partial "meta.txt not found"
}

@test "dnp::load_command with directory missing tar file › expect error" {
  # Test case: When load command is called with directory missing tar file, it should show error
  local test_dir="${MOCK_SAVE_DIR}/no_tar"
  mkdir -p "${test_dir}"
#  echo "COMPOSE_SERVICE=project-develop" > "${test_dir}/meta.txt"
#  echo "IMAGE_NAME=test-image" >> "${test_dir}/meta.txt"
#  echo "SUPER_PROJECT_REPO_NAME=test-project" >> "${test_dir}/meta.txt"
#  echo "DN_PROJECT_ALIAS_PREFIX=test" >> "${test_dir}/meta.txt"
#  echo "TAR_FILENAME=test-image-develop.latest.tar" >> "${test_dir}/meta.txt"

  cat > "${test_dir}/meta.txt" << 'EOF'
# DNP Save Metadata
#   Generated on: Fri Dec 15 14:30:00 UTC 2023
#   From host:
#     Name: MacBook-Pro-M3-Karen
#     Architecture and OS: darwin/arm64

# Configuration
DNP_CONFIG_SCHEME_VERSION=1.0
DN_PROJECT_GIT_REMOTE_URL=https://github.com/test/test-project.git
DN_PROJECT_ALIAS_PREFIX=test

# Project Information
SUPER_PROJECT_REPO_NAME=test-project
SERVICE=develop
IMAGE_NAME=test-image-develop.latest

# Git Information
BRANCH=main
COMMIT=abc123def456

# Save Information
SAVE_DATE=Fri Dec 15 14:30:00 UTC 2023
SAVE_TIMESTAMP=202312151430
TAR_FILENAME=test-image-develop.latest.tar
EOF

  # Note: No tar file is created in this directory, so real find command will not find any *.tar files

  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/load.bash && dnp::load_command ${test_dir}"

  # Should fail
  assert_failure

  # Should output error message
  assert_output --partial "Mock n2st::print_msg_error: Docker image archive file .tar not found in ${test_dir}"
}

@test "dnp::load_command with develop service › expect success" {
  # Test case: When load command is called with develop service, it should succeed
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/load.bash && dnp::load_command ${MOCK_DEVELOP_SAVE_DIR}"

  # Should succeed
  assert_success

  # Should output expected messages
  assert_output --partial "Load develop image procedure"
  assert_output --partial "Loading saved image"
  assert_output --partial "Service: develop"
  assert_output --partial "Image: test-image-develop.latest"
  assert_output --partial "Repository: test-project"
  assert_output --partial "Loading Docker image from"
  assert_output --partial "Mock docker image load called"
  assert_output --partial "Load completed successfully"
  refute_output --partial "Could not complete post-load actions for "
}

@test "dnp::load_command with deploy service › expect success with directory change" {
  # Test case: When load command is called with deploy service, it should succeed and change directory
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/load.bash && dnp::load_command ${MOCK_DEPLOY_SAVE_DIR}"

  # Should succeed
  assert_success

  # Should output expected messages
  assert_output --partial "Load deploy image procedure"
  assert_output --partial "Loading saved image"
  assert_output --partial "Service: deploy"
  assert_output --partial "Image: test-image-deploy.latest"
  assert_output --partial "Repository: test-project"
  assert_output --partial "Loading Docker image from"
  assert_output --partial "Mock docker image load called"
  assert_output --partial "Changing to project directory"
  assert_output --partial "Load completed successfully"
  assert_output --partial "Changed to project directory"
  refute_output --partial "Could not complete post-load actions for "
}

@test "dnp::load_command help flag › expect help output" {
  # Test case: When load command is called with help flag, it should show help
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/load.bash && dnp::load_command --help"

  # Should succeed (exit 0 from help)
  assert_success

  # Should output help message
  assert_output --partial "Mock dnp::command_help_menu"
}

@test "dnp::handle_deploy_post_load function › expect directory change" {
  # Test case: Test the deploy post-load function directly
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/load.bash && dnp::handle_deploy_post_load ${MOCK_DEPLOY_SAVE_DIR} test-project"

  # Should succeed
  assert_success

  # Should output expected messages
  assert_output --partial "Changing to project directory"
  assert_output --partial "Changed to project directory"
}

@test "dnp::handle_develop_post_load function with existing alias › expect alias execution" {
  # Test case: Test the develop post-load function with existing alias
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/load.bash && dnp::handle_develop_post_load test"

  # Should succeed
  assert_success

  # Should output expected messages
  assert_output --partial "Executing alias: dnp-test-cd"
  assert_output --partial "Mock alias dnp-test-cd executed"
  assert_output --partial "Executed alias: dnp-test-cd"
}

@test "dnp::handle_develop_post_load function with non-existing alias › expect warning" {
  # Test case: Test the develop post-load function with non-existing alias
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/load.bash && dnp::handle_develop_post_load nonexistent"

  # Should succeed (warnings don't fail)
  assert_success

  # Should output warning message
  assert_output --partial "Alias not found: dnp-nonexistent-cd"
  assert_output --partial "Please manually navigate"
}

@test "dnp::handle_develop_post_load function with empty alias › expect warning" {
  # Test case: Test the develop post-load function with empty alias
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/load.bash && dnp::handle_develop_post_load ''"

  # Should succeed (warnings don't fail)
  assert_success

  # Should output warning message
  assert_output --partial "DN_PROJECT_ALIAS_PREFIX not found"
  assert_output --partial "Please manually navigate"
}
