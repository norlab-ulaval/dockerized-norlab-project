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

  # This is the path to the mock super project (the user side) - same as test_setup_host_for_running_this_super_project.bats
  export MOCK_PROJECT_PATH="${BATS_DOCKER_WORKDIR}/utilities/tmp/dockerized-norlab-project-mock"

  # Create temporary directory for tests
  export MOCK_SAVE_DIR=$(temp_make)

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

  # Create mock tar files
  touch "${MOCK_DEPLOY_SAVE_DIR}/test-image-deploy.latest.tar"
  touch "${MOCK_DEVELOP_SAVE_DIR}/test-image-develop.latest.tar"
}

setup() {
  # Change cwd to the mock super project directory - same as test_setup_host_for_running_this_super_project.bats
  cd "${MOCK_PROJECT_PATH}" || exit 1

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
  temp_del "${MOCK_SAVE_DIR}"
}

# ====Test cases==================================================================================

@test "dnp::load_command with no arguments and no meta.txt in cwd › expect error" {
  # Test case: When load command is called without arguments and no meta.txt in current directory, it should show error
  # Create a test directory without .dockerized_norlab to ensure dnp::cd_to_dnp_super_project_root fails
  local test_dir="${MOCK_SAVE_DIR}/no_dnp_project"
  mkdir -p "${test_dir}"

  cd "${test_dir}"
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dnp_lib.bash"
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  run dnp::load_command

  # Should fail
  assert_failure

  # Should output error message about .dockerized_norlab directory not found
  assert_output --partial "directory not found"
}

@test "dnp::load_command with non-existent directory › expect error" {
  # Test case: When load command is called with non-existent directory, it should show error
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  run dnp::load_command /non/existent/dir

  # Should fail
  assert_failure

  # Should output error message
  assert_output --partial "Save directory does not exist"
}

@test "dnp::load_command with directory missing meta.txt › expect error" {
  # Test case: When load command is called with directory missing meta.txt, it should show error
  local test_dir="${MOCK_SAVE_DIR}/no_meta"
  mkdir -p "${test_dir}"

  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  run dnp::load_command "${test_dir}"

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

  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  run dnp::load_command "${test_dir}"

  # Should fail
  assert_failure

  # Should output error message
  assert_output --partial "not found"
}

@test "dnp::load_command with develop service › expect success" {
  # Test case: When load command is called with develop service, it should succeed
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  run dnp::load_command "${MOCK_DEVELOP_SAVE_DIR}"

  # Should succeed
  assert_success

  # Should output expected messages
  assert_output --partial "load develop image procedure"
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
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  run dnp::load_command "${MOCK_DEPLOY_SAVE_DIR}"

  # Should succeed
  assert_success

  # Should output expected messages
  assert_output --partial "load deploy image procedure"
  assert_output --partial "Loading saved image"
  assert_output --partial "Service: deploy"
  assert_output --partial "Image: test-image-deploy.latest"
  assert_output --partial "Repository: test-project"
  assert_output --partial "Loading Docker image from"
  assert_output --partial "Mock docker image load called"
  assert_output --partial "Load completed successfully"
  assert_output --partial "To change to the project directory, run:"
  assert_output --partial "You can then run 'dnp up deploy' or 'dnp run deploy' commands"
  refute_output --partial "Could not complete post-load actions for "
}

@test "dnp::load_command help flag › expect help output" {
  # Test case: When load command is called with help flag, it should show help
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  run dnp::load_command --help

  # Should succeed (exit 0 from help)
  assert_success

  # Should output help message
  assert_output --partial "Load Docker image from file for offline use"
}

@test "dnp::handle_deploy_post_load function › expect directory change instructions" {
  # Test case: Test the deploy post-load function directly
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  run dnp::handle_deploy_post_load "${MOCK_DEPLOY_SAVE_DIR}" test-project

  # Should succeed
  assert_success

  # Should output expected messages
  assert_output --partial "Load completed successfully"
  assert_output --partial "To change to the project directory, run:"
  assert_output --partial "You can then run 'dnp up deploy' or 'dnp run deploy' commands"
}

@test "dnp::handle_develop_post_load function with existing alias › expect alias execution" {
  # Test case: Test the develop post-load function with existing alias
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  run dnp::handle_develop_post_load test

  # Should succeed
  assert_success

  # Should output expected messages
  assert_output --partial "Check aliases: dnp-test-cd"
  assert_output --partial "Mock alias dnp-test-cd executed"
}

@test "dnp::handle_develop_post_load function with non-existing alias › expect warning" {
  # Test case: Test the develop post-load function with non-existing alias
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  run dnp::handle_develop_post_load nonexistent

  # Should succeed (warnings don't fail)
  assert_success

  # Should output warning message
  assert_output --partial "Alias not found: dnp-nonexistent-cd"
  assert_output --partial "Please manually navigate"
}

@test "dnp::handle_develop_post_load function with empty alias › expect warning" {
  # Test case: Test the develop post-load function with empty alias
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  run dnp::handle_develop_post_load ''

  # Should succeed (warnings don't fail)
  assert_success

  # Should output warning message
  assert_output --partial "DN_PROJECT_ALIAS_PREFIX not found"
  assert_output --partial "Please manually navigate"
}

@test "dnp::load_command with no arguments but meta.txt in current directory › expect success" {
  # Test case: When load command is called without arguments but meta.txt exists in current directory
  local test_dir="${MOCK_SAVE_DIR}/current_dir_test"
  mkdir -p "${test_dir}/test-project"

  # Create meta.txt in test directory
  cat > "${test_dir}/meta.txt" << 'EOF'
# DNP Save Metadata
SERVICE=deploy
IMAGE_NAME=test-image-deploy.latest
SUPER_PROJECT_REPO_NAME=test-project
DN_PROJECT_ALIAS_PREFIX=test
TAR_FILENAME=test-image-deploy.latest.tar
EOF

  # Create mock tar file
  touch "${test_dir}/test-image-deploy.latest.tar"

  # Change to test directory and run load command without arguments
  # Note: This test doesn't need load_super_project_config.bash since it tests the case where meta.txt is in current directory
  cd "${test_dir}"
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dnp_lib.bash"
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  run dnp::load_command

  # Should succeed
  assert_success

  # Should output expected messages
  assert_output --partial "Using current directory as save directory"
  assert_output --partial "load deploy image procedure"
  assert_output --partial "Loading saved image"
}

# Note: The test case for "dnp::load_command with no arguments and meta.txt in super project root"
# is covered by the integration test test_save_load_pipeline.bash as it requires complex
# DNP configuration setup that is better suited for integration testing rather than unit testing.
