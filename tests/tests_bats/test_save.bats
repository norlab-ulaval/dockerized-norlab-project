#!/usr/bin/env bats
# =================================================================================================
# Unit tests for save.bash command
#
# Test cases:
# - Basic save command functionality
# - Argument validation
# - Service validation (develop/deploy)
# - Directory validation
# - Docker image save functionality
# - Metadata file creation
# - Project structure copying for deploy service
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

TESTED_FILE="save.bash"
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

  # Create mock project structure for deploy tests
  export MOCK_PROJECT_ROOT="${MOCK_DNP_DIR}/mock_project"
  mkdir -p "${MOCK_PROJECT_ROOT}/.dockerized_norlab/configuration"
  mkdir -p "${MOCK_PROJECT_ROOT}/.git"
  echo "mock config" > "${MOCK_PROJECT_ROOT}/.dockerized_norlab/configuration/.env"
  echo "mock git" > "${MOCK_PROJECT_ROOT}/.git/config"

  # Create mock load_super_project_config.bash (after MOCK_PROJECT_ROOT is defined)
  cat > "${MOCK_DNP_DIR}/src/lib/core/utils/load_super_project_config.bash" << EOF
#!/bin/bash
# Mock load_super_project_config.bash
export SUPER_PROJECT_REPO_NAME="test-project"
export DN_PROJECT_IMAGE_NAME="test-image"
export DN_PROJECT_HUB="norlabulaval"
export PROJECT_TAG="latest"
export SUPER_PROJECT_ROOT="${MOCK_PROJECT_ROOT}"
export DN_PROJECT_GIT_REMOTE_URL="https://github.com/test/test-project.git"
export DN_PROJECT_ALIAS_PREFIX="test"
export DNP_CONFIG_SCHEME_VERSION="1.0"
echo "Mock load_super_project_config.bash loaded"
return 0
EOF
}

setup() {
  # Create necessary directories in the temporary directory
  mkdir -p "${MOCK_DNP_DIR}/src/lib/commands"
  mkdir -p "${MOCK_DNP_DIR}/src/lib/core/utils"

  # Copy the save.bash file to the temporary directory
  cp "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" "${MOCK_DNP_DIR}/src/lib/commands/"

  source "${MOCK_DNP_DIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1

  # Change to the temporary directory
  cd "${MOCK_DNP_DIR}" || exit 1

  # Mock docker command
  function docker() {
    case "$1" in
      "image")
        case "$2" in
          "save")
            echo "Mock docker image save called with args: $*"
            # Create a mock tar file
            touch "$4"  # $4 should be the output file path
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

  # Mock git commands
  function git() {
    case "$1" in
      "branch")
        echo "main"
        return 0
        ;;
      "rev-parse")
        echo "abc123def456"
        return 0
        ;;
      *)
        echo "Mock git command: $*"
        return 0
        ;;
    esac
  }
  export -f git

  # Mock date command for consistent timestamps
  function date() {
    if [[ "$1" == "+%Y%m%d%H%M" ]]; then
      echo "202312151430"
    else
      echo "Fri Dec 15 14:30:00 UTC 2023"
    fi
  }
  export -f date

  # Note: Using real cp and mkdir commands instead of mocking them
  # This allows us to test actual file/directory creation and copying
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

@test "dnp::save_command with no arguments › expect error" {
  # Test case: When save command is called without arguments, it should show error
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/save.bash && dnp::save_command"

  # Should fail
  assert_failure

  # Should output error message
  assert_output --partial "DIRPATH argument is required"
}

@test "dnp::save_command with only dirpath › expect error" {
  # Test case: When save command is called with only dirpath, it should show error
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/save.bash && dnp::save_command ${MOCK_SAVE_DIR}"

  # Should fail
  assert_failure

  # Should output error message
  assert_output --partial "SERVICE argument is required"
}

@test "dnp::save_command with invalid service › expect error" {
  # Test case: When save command is called with invalid service, it should show error
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/save.bash && dnp::save_command ${MOCK_SAVE_DIR} invalid"

  # Should fail
  assert_failure

  # Should output error message
  assert_output --partial "Invalid SERVICE: invalid"
}

@test "dnp::save_command with non-existent directory › expect error" {
  # Test case: When save command is called with non-existent directory, it should show error
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/save.bash && dnp::save_command /non/existent/dir develop"

  # Should fail
  assert_failure

  # Should output error message
  assert_output --partial "Directory does not exist"
}

@test "dnp::save_command with develop service › expect success" {
  # Test case: When save command is called with develop service, it should succeed
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/save.bash && dnp::save_command ${MOCK_SAVE_DIR} develop"

  # Should succeed
  assert_success

  # Should output expected messages
  assert_output --partial "Save develop image procedure"
  assert_output --partial "Creating save directory"
  assert_output --partial "Saving Docker image"
  assert_output --partial "Creating metadata file"
  assert_output --partial "Save completed successfully"
}

@test "dnp::save_command with deploy service › expect success with project copy" {
  # Test case: When save command is called with deploy service, it should succeed and copy project structure
  # Set SUPER_PROJECT_ROOT to our mock project
  export SUPER_PROJECT_ROOT="${MOCK_PROJECT_ROOT}"

  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/save.bash && dnp::save_command ${MOCK_SAVE_DIR} deploy"

  # Should succeed
  assert_success

  # Should output expected messages
  assert_output --partial "Save deploy image procedure"
  assert_output --partial "Creating save directory"
  assert_output --partial "Saving Docker image"
  assert_output --partial "Creating metadata file"
  assert_output --partial "Copying project structure for deploy service"
  assert_output --partial "Save completed successfully"
}

@test "dnp::save_command help flag › expect help output" {
  # Test case: When save command is called with help flag, it should show help
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/save.bash && dnp::save_command --help"

  # Should succeed (exit 0 from help)
  assert_success

  # Should output help message
  assert_output --partial "Mock dnp::command_help_menu"
}

@test "dnp::create_save_metadata function › expect metadata file creation" {
  # Test case: Test the metadata creation function directly
  local test_meta_file="${MOCK_SAVE_DIR}/test_meta.txt"

  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/save.bash && dnp::create_save_metadata ${test_meta_file} develop"

  # Should succeed
  assert_success

  # Check if metadata file was created (mocked)
  assert_file_exist "${test_meta_file}"
}

@test "dnp::copy_project_structure_for_deploy function › expect project structure copy" {
  # Test case: Test the project structure copy function directly
  # Set SUPER_PROJECT_ROOT to our mock project
  export SUPER_PROJECT_ROOT="${MOCK_PROJECT_ROOT}"
  export SUPER_PROJECT_REPO_NAME="test-project"

  local test_save_dir="${MOCK_SAVE_DIR}/test_deploy"
  mkdir -p "${test_save_dir}"

  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/save.bash && dnp::copy_project_structure_for_deploy ${test_save_dir}"

  # Should succeed
  assert_success

  # Test actual file and directory creation using bats file/dir assert functionalities
  local project_copy_path="${test_save_dir}/test-project"

  # Verify project directory was created
  assert_dir_exist "${project_copy_path}"

  # Verify .dockerized_norlab directory was copied
  assert_dir_exist "${project_copy_path}/.dockerized_norlab"
  assert_dir_exist "${project_copy_path}/.dockerized_norlab/configuration"
  assert_file_exist "${project_copy_path}/.dockerized_norlab/configuration/.env"

  # Verify .git directory was copied
  assert_dir_exist "${project_copy_path}/.git"
  assert_file_exist "${project_copy_path}/.git/config"

  # Verify artifact and external_data directories were created
  assert_dir_exist "${project_copy_path}/artifact"
  assert_dir_exist "${project_copy_path}/external_data"
}
