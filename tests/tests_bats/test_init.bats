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

TESTED_FILE="init.bash"
TESTED_FILE_PATH="src/lib/commands"

# ....Setup........................................................................................
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
  export MOCK_PROJECT_PATH="${BATS_DOCKER_WORKDIR}/utilities/tmp/dockerized-norlab-project-mock"

  # Create temporary directory for tests
  export MOCK_DNP_DIR=$(temp_make)

  # Create mock functions directory in the temporary directory
  mkdir -p "${MOCK_DNP_DIR}/src/lib/core/utils/"

  # Copy the real template files
  mkdir -p "${MOCK_DNP_DIR}/src/lib"
  cp -r "${BATS_DOCKER_WORKDIR}/src/lib/template" "${MOCK_DNP_DIR}/src/lib/"

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
export DNP_LIB_PATH="${MOCK_DNP_DIR}/src/lib"

export N2ST_PATH="utilities/norlab-shell-script-tools"
cd "${N2ST_PATH:?'Variable not set'}" || exit 1
source "import_norlab_shell_script_tools_lib.bash"

# ....Mock dependencies loading test functions.....................................................
function dnp::import_lib_and_dependencies() {
  return 0
}

function n2st::print_msg() {
  echo "Mock n2st::print_msg called with args: $*"
  return 0
}

function n2st::print_msg_warning() {
  echo "Mock n2st::print_msg_warning called with args: $*"
  return 0
}

function n2st::print_msg_error_and_exit() {
  echo "Mock n2st::print_msg_error_and_exit called with args: $*"
  exit 1
}


# ....Mock ui.bash functions.......................................................................
function dnp::command_help_menu() {
  echo "Mock dnp::command_help_menu called with args: $*"
  return 0
}

function dnp::unknown_subcommand_msg() {
  echo "Mock dnp::unknown_subcommand_msg called with args: $*"
  return 1
}

# ....Export mock functions........................................................................
for func in $(compgen -A function | grep -e dnp:: -e n2st::); do
  export -f "$func"
done

# ....Teardown.....................................................................................
# Print a message to indicate that the mock import_dnp_lib.bash has been loaded
echo "[DNP done] Mock import_dnp_lib.bash and its librairies loaded"
EOF

}

setup() {
  # Create necessary directories in the temporary directory
  mkdir -p "${MOCK_DNP_DIR}/src/lib/commands"

  # Copy the init.bash file to the temporary directory
  cp "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" "${MOCK_DNP_DIR}/src/lib/commands/"
  cp "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/super_project_dnp_sanity_check.bash" "${MOCK_DNP_DIR}/src/lib/core/utils/super_project_dnp_sanity_check.bash"
  cp "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/setup_host_for_running_this_super_project.bash" "${MOCK_DNP_DIR}/src/lib/core/utils/setup_host_for_running_this_super_project.bash"

  source "${MOCK_DNP_DIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1

  # Create a test git repository
  if [[ -z "${TEST_REPO_DIR}" ]]; then
    export TEST_REPO_DIR=$(temp_make)
  fi
  mkdir -p "${TEST_REPO_DIR}/.git"
  cd "${TEST_REPO_DIR}" || exit 1

  # Mock git commands
  function git() {
    if [[ "$1" == "remote" && "$2" == "get-url" && "$3" == "origin" ]]; then
      echo "https://github.com/user/test-project.git"
    else
      command git "$@"
    fi
  }
  export -f git
}

# ....Teardown.....................................................................................
teardown() {
  if [[ -n "${TEST_REPO_DIR}" && -d "${TEST_REPO_DIR}" ]]; then
    temp_del "${TEST_REPO_DIR}"
  fi
  bats_print_run_env_variable_on_error
  cd "${BATS_DOCKER_WORKDIR}" || exit 1
}

teardown_file() {
  # Clean up temporary directories
  temp_del "${MOCK_DNP_DIR}"
}

# ====Test cases==================================================================================

# Test cases for dnp::get_super_project_acronym
@test "dnp::get_super_project_acronym with onewordname › expect first three letters" {
  # Test case: When the project name is a single word, it should return the first three letters
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/init.bash && dnp::get_super_project_acronym 'onewordname'"

  # Should succeed
  assert_success

  # Should output the first three letters
  assert_output "one"
}

@test "dnp::get_super_project_acronym with multi-word-name › expect acronym 'mwn'" {
  # Test case: When the project name has multiple words separated by dashes, it should return the acronym
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/init.bash && dnp::get_super_project_acronym 'multi-word-name'"

  # Should succeed
  assert_success

  # Should output the acronym
  assert_output "mwn"
}

@test "dnp::get_super_project_acronym with multti_word_name › expect acronym 'mwn'" {
  # Test case: When the project name has multiple words separated by underscores, it should return the acronym
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/init.bash && dnp::get_super_project_acronym 'multti_word_name'"

  # Should succeed
  assert_success

  # Should output the acronym
  assert_output "mwn"
}

@test "dnp::get_super_project_acronym with multi-1word-2name-3with-number › expect acronym 'm123n'" {
  # Test case: When the project name has multiple words with numbers, it should return the acronym including numbers
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/init.bash && dnp::get_super_project_acronym 'multi-1word-2name-3with-number'"

  # Should succeed
  assert_success

  # Should output the acronym
  assert_output "m123n"
}

# Test cases for dnp::init_command
@test "dnp::init_command with --help › expect help menu" {
  # Test case: When init command is called with --help, it should show the help menu
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/init.bash && dnp::init_command --help"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dnp::command_help_menu called with args:"
}

@test "dnp::init_command with -h › expect help menu" {
  # Test case: When init command is called with -h, it should show the help menu
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/init.bash && dnp::init_command -h"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dnp::command_help_menu called with args:"
}

@test "dnp::init_command with unknown option › expect error" {
  # Test case: When init command is called with an unknown option, it should show an error
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/init.bash && dnp::init_command --unknown-option"

  # Should fail
  assert_failure

  # Should output the error message
  assert_output --partial "Mock dnp::unknown_subcommand_msg called with args: init"
}

@test "dnp::init_command without .git directory › expect error" {
  # Test case: When init command is called in a directory without .git, it should show an error
  # Remove the .git directory
  rm -rf "${TEST_REPO_DIR}/.git"

  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/init.bash && dnp::init_command"

  # Should fail
  assert_failure

  # Should output the error message
  assert_output --partial "Mock n2st::print_msg_error_and_exit called with args: Cwd is not at repository root"
}

@test "dnp::init_command with .dockerized_norlab_project already exists › Y -> expect update" {
  # Test case: When init command is called in a directory with .dockerized_norlab_project, it should show an error
  # Create the .dockerized_norlab_project directory
  mkdir -p "${TEST_REPO_DIR}/.dockerized_norlab_project"

  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/init.bash && yes | dnp::init_command"

  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_msg_warning called with args: This project is already DNP initialized"
  assert_output --partial "Mock n2st::print_msg called with args: DNP project initialized successfully."
}

@test "dnp::init_command with .dockerized_norlab_project already exists › N -> expect see you" {
  # Test case: When init command is called in a directory with .dockerized_norlab_project, it should show an error
  # Create the .dockerized_norlab_project directory
  mkdir -p "${TEST_REPO_DIR}/.dockerized_norlab_project"

  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/init.bash && echo 'N' | dnp::init_command"

  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_msg_warning called with args: This project is already DNP initialized since"
  assert_output --partial "Mock n2st::print_msg called with args: See you"
}

@test "dnp::init_command with valid repository › expect success" {
  # Test case: When init command is called in a valid repository, it should initialize the project
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/init.bash && dnp::init_command"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Initializing DNP project: test-project"
  assert_output --partial "Mock n2st::print_msg called with args: DNP project initialized successfully"
}

@test "dnp::init_command tests for PLACEHOLDER_* substitutions › expect all placeholders replaced" {
  # Test case: When init command is called, it should replace all placeholders
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/init.bash && dnp::init_command"

  # Should succeed
  assert_success

  # Check .env.dnp file for replaced placeholders
  assert_file_contains "${TEST_REPO_DIR}/.dockerized_norlab_project/configuration/.env.dnp" "https://github.com/user/test-project.git"
  assert_file_contains "${TEST_REPO_DIR}/.dockerized_norlab_project/configuration/.env.dnp" "IamDNP_tp"
  assert_file_contains "${TEST_REPO_DIR}/.dockerized_norlab_project/configuration/.env.dnp" "tp"

  # Check README.md file for replaced placeholders
  assert_file_contains "${TEST_REPO_DIR}/.dockerized_norlab_project/README.md" "IamDNP_tp"
  assert_file_contains "${TEST_REPO_DIR}/.dockerized_norlab_project/README.md" "test-project"

  # Verify placeholders are not present anymore
  assert_file_not_contains "${TEST_REPO_DIR}/.dockerized_norlab_project/configuration/.env.dnp" "PLACEHOLDER_DN_PROJECT_GIT_REMOTE_URL"
  assert_file_not_contains "${TEST_REPO_DIR}/.dockerized_norlab_project/configuration/.env.dnp" "PLACEHOLDER_DN_CONTAINER_NAME"
  assert_file_not_contains "${TEST_REPO_DIR}/.dockerized_norlab_project/configuration/.env.dnp" "PLACEHOLDER_DN_PROJECT_ALIAS_PREFIX"
  assert_file_not_contains "${TEST_REPO_DIR}/.dockerized_norlab_project/README.md" "PLACEHOLDER_DN_CONTAINER_NAME"
  assert_file_not_contains "${TEST_REPO_DIR}/.dockerized_norlab_project/README.md" "PLACEHOLDER_SUPER_PROJECT_NAME"
  assert_file_not_contains "${TEST_REPO_DIR}/.dockerized_norlab_project/README.md" "PLACEHOLDER_SUPER_PROJECT_USER"
}

@test "dnp::init_command tests for directory creation › expect required directories created" {
  # Test case: When init command is called, it should create required directories

  # Create a mock function for mkdir to track directory creation
  mkdir_calls=""
  function mkdir() {
    mkdir_calls="${mkdir_calls}${*}\n"
    command mkdir "$@"
  }
  export -f mkdir

  # Run the init command
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/init.bash && dnp::init_command"

  # Should succeed
  assert_success

  # Check if the required directories were created
  assert_dir_exist "${TEST_REPO_DIR}/artifact"
  assert_dir_exist "${TEST_REPO_DIR}/external_data"
  assert_dir_exist "${TEST_REPO_DIR}/src/launcher"
  assert_dir_exist "${TEST_REPO_DIR}/src/tools"
  assert_dir_exist "${TEST_REPO_DIR}/tests"

  assert_file_exist "${TEST_REPO_DIR}/src/launcher/configs/example_hparm_optim_exp_config.yaml"
  assert_file_exist "${TEST_REPO_DIR}/src/launcher/configs/hparam_optimization_base.yaml"
  assert_file_exist "${TEST_REPO_DIR}/src/launcher/configs/global_config.yaml"
  assert_file_exist "${TEST_REPO_DIR}/src/launcher/mock_app.py"
  assert_file_exist "${TEST_REPO_DIR}/src/launcher/mock_app_hparam_optim.py"
  assert_file_exist "${TEST_REPO_DIR}/src/tools/try_pytorch.py"
  assert_file_exist "${TEST_REPO_DIR}/src/README.md"

  assert_file_exist "${TEST_REPO_DIR}/tests/pytest.dnp.ini"
  assert_file_exist "${TEST_REPO_DIR}/tests/pytest.dnp_no_xdist.ini"
  assert_file_exist "${TEST_REPO_DIR}/tests/test_python_interpreter_has_ros.py"
  assert_file_exist "${TEST_REPO_DIR}/tests/test_try_pytorch.py"

  assert_file_exist "${TEST_REPO_DIR}/external_data/README.md"
  assert_file_exist "${TEST_REPO_DIR}/artifact/README.md"
  assert_file_exist "${TEST_REPO_DIR}/artifact/optuna_storage/README.md"
}

@test "dnp::init_command tests for README.md creation when it doesn't exist › expect README.md created" {
  # Test case: When init command is called and README.md doesn't exist, it should create it

  # Make sure README.md doesn't exist
  rm -f "${TEST_REPO_DIR}/README.md"

  # Run the init command
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/init.bash && dnp::init_command"

  # Should succeed
  assert_success

  # Verify README.md was created
  assert_file_exist "${TEST_REPO_DIR}/README.md"

  # Check README.md content
  run cat "${TEST_REPO_DIR}/README.md"
  assert_output --partial "# test-project"
  assert_output --partial "This project is initialized with [Dockerized-NorLab-Project]"
}

@test "dnp::init_command tests for README.md creation when it exists › expect README.md not modified" {
  # Test case: When init command is called and README.md exists, it should not modify it

  # Create a README.md file
  touch "${TEST_REPO_DIR}/README.md"

  # Run the init command
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/init.bash && dnp::init_command"

  # Should succeed
  assert_success

  # Verify README.md is not modified
  refute_output --partial "cat > \"README.md\" << EOF"
}

@test "dnp::init_command tests for .gitignore setup when it doesn't exist › expect .gitignore created from template" {
  # Test case: When init command is called and .gitignore doesn't exist, it should create it from template

  # Make sure .gitignore doesn't exist
  rm -f "${TEST_REPO_DIR}/.gitignore"

  # Create a mock template .gitignore file
  mkdir -p "${MOCK_DNP_DIR}/src/lib/template"
  echo "# Template .gitignore file" > "${MOCK_DNP_DIR}/src/lib/template/.gitignore"

  # Run the init command
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/init.bash && dnp::init_command"

  # Should succeed
  assert_success

  # Verify .gitignore was created
  assert_file_exist "${TEST_REPO_DIR}/.gitignore"
}

@test "dnp::init_command tests for .gitignore setup when it exists › expect .gitignore appended" {
  # Test case: When init command is called and .gitignore exists, it should append required entries

  # Create a .gitignore file with some content
  echo "# Existing .gitignore content" > "${TEST_REPO_DIR}/.gitignore"

  # Run the init command
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/init.bash && dnp::init_command"

  # Should succeed
  assert_success

  # Verify .gitignore exists
  assert_file_exist "${TEST_REPO_DIR}/.gitignore"

  # Check .gitignore content
  run cat "${TEST_REPO_DIR}/.gitignore"
  assert_output --partial "# Existing .gitignore content"
  assert_output --partial "# ====Dockerized-NorLab(required)=="
  assert_output --partial "**/.dockerized_norlab_project/dn_container_env_variable/"
  assert_output --partial "**/.dockerized_norlab_project/configuration/.env.local"
}

@test "dnp::init_command tests for .dockerignore setup when it doesn't exist › expect .dockerignore created from template" {
  # Test case: When init command is called and .dockerignore doesn't exist, it should create it from template

  # Make sure .dockerignore doesn't exist
  rm -f "${TEST_REPO_DIR}/.dockerignore"

  # Create a mock template .dockerignore file
  mkdir -p "${MOCK_DNP_DIR}/src/lib/template"
  echo "# Template .dockerignore file" > "${MOCK_DNP_DIR}/src/lib/template/.dockerignore"

  # Run the init command
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/init.bash && dnp::init_command"

  # Should succeed
  assert_success

  # Verify .dockerignore was created
  assert_file_exist "${TEST_REPO_DIR}/.dockerignore"
}

@test "dnp::init_command tests for .dockerignore setup when it exists › expect .dockerignore appended" {
  # Test case: When init command is called and .dockerignore exists, it should append required entries

  # Create a .dockerignore file with some content
  echo "# Existing .dockerignore content" > "${TEST_REPO_DIR}/.dockerignore"

  # Run the init command
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/init.bash && dnp::init_command"

  # Should succeed
  assert_success

  # Verify .dockerignore exists
  assert_file_exist "${TEST_REPO_DIR}/.dockerignore"

  # Check .dockerignore content
  run cat "${TEST_REPO_DIR}/.dockerignore"
  assert_output --partial "# Existing .dockerignore content"
  assert_output --partial "# ====Dockerized-NorLab(required)=="
  assert_output --partial "!**/.dockerized_norlab_project/"
  assert_output --partial "!**/version.txt"
  assert_output --partial "!**/.git"
}
