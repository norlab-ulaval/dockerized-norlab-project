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
  apt-get update && \
      apt-get install --yes rsync

  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
  # Mock repo dockerized-norlab-project-mock-EMPTY is cloned in setup()

  # Create temporary directory for tests
  export MOCK_DNA_DIR=$(temp_make)

  # Create mock functions directory in the temporary directory
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/utils/"

  # Copy the real template files
  mkdir -p "${MOCK_DNA_DIR}/src/lib"
  cp -r "${BATS_DOCKER_WORKDIR}/src/lib/template" "${MOCK_DNA_DIR}/src/lib/"

  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/"
  cp -r "${BATS_DOCKER_WORKDIR}/src/lib/core/utils" "${MOCK_DNA_DIR}/src/lib/core/"
#  cp "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/super_project_dna_sanity_check.bash" "${MOCK_DNA_DIR}/src/lib/core/utils/super_project_dna_sanity_check.bash"
#  cp "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/setup_host_for_running_this_super_project.bash" "${MOCK_DNA_DIR}/src/lib/core/utils/setup_host_for_running_this_super_project.bash"
#  cp "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/load_super_project_config.bash" "${MOCK_DNA_DIR}/src/lib/core/utils/load_super_project_config.bash"


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
export DNA_GIT_REMOTE_URL="https://github.com/norlab-ulaval/dockerized-norlab-project"

export N2ST_PATH="${BATS_DOCKER_WORKDIR}/utilities/norlab-shell-script-tools"
source "${N2ST_PATH}/import_norlab_shell_script_tools_lib.bash"

# ....Mock dependencies loading test functions.....................................................
function dna::import_lib_and_dependencies() {
  return 0
}

function n2st::print_msg() {
  echo -e "Mock n2st::print_msg called with args: $*"
  return 0
}

function n2st::print_msg_done() {
  echo -e "Mock n2st::print_msg_done called with args: $*"
  return 0
}

function n2st::print_msg_warning() {
  echo -e "Mock n2st::print_msg_warning called with args: $*"
  return 0
}

function n2st::print_msg_error_and_exit() {
  echo -e "Mock n2st::print_msg_error_and_exit called with args: $*"
  exit 1
}


# ....Mock ui.bash functions.......................................................................
function dna::command_help_menu() {
  echo -e "Mock dna::command_help_menu called with args: $*"
  return 0
}

function dna::unknown_subcommand_msg() {
  echo -e "Mock dna::unknown_subcommand_msg called with args: $*"
  return 1
}

# ....Export mock functions........................................................................
for func in $(compgen -A function | grep -e dna:: -e n2st::); do
  # shellcheck disable=SC2163
  export -f "${func}"
done

# ....Teardown.....................................................................................
# Print a message to indicate that the mock import_dna_lib.bash has been loaded
echo -e "[DNA done] Mock import_dna_lib.bash and its librairies loaded"
EOF

  cat > "${MOCK_DNA_DIR}/src/lib/core/utils/load_super_project_config.bash" << 'EOF'
#!/bin/bash
# Mock load_super_project_config.bash
export SUPER_PROJECT_ROOT="$(pwd)"
export SUPER_PROJECT_REPO_NAME="dockerized-norlab-project-mock-EMPTY"
export DN_PROJECT_IMAGE_NAME="test-image"
export DN_PROJECT_HUB="norlabulaval"
export PROJECT_TAG="latest"
export DN_PROJECT_GIT_REMOTE_URL="https://github.com/norlab-ulaval/dockerized-norlab-project-mock-EMPTY.git"
export DN_PROJECT_ALIAS_PREFIX="test"
export DNA_CONFIG_SCHEME_VERSION="1.0"
echo "Mock load_super_project_config.bash loaded"
return 0
EOF
}

setup() {

  # Create necessary directories in the temporary directory
  mkdir -p "${MOCK_DNA_DIR}/src/lib/commands"

  # Copy the init.bash file to the temporary directory
  cp "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" "${MOCK_DNA_DIR}/src/lib/commands/"

  source "${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash" || exit 1

  # Create a test git repository
  if [[ -z "${TEST_TMP_REPO_ROOT}" ]]; then
    export TEST_TMP_REPO_ROOT=$(temp_make)
  fi
  cd "${TEST_TMP_REPO_ROOT}" || exit 1
  git clone https://github.com/norlab-ulaval/dockerized-norlab-project-mock-EMPTY.git
  export TEST_EMPTY_REPO="${TEST_TMP_REPO_ROOT}/dockerized-norlab-project-mock-EMPTY"
  cd "${TEST_EMPTY_REPO}" || exit 1

}

# ....Teardown.....................................................................................
teardown() {
  if [[ -n "${TEST_TMP_REPO_ROOT}" && -d "${TEST_TMP_REPO_ROOT}" ]]; then
    temp_del "${TEST_TMP_REPO_ROOT}"
  fi
  bats_print_run_env_variable_on_error
  cd "${BATS_DOCKER_WORKDIR}" || exit 1
}

teardown_file() {
  # Clean up temporary directories
  temp_del "${MOCK_DNA_DIR}"
}

# ====Test cases==================================================================================


# ....Test cases for dna::get_acronym (the underlying function)....................................
@test "dna::get_acronym with single word › expect first letter" {
  # Test case: When the input is a single word, it should return the first letter
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && dna::get_acronym 'singleword'"

  # Should succeed
  assert_success

  # Should output the first letter
  assert_output "s"
}

@test "dna::get_acronym with dash-separated words › expect acronym" {
  # Test case: When the input has words separated by dashes, it should return the acronym
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && dna::get_acronym 'multi-word-name'"

  # Should succeed
  assert_success

  # Should output the acronym
  assert_output "mwn"
}

@test "dna::get_acronym with underscore-separated words › expect acronym" {
  # Test case: When the input has words separated by underscores, it should return the acronym
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && dna::get_acronym 'multi_word_name'"

  # Should succeed
  assert_success

  # Should output the acronym
  assert_output "mwn"
}

@test "dna::get_acronym with mixed separators › expect acronym" {
  # Test case: When the input has words separated by both dashes and underscores, it should return the acronym
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && dna::get_acronym 'multi-word_mixed-name'"

  # Should succeed
  assert_success

  # Should output the acronym
  assert_output "mwmn"
}

@test "dna::get_acronym with numbers › expect acronym including numbers" {
  # Test case: When the input has words with numbers, it should return the acronym including numbers
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && dna::get_acronym 'multi-1word-2name-3with-number'"

  # Should succeed
  assert_success

  # Should output the acronym
  assert_output "m123n"
}

@test "dna::get_acronym with uppercase letters › expect lowercase acronym" {
  # Test case: When the input has uppercase letters, it should return lowercase acronym
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && dna::get_acronym 'Multi-Word-Name'"

  # Should succeed
  assert_success

  # Should output the lowercase acronym
  assert_output "mwn"
}

@test "dna::get_acronym with empty string › expect empty output" {
  # Test case: When the input is empty, it should return empty output
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && dna::get_acronym ''"

  # Should succeed
  assert_success

  # Should output empty string
  assert_output ""
}

# ....Test cases for dna::get_super_project_acronym................................................
@test "dna::get_super_project_acronym with onewordname › expect first three letters" {
  # Test case: When the project name is a single word, it should return the first three letters
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && dna::get_super_project_acronym 'onewordname'"

  # Should succeed
  assert_success

  # Should output the first three letters
  assert_output "one"
}

@test "dna::get_super_project_acronym with multi-word-name › expect acronym 'mwn'" {
  # Test case: When the project name has multiple words separated by dashes, it should return the acronym
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && dna::get_super_project_acronym 'multi-word-name'"

  # Should succeed
  assert_success

  # Should output the acronym
  assert_output "mwn"
}

@test "dna::get_super_project_acronym with multti_word_name › expect acronym 'mwn'" {
  # Test case: When the project name has multiple words separated by underscores, it should return the acronym
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && dna::get_super_project_acronym 'multti_word_name'"

  # Should succeed
  assert_success

  # Should output the acronym
  assert_output "mwn"
}

@test "dna::get_super_project_acronym with multi-1word-2name-3with-number › expect acronym 'm123n'" {
  # Test case: When the project name has multiple words with numbers, it should return the acronym including numbers
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && dna::get_super_project_acronym 'multi-1word-2name-3with-number'"

  # Should succeed
  assert_success

  # Should output the acronym
  assert_output "m123n"
}

# ....Test cases for dna::init_command.............................................................
@test "dna::init_command with --help › expect help menu" {
  # Test case: When init command is called with --help, it should show the help menu
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && dna::init_command --help"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}

@test "dna::init_command with -h › expect help menu" {
  # Test case: When init command is called with -h, it should show the help menu
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && dna::init_command -h"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}

@test "dna::init_command with unknown option › expect error" {
  # Test case: When init command is called with an unknown option, it should show an error
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && dna::init_command --unknown-option"

  # Should fail
  assert_failure

  # Should output the error message
  assert_output --partial "Mock dna::unknown_subcommand_msg called with args: init"
}

@test "dna::init_command without .git directory › expect error" {
  # Test case: When init command is called in a directory without .git, it should show an error
  # Remove the .git directory
  rm -rf "${TEST_EMPTY_REPO}/.git"

  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && dna::init_command"

  # Should fail
  assert_failure

  # Should output the error message
  assert_output --partial "Mock n2st::print_msg_error_and_exit called with args: Cwd is not at repository root"
}

@test "dna::init_command with .dockerized_norlab already exists › Y -> expect update" {
  # Test case: When init command is called in a directory with .dockerized_norlab, it should show an error
  # Create the .dockerized_norlab directory
  mkdir -p "${TEST_EMPTY_REPO}/.dockerized_norlab/configuration"
  touch "${TEST_EMPTY_REPO}/.dockerized_norlab/configuration/.env"

  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && echo 'Y' | dna::init_command"

  assert_success

  assert_dir_exist "${TEST_EMPTY_REPO}/.dockerized_norlab"
  assert_file_exist "${TEST_EMPTY_REPO}/.dockerized_norlab/configuration/.env"
  assert_file_exist "${TEST_EMPTY_REPO}/.dockerized_norlab/configuration/.env.old"

  # Should output the expected message
  assert_output --partial "Mock n2st::print_msg called with args: Preparing dockerized-norlab-project-mock-EMPTY DNA-initialization"
  assert_output --partial "Mock n2st::print_msg_warning called with args: This project is already DNA initialized since"
  assert_output --partial "Mock n2st::print_msg_done called with args: DNA project initialized successfully."

}

@test "dna::init_command with .dockerized_norlab already exists › N -> expect 'No problem, see you later'" {
  # Test case: When init command is called in a directory with .dockerized_norlab, it should show an error
  # Create the .dockerized_norlab directory
  mkdir -p "${TEST_EMPTY_REPO}/.dockerized_norlab"

  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && echo 'N' | dna::init_command"

  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_msg called with args: Preparing dockerized-norlab-project-mock-EMPTY DNA-initialization"
  assert_output --partial "Mock n2st::print_msg_warning called with args: This project is already DNA initialized since"
  assert_output --partial "Mock n2st::print_msg called with args: No problem, see you later"

}

@test "dna::init_command with valid repository › expect success" {

  assert_dir_not_exist "${TEST_EMPTY_REPO}/.dockerized_norlab"

  # Test case: When init command is called in a valid repository, it should initialize the project
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && echo 'Y' | dna::init_command"

  # Should succeed
  assert_success

  assert_dir_exist "${TEST_EMPTY_REPO}/.dockerized_norlab"

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Preparing dockerized-norlab-project-mock-EMPTY DNA-initialization"
  assert_output --partial "Mock n2st::print_msg called with args: Ready to proceed with DNA-initialization"
  assert_output --partial "Mock n2st::print_msg called with args: Initializing dockerized-norlab-project-mock-EMPTY"
  assert_output --partial "Mock n2st::print_msg_done called with args: DNA project initialized successfully."
}

@test "dna::init_command tests for PLACEHOLDER_* substitutions › expect all placeholders replaced" {
  assert_dir_not_exist "${TEST_EMPTY_REPO}/.dockerized_norlab"

  # Test case: When init command is called, it should replace all placeholders
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && echo 'Y' | dna::init_command"

  # Should succeed
  assert_success

  # Check .env.dna file for replaced placeholders
  assert_file_contains "${TEST_EMPTY_REPO}/.dockerized_norlab/configuration/.env.dna" "https://github.com/norlab-ulaval/dockerized-norlab-project-mock-EMPTY.git"
  assert_file_contains "${TEST_EMPTY_REPO}/.dockerized_norlab/configuration/.env.dna" "IamDNA_dnpme"
  assert_file_contains "${TEST_EMPTY_REPO}/.dockerized_norlab/configuration/.env.dna" "dnpme"

  # Check README.md file for replaced placeholders
  assert_file_contains "${TEST_EMPTY_REPO}/.dockerized_norlab/dn_container_env_variable/README.md" "IamDNA_dnpme"
  #assert_file_contains "${TEST_EMPTY_REPO}/.dockerized_norlab/README.md" "IamDNA_dnpme"
  #assert_file_contains "${TEST_EMPTY_REPO}/.dockerized_norlab/README.md" "dockerized-norlab-project-mock-EMPTY"

  # Verify placeholders are not present anymore
  assert_file_not_contains "${TEST_EMPTY_REPO}/.dockerized_norlab/configuration/.env.dna" "PLACEHOLDER_DN_PROJECT_GIT_REMOTE_URL"
  assert_file_not_contains "${TEST_EMPTY_REPO}/.dockerized_norlab/configuration/.env.dna" "PLACEHOLDER_DN_CONTAINER_NAME"
  assert_file_not_contains "${TEST_EMPTY_REPO}/.dockerized_norlab/configuration/.env.dna" "PLACEHOLDER_DN_PROJECT_ALIAS_PREFIX"
  assert_file_not_contains "${TEST_EMPTY_REPO}/.dockerized_norlab/dn_container_env_variable/README.md" "PLACEHOLDER_DN_CONTAINER_NAME"
  #assert_file_not_contains "${TEST_EMPTY_REPO}/.dockerized_norlab/README.md" "PLACEHOLDER_DN_CONTAINER_NAME"
  #assert_file_not_contains "${TEST_EMPTY_REPO}/.dockerized_norlab/README.md" "PLACEHOLDER_SUPER_PROJECT_NAME"
  #assert_file_not_contains "${TEST_EMPTY_REPO}/.dockerized_norlab/README.md" "PLACEHOLDER_SUPER_PROJECT_USER"
}

@test "dna::init_command tests for file/directory creation › expect required files/directories created" {
  # Test case: When init command is called, it should create required directories

  assert_dir_not_exist "${TEST_EMPTY_REPO}/.dockerized_norlab"

  # Run the init command
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && echo 'Y' | dna::init_command"

  # Should succeed
  assert_success

  # Check if the required directories were created
  assert_dir_exist "${TEST_EMPTY_REPO}/artifact"
  assert_dir_exist "${TEST_EMPTY_REPO}/external_data"
  assert_dir_exist "${TEST_EMPTY_REPO}/src/launcher"
  assert_dir_exist "${TEST_EMPTY_REPO}/src/dna_example"
  assert_dir_exist "${TEST_EMPTY_REPO}/tests"
  assert_dir_exist "${TEST_EMPTY_REPO}/slurm_jobs"

  assert_file_exist "${TEST_EMPTY_REPO}/src/launcher/configs/example_app_hparm_optim.yaml"
  assert_file_exist "${TEST_EMPTY_REPO}/src/launcher/configs/hparam_optimization_base.yaml"
  assert_file_exist "${TEST_EMPTY_REPO}/src/launcher/configs/global_config.yaml"
  assert_file_exist "${TEST_EMPTY_REPO}/src/launcher/example_app.py"
  assert_file_exist "${TEST_EMPTY_REPO}/src/launcher/example_app_hparm_optim.py"
  assert_file_exist "${TEST_EMPTY_REPO}/src/dna_example/try_pytorch.py"
  assert_file_exist "${TEST_EMPTY_REPO}/src/README.md"
  assert_file_exist "${TEST_EMPTY_REPO}/slurm_jobs/slurm_job.dryrun.bash"
  assert_file_exist "${TEST_EMPTY_REPO}/slurm_jobs/slurm_job.hydra_template.bash"
  assert_file_exist "${TEST_EMPTY_REPO}/slurm_jobs/slurm_job.template.bash"

  assert_file_exist "${TEST_EMPTY_REPO}/tests/pytest.ini"
  assert_file_exist "${TEST_EMPTY_REPO}/tests/pytest.no_xdist.ini"
  assert_file_exist "${TEST_EMPTY_REPO}/tests/test_dna_example/test_python_interpreter_has_ros.py"
  assert_file_exist "${TEST_EMPTY_REPO}/tests/test_dna_example/test_try_pytorch.py"

  assert_file_exist "${TEST_EMPTY_REPO}/external_data/README.md"
  assert_file_exist "${TEST_EMPTY_REPO}/artifact/README.md"
  assert_file_exist "${TEST_EMPTY_REPO}/artifact/optuna_storage/README.md"

}

@test "dna::init_command check validate and setup sub script are executed › expect succes" {
  # Test case: When init command exit, function execution from subscript should have succeed

  assert_dir_not_exist "${TEST_EMPTY_REPO}/.dockerized_norlab"

  # Run the init command
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && echo 'Y' | dna::init_command"

  # Should succeed
  assert_success

  # super_project_dna_sanity_check.bash -> dna::super_project_dna_sanity_check related
  assert_output --partial "Super project dockerized-norlab-project-mock-EMPTY setup is OK"

  # setup_host_for_running_this_super_project.bash -> dna::setup_host_for_this_super_project related
  assert_output --partial "Adding project aliases to .bashrc..."
  assert_output --partial "Setup completed!

    New available alias added to ~/.bashrc:
"
}

@test "dna::init_command tests for README.md creation when it doesn't exist › expect README.md created" {
  # Test case: When init command is called and README.md doesn't exist, it should create it

  # Make sure README.md doesn't exist
  rm -f "${TEST_EMPTY_REPO}/README.md"

  # Run the init command
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && echo 'Y' | dna::init_command"

  # Should succeed
  assert_success

  # Verify README.md was created
  assert_file_exist "${TEST_EMPTY_REPO}/README.md"

  # Check README.md content
  run cat "${TEST_EMPTY_REPO}/README.md"
  assert_output --partial "# dockerized-norlab-project-mock-EMPTY"
  assert_output --partial "This project is initialized with [Dockerized-NorLab project application (DNA)](https:"
}

@test "dna::init_command tests for README.md creation when it exists › expect README.md not modified" {
  # Test case: When init command is called and README.md exists, it should not modify it

  # Create a README.md file
  touch "${TEST_EMPTY_REPO}/README.md"

  # Run the init command
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && echo 'Y' | dna::init_command"

  # Should succeed
  assert_success

  # Verify README.md is not modified
  refute_output --partial "cat > \"README.md\" << EOF"
}

@test "dna::init_command tests for .gitignore setup when it doesn't exist › expect .gitignore created from template" {
  # Test case: When init command is called and .gitignore doesn't exist, it should create it from template

  # Make sure .gitignore doesn't exist
  rm -f "${TEST_EMPTY_REPO}/.gitignore"

  # Create a mock template .gitignore file
  mkdir -p "${MOCK_DNA_DIR}/src/lib/template"
  echo "# Template .gitignore file" > "${MOCK_DNA_DIR}/src/lib/template/.gitignore"

  # Run the init command
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && echo 'Y' | dna::init_command"

  # Should succeed
  assert_success

  # Verify .gitignore was created
  assert_file_exist "${TEST_EMPTY_REPO}/.gitignore"
}

@test "dna::init_command tests for .gitignore setup when it exists › expect .gitignore appended" {
  # Test case: When init command is called and .gitignore exists, it should append required entries

  # Create a .gitignore file with some content
  echo "# Existing .gitignore content" > "${TEST_EMPTY_REPO}/.gitignore"

  # Run the init command
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && echo 'Y' | dna::init_command"

  # Should succeed
  assert_success

  # Verify .gitignore exists
  assert_file_exist "${TEST_EMPTY_REPO}/.gitignore"

  # Check .gitignore content
  run cat "${TEST_EMPTY_REPO}/.gitignore"
  assert_output --partial "# Existing .gitignore content"
  assert_output --partial "# ====Dockerized-NorLab(required)=="
  assert_output --partial "/.dockerized_norlab/dn_container_env_variable/"
  assert_output --partial "/.dockerized_norlab/configuration/.env.local"
  assert_output --partial "!/.dockerized_norlab/dn_container_env_variable/README.md"
}

@test "dna::init_command tests for .dockerignore setup when it doesn't exist › expect .dockerignore created from template" {
  # Test case: When init command is called and .dockerignore doesn't exist, it should create it from template

  # Make sure .dockerignore doesn't exist
  rm -f "${TEST_EMPTY_REPO}/.dockerignore"

  # Create a mock template .dockerignore file
  mkdir -p "${MOCK_DNA_DIR}/src/lib/template"
  echo "# Template .dockerignore file" > "${MOCK_DNA_DIR}/src/lib/template/.dockerignore"

  # Run the init command
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && echo 'Y' | dna::init_command"

  # Should succeed
  assert_success

  # Verify .dockerignore was created
  assert_file_exist "${TEST_EMPTY_REPO}/.dockerignore"
}

@test "dna::init_command tests for .dockerignore setup when it exists › expect .dockerignore appended" {
  # Test case: When init command is called and .dockerignore exists, it should append required entries

  # Create a .dockerignore file with some content
  echo "# Existing .dockerignore content" > "${TEST_EMPTY_REPO}/.dockerignore"

  # Run the init command
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && echo 'Y' | dna::init_command"

  # Should succeed
  assert_success

  # Verify .dockerignore exists
  assert_file_exist "${TEST_EMPTY_REPO}/.dockerignore"

  # Check .dockerignore content
  run cat "${TEST_EMPTY_REPO}/.dockerignore"
  assert_output --partial "# Existing .dockerignore content"
  assert_output --partial "# ====Dockerized-NorLab(required)=="
  assert_output --partial "!**/.dockerized_norlab/"
  assert_output --partial "!**/version.txt"
  assert_output --partial "!**/.git"
}

@test "dna::init_command tests for backup functionality › expect .old backup files created when files exist" {
  # Test case: When init command is called and files already exist, it should create .old backup files
  # Note: Without --update flag, rsync will always backup existing files

  # Create existing files that will be backed up
  mkdir -p "${TEST_EMPTY_REPO}/artifact"
  echo "Original artifact README content" > "${TEST_EMPTY_REPO}/artifact/README.md"

  mkdir -p "${TEST_EMPTY_REPO}/external_data"
  echo "Original external_data README content" > "${TEST_EMPTY_REPO}/external_data/README.md"

  # Run the init command
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && echo 'Y' | dna::init_command"

  # Should succeed
  assert_success

  # Verify backup files were created (rsync always creates backups when files exist)
  assert_file_exist "${TEST_EMPTY_REPO}/artifact/README.md.old"
  assert_file_exist "${TEST_EMPTY_REPO}/external_data/README.md.old"

  # Verify backup files contain original content
  run cat "${TEST_EMPTY_REPO}/artifact/README.md.old"
  assert_output "Original artifact README content"

  run cat "${TEST_EMPTY_REPO}/external_data/README.md.old"
  assert_output "Original external_data README content"

  # Verify new files were created with template content
  assert_file_exist "${TEST_EMPTY_REPO}/artifact/README.md"
  assert_file_exist "${TEST_EMPTY_REPO}/external_data/README.md"
}

@test "dna::init_command tests .dockerized_norlab content › expect rsync commands work and content is setup" {
  # Test case: Verify that rsync commands work properly using only rsync (no cp fallback)
  # and use --recursive instead of -r for clarity

  assert_dir_not_exist "${TEST_EMPTY_REPO}/.dockerized_norlab"

  # Run the init command
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && echo 'Y' | dna::init_command"

  # Should succeed (rsync is now a required dependency)
  assert_success

  # Verify all expected files and directories were created
  assert_dir_exist "${TEST_EMPTY_REPO}/.dockerized_norlab"
  assert_file_exist "${TEST_EMPTY_REPO}/.dockerized_norlab/README.md"
  assert_file_exist "${TEST_EMPTY_REPO}/artifact/README.md"
  assert_file_exist "${TEST_EMPTY_REPO}/external_data/README.md"
  assert_file_exist "${TEST_EMPTY_REPO}/src/launcher/example_app.py"
  assert_file_exist "${TEST_EMPTY_REPO}/tests/pytest.ini"

  # Cleanup ok
  assert_dir_exist "${TEST_EMPTY_REPO}/.dockerized_norlab/dn_container_env_variable/"
  assert_file_exist "${TEST_EMPTY_REPO}/.dockerized_norlab/dn_container_env_variable/README.md"
  assert_file_not_exist "${TEST_EMPTY_REPO}/.dockerized_norlab/dn_container_env_variable/.env.dn_expose_PLACEHOLDER_DN_CONTAINER_NAME"
}

@test "dna::init_command tests for file ownership and permission validation › expect proper ownership and permissions" {
  # Test case: Verify that copied files have proper ownership and permissions matching the super project

  assert_dir_not_exist "${TEST_EMPTY_REPO}/.dockerized_norlab"

  # Run the init command
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/init.bash && echo 'Y' | dna::init_command"

  # Should succeed
  assert_success

  # Verify files were created
  assert_file_exist "${TEST_EMPTY_REPO}/artifact/README.md"
  assert_file_exist "${TEST_EMPTY_REPO}/external_data/README.md"
  assert_dir_exist "${TEST_EMPTY_REPO}/.dockerized_norlab"

  # Verify ownership
  assert_file_owner "$(whoami)" "${TEST_EMPTY_REPO}/artifact/README.md"
  assert_file_owner "$(whoami)" "${TEST_EMPTY_REPO}/external_data/README.md"
  assert_file_owner "$(whoami)" "${TEST_EMPTY_REPO}/.dockerized_norlab"

  # Verify permission
  echo "Permissions:
   artifact/README.md: $(stat -c '%a' "${TEST_EMPTY_REPO}/artifact/README.md")
   external_data/README.md: $(stat -c '%a' "${TEST_EMPTY_REPO}/external_data/README.md")
   .dockerized_norlab/: $(stat -c '%a' "${TEST_EMPTY_REPO}/.dockerized_norlab")"

  assert_file_permission 644 "${TEST_EMPTY_REPO}/artifact/README.md"
  assert_file_permission 644 "${TEST_EMPTY_REPO}/external_data/README.md"
  assert_file_permission 755 "${TEST_EMPTY_REPO}/.dockerized_norlab"

}
