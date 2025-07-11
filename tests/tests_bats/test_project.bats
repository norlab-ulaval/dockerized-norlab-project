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

TESTED_FILE="project.bash"
TESTED_FILE_PATH="src/lib/commands"

# ....Setup........................................................................................
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
  export MOCK_PROJECT_PATH="${BATS_DOCKER_WORKDIR}/utilities/tmp/dockerized-norlab-project-mock"

  # Create temporary directory for tests
  export MOCK_DNA_DIR=$(temp_make)

  # Create mock functions directory in the temporary directory
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/execute/"
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/utils/"

  # Create mock functions for dependencies
  cat > "${MOCK_DNA_DIR}/src/lib/core/utils/load_super_project_config.bash" << 'EOF'
#!/bin/bash
# Mock load_super_project_config.bash
echo "Mock load_super_project_config.bash loaded"
return 0
EOF

  cat > "${MOCK_DNA_DIR}/src/lib/core/execute/build.all.bash" << 'EOF'
#!/bin/bash
# Mock build.all.bash
function dna::build_services() {
  echo "Mock dna::build_services called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNA_DIR}/src/lib/core/execute/build.all.multiarch.bash" << 'EOF'
#!/bin/bash
# Mock build.all.multiarch.bash
function dna::build_services_multiarch() {
  echo "Mock dna::build_services_multiarch called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNA_DIR}/src/lib/core/execute/project_validate.all.bash" << 'EOF'
#!/bin/bash
# Mock project_validate.all.bash
function dna::project_validate_all() {
  echo "Mock dna::project_validate_all called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNA_DIR}/src/lib/core/execute/project_validate.slurm.bash" << 'EOF'
#!/bin/bash
# Mock project_validate.slurm.bash
function dna::project_validate_slurm() {
  echo "Mock dna::project_validate_slurm called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNA_DIR}/src/lib/core/utils/super_project_dna_sanity_check.bash" << 'EOF'
#!/bin/bash
# Mock super_project_dna_sanity_check.bash
function dna::super_project_dna_sanity_check() {
  echo "Mock dna::super_project_dna_sanity_check called with args: $*"
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
export MSG_LINE_CHAR_BUILDER_LVL2="-"

# Set up environment variables
export DNA_SPLASH_NAME_FULL="Dockerized-NorLab (DN)"
export DNA_SPLASH_NAME_SMALL="Dockerized-NorLab"
export DNA_ROOT="${MOCK_DNA_DIR}"
export DNA_LIB_PATH="${MOCK_DNA_DIR}/src/lib"
export DNA_LIB_EXEC_PATH="${MOCK_DNA_DIR}/src/lib/core/execute"

# ....Mock dependencies loading test functions.....................................................
function dna::import_lib_and_dependencies() {
  return 0
}

# ....Mock ui.bash functions.......................................................................
function dna::command_help_menu() {
  echo "Mock dna::command_help_menu called with args: $*"
  return 0
}

# ....Mock N2ST functions..........................................................................
function n2st::print_formated_script_header() {
  echo "Mock n2st::print_formated_script_header called with args: $*"
  return 0
}

function n2st::print_formated_script_footer() {
  echo "Mock n2st::print_formated_script_footer called with args: $*"
  return 0
}

function n2st::print_msg() {
  echo "Mock n2st::print_msg called with args: $*"
  return 0
}

function n2st::draw_horizontal_line_across_the_terminal_window() {
  echo "Mock n2st::draw_horizontal_line_across_the_terminal_window called with args: $*"
  return 0
}

# ....Export mock functions........................................................................
for func in $(compgen -A function | grep -e dna:: -e n2st::); do
  # shellcheck disable=SC2163
  export -f "${func}"
done

# ....Teardown.....................................................................................
# Print a message to indicate that the mock import_dna_lib.bash has been loaded
echo "[DNA done] Mock import_dna_lib.bash and its librairies loaded"
EOF
}

setup() {
  # Create necessary directories in the temporary directory
  mkdir -p "${MOCK_DNA_DIR}/src/lib/commands"
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/utils"
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/execute"

  # Copy the project.bash file to the temporary directory
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

@test "dna::project_validate_command with no arguments › expect default behavior" {
  # Test case: When project validate command is called without arguments, it should validate all configurations
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/project.bash && dna::project_validate_command"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_formated_script_header called with args: project validate procedure"
  assert_output --partial "Validating configuration..."
  assert_output --partial "Mock dna::project_validate_all called with args:"
  assert_output --partial "Mock n2st::print_formated_script_footer called with args: project validate procedure"
}

@test "dna::project_validate_command with --slurm › expect slurm validation" {
  # Test case: When project validate command is called with --slurm, it should validate slurm configuration
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/project.bash && dna::project_validate_command --slurm"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_formated_script_header called with args: project validate procedure"
  assert_output --partial "Validating slurm configuration..."
  assert_output --partial "Mock dna::project_validate_slurm called with args:"
  assert_output --partial "Mock n2st::print_formated_script_footer called with args: project validate procedure"
}

@test "dna::project_validate_command with --slurm and arguments › expect arguments passed to slurm validation" {
  # Test case: When project validate command is called with --slurm and arguments, it should pass them to slurm validation
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/project.bash && dna::project_validate_command --slurm /path/to/slurm/jobs"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_formated_script_header called with args: project validate procedure"
  assert_output --partial "Validating slurm configuration..."
  assert_output --partial "Mock dna::project_validate_slurm called with args: /path/to/slurm/jobs"
  assert_output --partial "Mock n2st::print_formated_script_footer called with args: project validate procedure"
}

@test "dna::project_validate_command with --help › expect help menu" {
  # Test case: When project validate command is called with --help, it should show the help menu
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/project.bash && dna::project_validate_command --help"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}

@test "dna::project_validate_command with -h › expect help menu" {
  # Test case: When project validate command is called with -h, it should show the help menu
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/project.bash && dna::project_validate_command -h"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}

@test "dna::project_validate_command with --include-multiarch › expect multiarch flag passed to validation" {
  # Test case: When project validate command is called with --include-multiarch, it should pass the flag to validation
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/project.bash && dna::project_validate_command --include-multiarch"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_formated_script_header called with args: project validate procedure"
  assert_output --partial "Validating configuration..."
  assert_output --partial "Mock dna::project_validate_all called with args: --include-multiarch"
  assert_output --partial "Mock n2st::print_formated_script_footer called with args: project validate procedure"
}

@test "dna::project_validate_command with --slurm and --include-multiarch › expect both flags passed to slurm validation" {
  # Test case: When project validate command is called with both --slurm and --include-multiarch, it should pass the multiarch flag to slurm validation
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/project.bash && dna::project_validate_command --slurm --include-multiarch"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_formated_script_header called with args: project validate procedure"
  assert_output --partial "Validating slurm configuration..."
  assert_output --partial "Mock dna::project_validate_slurm called with args: --include-multiarch"
  assert_output --partial "Mock n2st::print_formated_script_footer called with args: project validate procedure"
}

@test "dna::project_validate_command with --include-multiarch and arguments › expect multiarch flag and arguments passed to validation" {
  # Test case: When project validate command is called with --include-multiarch and additional arguments, it should pass both to validation
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/project.bash && dna::project_validate_command --include-multiarch /path/to/config"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_formated_script_header called with args: project validate procedure"
  assert_output --partial "Validating configuration..."
  assert_output --partial "Mock dna::project_validate_all called with args: --include-multiarch /path/to/config"
  assert_output --partial "Mock n2st::print_formated_script_footer called with args: project validate procedure"
}

@test "dna::project_validate_command with --slurm, --include-multiarch and arguments › expect all passed to slurm validation" {
  # Test case: When project validate command is called with --slurm, --include-multiarch and additional arguments, it should pass multiarch flag and arguments to slurm validation
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/project.bash && dna::project_validate_command --slurm --include-multiarch /path/to/slurm/jobs"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_formated_script_header called with args: project validate procedure"
  assert_output --partial "Validating slurm configuration..."
  assert_output --partial "Mock dna::project_validate_slurm called with args: --include-multiarch /path/to/slurm/jobs"
  assert_output --partial "Mock n2st::print_formated_script_footer called with args: project validate procedure"
}

@test "dna::project_sanity_command with no arguments › expect default behavior" {
  # Test case: When project sanity command is called without arguments, it should validate super project setup
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/project.bash && dna::project_sanity_command"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_formated_script_header called with args: project sanity procedure"
  assert_output --partial "Validating super project setup..."
  assert_output --partial "Mock dna::super_project_dna_sanity_check called with args:"
  assert_output --partial "Mock n2st::print_formated_script_footer called with args: project sanity procedure"
}

@test "dna::project_sanity_command with arguments › expect arguments passed to sanity check" {
  # Test case: When project sanity command is called with arguments, it should pass them to sanity check
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/project.bash && dna::project_sanity_command --some-arg value"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_formated_script_header called with args: project sanity procedure"
  assert_output --partial "Validating super project setup..."
  assert_output --partial "Mock dna::super_project_dna_sanity_check called with args: --some-arg value"
  assert_output --partial "Mock n2st::print_formated_script_footer called with args: project sanity procedure"
}

@test "dna::project_sanity_command with --help › expect help menu" {
  # Test case: When project sanity command is called with --help, it should show the help menu
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/project.bash && dna::project_sanity_command --help"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}

@test "dna::project_sanity_command with -h › expect help menu" {
  # Test case: When project sanity command is called with -h, it should show the help menu
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/project.bash && dna::project_sanity_command -h"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}

@test "dna::project_dotenv_command with no arguments › expect default behavior" {
  # Test case: When project dotenv command is called without arguments, it should show consolidated and interpolated dotenv config files
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/project.bash && dna::project_dotenv_command"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_formated_script_header called with args: project dotenv procedure"
  assert_output --partial "Mock n2st::print_msg called with args: Showing consolidated and interpolated dotenv config files..."
  assert_output --partial "Mock n2st::draw_horizontal_line_across_the_terminal_window called with args: ="
  assert_output --partial "Mock n2st::print_formated_script_footer called with args: project dotenv procedure"
}

@test "dna::project_dotenv_command with arguments › expect arguments ignored" {
  # Test case: When project dotenv command is called with arguments, it should ignore them and show dotenv config files
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/project.bash && dna::project_dotenv_command --some-arg value"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_formated_script_header called with args: project dotenv procedure"
  assert_output --partial "Mock n2st::print_msg called with args: Showing consolidated and interpolated dotenv config files..."
  assert_output --partial "Mock n2st::draw_horizontal_line_across_the_terminal_window called with args: ="
  assert_output --partial "Mock n2st::print_formated_script_footer called with args: project dotenv procedure"
}

@test "dna::project_dotenv_command with --help › expect help menu" {
  # Test case: When project dotenv command is called with --help, it should show the help menu
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/project.bash && dna::project_dotenv_command --help"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}

@test "dna::project_dotenv_command with -h › expect help menu" {
  # Test case: When project dotenv command is called with -h, it should show the help menu
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/project.bash && dna::project_dotenv_command -h"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}
