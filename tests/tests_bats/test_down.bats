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

TESTED_FILE="down.bash"
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

  cat > "${MOCK_DNA_DIR}/src/lib/core/execute/down.bash" << 'EOF'
#!/bin/bash
# Mock down.bash
function dna::down_command() {
  echo "Mock dna::down_command called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNA_DIR}/src/lib/core/execute/down.slurm.bash" << 'EOF'
#!/bin/bash
# Mock down.slurm.bash
function dna::down_slurm() {
  echo "Mock dna::down_slurm called with args: $*"
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

function n2st::print_msg() {
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

function n2st::print_msg_done() {
  echo "Mock n2st::print_msg_done called with args: $*"
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

  # Copy the down.bash file to the temporary directory
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

@test "dna::down_command with no arguments › expect default behavior" {
  # Test case: When down command is called without arguments, it should stop containers with default options
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/down.bash && dna::down_command"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_formated_script_header called with args: down procedure"
  assert_output --partial "Mock dna::down_command called with args:"
  assert_output --partial "Mock n2st::print_formated_script_footer called with args: down procedure"
}

@test "dna::down_command with --help › expect help menu" {
  # Test case: When down command is called with --help, it should show the help menu
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/down.bash && dna::down_command --help"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}

@test "dna::down_command with -h › expect help menu" {
  # Test case: When down command is called with -h, it should show the help menu
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/down.bash && dna::down_command -h"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}

@test "dna::down_command with --slurm › expect slurm down" {
  # Test case: When down command is called with --slurm, it should stop slurm containers
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/down.bash && dna::down_command --slurm"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_formated_script_header called with args: down procedure"
  assert_output --partial "Mock dna::down_slurm called with args:"
  assert_output --partial "Mock n2st::print_msg_done called with args: slurm container down"
  assert_output --partial "Mock n2st::print_formated_script_footer called with args: down procedure"
}

@test "dna::down_command with docker-compose flags › expect flags passed to down_command" {
  # Test case: When down command is called with docker-compose flags, it should pass them to down_command
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/down.bash && dna::down_command --volumes --remove-orphans"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dna::down_command called with args: --volumes --remove-orphans"
}

@test "dna::down_command with --slurm and docker-compose flags › expect flags passed to down_slurm" {
  # Test case: When down command is called with --slurm and docker-compose flags, it should pass them to down_slurm
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/down.bash && dna::down_command --slurm --volumes --remove-orphans"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock dna::down_slurm called with args: --volumes --remove-orphans"
}
