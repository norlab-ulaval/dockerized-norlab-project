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

TESTED_FILE="update.bash"
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
export DNA_RELEASE_CONFIG_SCHEME_VERSION="1"
export N2ST_VERSION="2.0.0"
export NBS_VERSION="3.0.0"
export IMAGE_ARCH_AND_OS="linux/amd64"

# ....Mock dependencies loading test functions.....................................................
function dna::import_lib_and_dependencies() {
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

# ....Load N2ST functions..........................................................................
source ${BATS_DOCKER_WORKDIR}/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash || exit 1

# ....Mock N2ST functions..........................................................................
function n2st::print_msg_error_and_exit() {
  echo "Mock n2st::print_msg_error_and_exit called with args: $*" >&2
  exit 1
}

function n2st::print_msg() {
  echo "Mock n2st::print_msg: $*"
  return 0
}


# ....Mock git commands............................................................................
function git() {
  case "$1" in
    "fetch")
      if [[ "$2" == "--tags" && "$3" == "origin" ]]; then
        echo "Mock git fetch --tags origin"
        return 0
      elif [[ "$2" == "--all" && "$3" == "--tags" && "$4" == "origin" ]]; then
        echo "Mock git fetch --tags origin"
        return 0
      fi
      ;;
    "tag")
      if [[ "$2" == "-l" ]]; then
        if [[ "$3" == "--merged" && "$4" == "origin/main" ]]; then
          echo "v1.1.0"
          echo "v1.2.0"
          return 0
        elif [[ "$3" == "--merged" && "$4" == "origin/beta" ]]; then
          echo "v1.0.0-beta.1"
          echo "v1.1.0-beta.5"
          return 0
        else
          echo "v1.1.0"
          echo "v1.0.0"
          echo "v1.2.0"
          echo "v1.0.0-beta.1"
          echo "v1.1.0-beta.5"
          return 0
        fi
      fi
      ;;
    "checkout")
      if [[ "$2" == "main" || "$2" == "beta" ]]; then
        echo "Mock git checkout $2"
        return 0
      fi
      ;;
    "pull")
      if [[ "$2" == "--recurse-submodules" && "$3" == "origin" ]]; then
        if [[ "$4" == "main" || "$4" == "beta" ]]; then
          echo "Mock git pull --recurse-submodules origin $4"
          return 0
        else
          echo "Mock git pull --recurse-submodules origin"
          return 0
        fi
      elif [[ "$2" == "origin" ]]; then
        echo "Mock git pull origin"
        return 0
      fi
      ;;
    *)
      command git "$@"
      ;;
  esac
}
export -f git

# ....Export mock functions........................................................................
for func in $(compgen -A function | grep -e dna:: -e n2st::); do
  # shellcheck disable=SC2163
  export -f "${func}"
done


# ....Teardown.....................................................................................
# Print a message to indicate that the mock import_dna_lib.bash has been loaded
echo "[dna done] Mock import_dna_lib.bash and its librairies loaded"
EOF
}

setup() {
  # Create necessary directories in the temporary directory
  mkdir -p "${MOCK_DNA_DIR}/src/lib/commands"
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/utils"

  # Copy the update.bash file to the temporary directory
  cp "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" "${MOCK_DNA_DIR}/src/lib/commands/"

  source "${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash" || exit 1

  # Change to the temporary directory
  cd "${MOCK_DNA_DIR}" || exit 1
  
  # Create mock .env file if needed
  echo "DNA_AUTO_UPDATE=false" > "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"
}

# ....Teardown.....................................................................................
teardown() {
  bats_print_run_env_variable_on_error
  # Clean up environment variables
  rm -f "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"
}

teardown_file() {
  # Clean up temporary directory
  temp_del "${MOCK_DNA_DIR}"
}

# ====Test cases===================================================================================

# ....Help menu....................................................................................

@test "dna::update_command with --help › expect help menu" {
  # Test case: When update command is called with --help, it should show the help menu
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/update.bash && dna::update_command --help"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}

@test "dna::update_command with -h › expect help menu" {
  # Test case: When update command is called with -h, it should show the help menu
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/update.bash && dna::update_command -h"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}

# ....Test helper functions........................................................................
# (NICE TO HAVE) ToDo: NMO-785 feat: consolidate update helper function to dedicated utility script

@test "dna::update_is_remote_newer function › expect correct version comparison" {
  # Test case: Test simplified version comparison function directly
  source "${MOCK_DNA_DIR}/src/lib/commands/update.bash"

  # Signature: dna::update_is_remote_newer "local-v" "remote-v"

  echo "Test equal versions - should return failure (not newer)" # >&3
  run dna::update_is_remote_newer '1.0.0' '1.0.0'
  assert_failure  # Function returns 1 (failure) when versions are equal

  echo "Test remote version newer - should return success" # >&3
  run dna::update_is_remote_newer '1.0.0' '1.1.0'
  assert_success  # Function returns 0 (success) when remote is newer

  echo "Test beta remote version equal - should return failure (not newer)" # >&3
  run dna::update_is_remote_newer '1.0.0-beta.21' '1.0.0-beta.21'
  assert_failure  # Function returns 1 (failure) when local is newer

  echo "Test beta remote version newer - should return success" # >&3
  run dna::update_is_remote_newer '1.0.0-beta.21' '1.0.0-beta.29'
  assert_success  # Function returns 0 (success) when remote is newer

  echo "Test local beta and main remote version newer - should return success" # >&3
  run dna::update_is_remote_newer '1.0.0-beta.21' '1.0.1'
  assert_success  # Function returns 0 (success) when remote is newer
}

@test "dna::update_get_auto_update_setting function › expect correct setting retrieval" {
  # Test case: Test auto-update setting retrieval

  # Case auto update set to true
  echo "DNA_AUTO_UPDATE=true" > "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/update.bash && dna::update_get_auto_update_setting"
  assert_success
  assert_output "true"

  # Case auto update set to false
  echo "DNA_AUTO_UPDATE=false" > "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/update.bash && dna::update_get_auto_update_setting"
  assert_success
  assert_output "false"

  # Case dotenv file does not exist
  rm -f "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/update.bash && dna::update_get_auto_update_setting"
  assert_success
  assert_output "false"
}

# ....Test dna::update_command.....................................................................

@test "dna::update_command with --toggle-auto › expect auto-update setting toggled from false to true" {
  # Test case: When update command is called with --toggle-auto, it should toggle auto-update from false to true
  echo "DNA_AUTO_UPDATE=false" > "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"

  source "${MOCK_DNA_DIR}/src/lib/commands/update.bash"
  run dna::update_command --toggle-auto
#  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/update.bash && dna::update_command --toggle-auto"

  # Should succeed
  assert_success

  # Should output current value and toggle confirmation
  assert_output --partial "Current DNA_AUTO_UPDATE: false"
  assert_output --partial "DNA_AUTO_UPDATE toggled to true"
  assert_file_contains "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local" "^DNA_AUTO_UPDATE=true"
}

@test "dna::update_command with --toggle-auto › expect auto-update setting toggled from true to false" {
  # Test case: When update command is called with --toggle-auto, it should toggle auto-update from true to false
  echo "DNA_AUTO_UPDATE=true" > "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"

  source "${MOCK_DNA_DIR}/src/lib/commands/update.bash"
  run dna::update_command --toggle-auto


  # Should succeed
  assert_success

  # Should output current value and toggle confirmation
  assert_output --partial "Current DNA_AUTO_UPDATE: true"
  assert_output --partial "DNA_AUTO_UPDATE toggled to false"
  assert_file_contains "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local" "DNA_AUTO_UPDATE=false"
}

@test "dna::update_command with --toggle-auto › expect auto-update setting toggled from unset to true" {
  # Test case: When update command is called with --toggle-auto and no setting exists, it should toggle to true
  rm -f "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"

  source "${MOCK_DNA_DIR}/src/lib/commands/update.bash"
  run dna::update_command --toggle-auto

  # Should succeed
  assert_success

  # Should output current value (defaults to false) and toggle confirmation
  assert_output --partial "Current DNA_AUTO_UPDATE: false"
  assert_output --partial "DNA_AUTO_UPDATE toggled to true"
  assert_file_contains "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local" "DNA_AUTO_UPDATE=true"
}

@test "dna::update_command with no arguments and no update needed › expect up to date message" {
  # Test case: When update command is called and DNA is up to date
  # Mock DNA_VERSION to match remote version
  export DNA_VERSION="1.2.0"

  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/update.bash && dna::update_command"

  # Should succeed
  assert_success

  # Should output up to date message
  assert_output --partial "Already up to date"
}

@test "dna::update_command with --status and no update needed › expect up to date message" {
  # Test case: When update command is called and DNA is up to date
  # Mock DNA_VERSION to match remote version
  export DNA_VERSION="1.2.0"

  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/update.bash && dna::update_command --status"

  # Should succeed
  assert_success

  # Should output up to date message
  assert_output --partial "Already up to date"
}

@test "dna::update_command with --status and update available › expect update available message" {
  # Test case: When update command is called and DNA is up to date
  # Mock DNA_VERSION to match remote version
  export DNA_VERSION="0.9.0"

  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/update.bash && dna::update_command --status"

  # Should succeed
  assert_success

  # Should output up to date message
  assert_output --partial "Update available: 0.9.0 → 1.2.0"
}

@test "dna::update_command with no arguments and update available, no auto-update, user declines › expect skip message" {
  # Test case: When update is available, auto-update is false, and user declines
  echo "DNA_AUTO_UPDATE=false" > "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"
  export DNA_VERSION="0.9.0"

  run timeout 10s bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/update.bash && echo 'N' | dna::update_command"

  # Should succeed
  assert_success

  # Should show update available and skip message
  assert_output --partial "Update available: 0.9.0 → 1.2.0"
  assert_output --partial "DNA update skipped"
}

@test "dna::update_command with no arguments and update available, no auto-update, user accepts › expect update performed" {
  # Test case: When update is available, auto-update is false, and user accepts
  echo "DNA_AUTO_UPDATE=false" > "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"
  export DNA_VERSION="0.9.0"

  run timeout 10s bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/update.bash && echo 'Y' | dna::update_command"

  # Should succeed
  assert_success

  # Should show update available and perform update
  assert_output --partial "Update available: 0.9.0 → 1.2.0"
  assert_output --partial "DNA successfully updated"
}

@test "dna::update_command with no arguments and update available, auto-update enabled › expect automatic update" {
  # Test case: When update is available and auto-update is enabled
  export DNA_VERSION="0.9.0"
  echo "DNA_AUTO_UPDATE=true" > "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"

  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/update.bash && dna::update_command"

  # Should succeed
  assert_success

  # Should show update available and auto-update message
  assert_output --partial "Update available: 0.9.0 → 1.2.0"
  assert_output --partial "Auto-update enabled, updating DNA"
  assert_output --partial "DNA successfully updated"
}

@test "dna::update_command with -y flag › expect forced update" {
  # Test case: When update command is called with -y flag, it should update without confirmation
  export DNA_VERSION="0.9.0"
  rm -f "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"

  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/update.bash && dna::update_command -y"

  # Should succeed
  assert_success

  # Should show update available and perform update
  assert_output --partial "Update available: 0.9.0 → 1.2.0"
  assert_output --partial "DNA successfully updated"
}

@test "dna::update_command with --yes flag › expect forced update" {
  # Test case: When update command is called with --yes flag, it should update without confirmation
  export DNA_VERSION="0.9.0"
  rm -f "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"

  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/update.bash && dna::update_command --yes"

  # Should succeed
  assert_success

  # Should show update available and perform update
  assert_output --partial "Update available: 0.9.0 → 1.2.0"
  assert_output --partial "DNA successfully updated"
}

@test "dna::update_command with local version newer than remote › expect up to date message" {
  # Test case: When local DNA version is newer than remote (simplified logic treats as up to date)
  export DNA_VERSION="2.0.0"

  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/update.bash && dna::update_command"

  # Should succeed
  assert_success

  # Should output up to date message (simplified logic no longer distinguishes local newer)
  assert_output --partial "Already up to date"
}

@test "dna::update_command with --include-prerelease flag › expect prerelease branch update" {
  # Test case: When update command is called with --include-prerelease flag, it should consider both branches
  export DNA_VERSION="0.9.0"
  rm -f "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"

  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/update.bash && dna::update_command --include-prerelease --yes"

  # Should succeed
  assert_success

  # Should show update available from main branch (since 1.2.0 > 1.1.0-beta.5) and perform update
  assert_output --partial "Update available: 0.9.0 → 1.2.0"
  assert_output --partial "Updating DNA repository to latest release from 'main' branch"
  assert_output --partial "DNA successfully updated to latest version from 'main' branch"
}

@test "dna::update_command with --include-prerelease and --status › expect prerelease branch status" {
  # Test case: When update command is called with --include-prerelease and --status, it should show status considering both branches
  export DNA_VERSION="1.0.0-beta.1"

  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/update.bash && dna::update_command --include-prerelease --status"

  # Should succeed
  assert_success

  # Should show update available from main branch (since 1.2.0 > 1.1.0-beta.5)
  assert_output --partial "Update available: 1.0.0-beta.1 → 1.2.0"
}

@test "dna::update_command with --toggle-auto --include-prerelease › expect prerelease auto-update setting toggled" {
  # Test case: When update command is called with --toggle-auto --include-prerelease, it should toggle prerelease setting
  echo "DNA_INCLUDE_PRERELEASE=false" > "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"

  source "${MOCK_DNA_DIR}/src/lib/commands/update.bash"
  run dna::update_command --toggle-auto --include-prerelease

  # Should succeed
  assert_success

  # Should output current value and toggle confirmation
  assert_output --partial "Current DNA_INCLUDE_PRERELEASE: false"
  assert_output --partial "DNA_INCLUDE_PRERELEASE toggled to true"
  assert_file_contains "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local" "DNA_INCLUDE_PRERELEASE=true"
}

@test "dna::update_command with DNA_INCLUDE_PRERELEASE=true › expect automatic prerelease update" {
  # Test case: When DNA_INCLUDE_PRERELEASE is true, should automatically consider both branches
  export DNA_VERSION="0.9.0"
  echo "DNA_INCLUDE_PRERELEASE=true" > "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"

  run timeout 10s bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/update.bash && dna::update_command"

  # Should succeed
  assert_success

  # Should show update available from main branch (auto selection) and perform update
  refute_output --partial "Mock n2st::print_msg: Would you like to update DNA now?"
  assert_output --partial "Update available: 0.9.0 → 1.2.0"
  assert_output --partial "DNA successfully updated to latest version from 'main' branch"
}

@test "dna::update_determine_latest_release_branch function › expect correct branch determination" {
  # Test case: Test branch determination function directly
  source "${MOCK_DNA_DIR}/src/lib/commands/update.bash"

  # Test should return "main" as it has version 1.2.0 which is newer than beta's 1.1.0-beta.5
  run dna::update_determine_latest_release_branch
  assert_success
  assert_output "main"
}

@test "dna::update_fetch_remote_latest_version with beta target › expect beta version" {
  # Test case: Test fetching latest version from beta branch
  source "${MOCK_DNA_DIR}/src/lib/commands/update.bash"

  run dna::update_fetch_remote_latest_version "beta"
  assert_success
  assert_output "1.1.0-beta.5"
}

@test "dna::update_fetch_remote_latest_version with main target › expect main version" {
  # Test case: Test fetching latest version from main branch
  source "${MOCK_DNA_DIR}/src/lib/commands/update.bash"

  run dna::update_fetch_remote_latest_version "main"
  assert_success
  assert_output "1.2.0"
}

@test "dna::update_fetch_remote_latest_version with auto target › expect automatic branch selection" {
  # Test case: Test automatic branch selection (should return main branch version)
  source "${MOCK_DNA_DIR}/src/lib/commands/update.bash"

  run dna::update_fetch_remote_latest_version "auto"
  assert_success
  assert_output "1.2.0"
}

@test "dna::update_command with unknown option › expect error" {
  # Test case: When update command is called with an unknown option, it should show an error
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/update.bash && dna::update_command --unknown-option"

  # Should fail
  assert_failure

  # Should output the unknown option message
  assert_output --partial "Mock dna::unknown_option_msg called with args: update --unknown-option"
}
