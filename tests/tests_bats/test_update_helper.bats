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

TESTED_FILE="update_helper.bash"
TESTED_FILE_PATH="src/lib/core/utils"

# ....Setup........................................................................................
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
  export MOCK_PROJECT_PATH="${BATS_DOCKER_WORKDIR}/utilities/tmp/dockerized-norlab-project-mock"

  local current_scheme_version
  current_scheme_version=$(grep "DNA_RELEASE_CONFIG_SCHEME_VERSION=" "${BATS_DOCKER_WORKDIR}/.env.dockerized-norlab-project" | cut -d'=' -f2)

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
export DNA_ROOT="REPLACE_WITH_MOCK_DNA_DIR"
export DNA_LIB_PATH="REPLACE_WITH_MOCK_DNA_DIR/src/lib"
export DNA_HUMAN_NAME="Dockerized-NorLab project application"
export DNA_VERSION="1.0.0"
export DNA_RELEASE_CONFIG_SCHEME_VERSION=REPLACE_WITH_SCHEME_VERSION
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
  sed -i "s|REPLACE_WITH_MOCK_DNA_DIR|${MOCK_DNA_DIR}|g" "${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash"
  sed -i "s/REPLACE_WITH_SCHEME_VERSION/${current_scheme_version}/" "${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash"
}

setup() {
  # Create necessary directories in the temporary directory
  mkdir -p "${MOCK_DNA_DIR}/src/lib/commands"
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/utils"

  # Copy the update.bash file to the temporary directory
  cp "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" "${MOCK_DNA_DIR}/src/lib/core/utils/"

  source "${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash" || exit 1
  source "${MOCK_DNA_DIR}/src/lib/core/utils/update_helper.bash" || exit 1

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

# Note: most fct are indirectly tested via 'test_update.bats' and 'test_dna.bats' for now
# (NICE TO HAVE) ToDo: implement missing unit-tests (ref task NMO-785)

@test "dna::update_is_remote_newer function › expect correct version comparison" {
  # Test case: Test simplified version comparison function directly

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
  run dna::update_get_auto_update_setting
  assert_success
  assert_output "true"

  # Case auto update set to false
  echo "DNA_AUTO_UPDATE=false" > "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"
  run dna::update_get_auto_update_setting
  assert_success
  assert_output "false"

  # Case dotenv file does not exist
  rm -f "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"
  run dna::update_get_auto_update_setting
  assert_success
  assert_output "false"
}

@test "dna::update_get_auto_update_prerelease_setting function › expect correct setting retrieval" {
  # Test case: Test include pre-release setting retrieval

  # Case include pre-release set to true
  echo "DNA_INCLUDE_PRERELEASE=true" > "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"
  run dna::update_get_auto_update_prerelease_setting
  assert_success
  assert_output "true"

  # Case include pre-release set to false
  echo "DNA_INCLUDE_PRERELEASE=false" > "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"
  run dna::update_get_auto_update_prerelease_setting
  assert_success
  assert_output "false"

  # Case dotenv file does not exist
  rm -f "${MOCK_DNA_DIR}/.env.dockerized-norlab-project.local"
  run dna::update_get_auto_update_prerelease_setting
  assert_success
  assert_output "false"
}

@test "dna::should_run_daily_update › expect update or no-update status" {
  # Test case 1: File does not exist
  assert_file_not_exist "/tmp/.dna_last_update_check"

  run dna::should_run_daily_update

  # Should succeed
  assert_success

  # Test case 2: File does exist and date is older than today
  echo "0000-00-00" > "/tmp/.dna_last_update_check"
  run dna::should_run_daily_update

  # Should succeed
  assert_success


  # Test case 3: File does exist and date today
  date +%Y-%m-%d > "/tmp/.dna_last_update_check"
  run dna::should_run_daily_update

  # Should succeed
  assert_failure

  # Teardown
  #cat "/tmp/.dna_last_update_check" >&3
  rm -f "/tmp/.dna_last_update_check"
}

@test "dna::update_timestamp › expect /tmp/.dna_last_update_check to be updated" {
  # Test case 1: File does not exist
  assert_file_not_exist "/tmp/.dna_last_update_check"

  run dna::update_timestamp

  # Should succeed
  assert_success

  assert_file_exist "/tmp/.dna_last_update_check"
  assert_file_contains "/tmp/.dna_last_update_check" "$(date +%Y-%m-%d)"

  # Test case 2: File does exist and is overriden
  echo "0000-00-00" > "/tmp/.dna_last_update_check"
  run dna::update_timestamp

  # Should succeed
  assert_success

  assert_file_exist "/tmp/.dna_last_update_check"
  assert_file_contains "/tmp/.dna_last_update_check" "$(date +%Y-%m-%d)"

  # Teardown
  #cat "/tmp/.dna_last_update_check" >&3
  rm -f "/tmp/.dna_last_update_check"
}
