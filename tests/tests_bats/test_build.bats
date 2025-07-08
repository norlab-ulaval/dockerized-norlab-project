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

TESTED_FILE="build.bash"
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
  mkdir -p "${MOCK_DNA_DIR}/src/lib/commands/"


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

  cat > "${MOCK_DNA_DIR}/src/lib/core/execute/build.deploy.bash" << 'EOF'
#!/bin/bash
# Mock build.deploy.bash
function dna::build_project_deploy_service() {
  echo "Mock dna::build_project_deploy_service called with args: $*"
  return 0
}
EOF

  # Create mock save.bash file
  cat > "${MOCK_DNA_DIR}/src/lib/commands/save.bash" << 'EOF'
#!/bin/bash
function dna::save_command() {
  echo "Mock dna::save_command called with args: $*"
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
export MSG_LINE_CHAR_BUILDER_LVL1="-"

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
  echo "Mock n2st::print_msg: $*"
  return 0
}

# ....Mock ui.bash functions.......................................................................
function dna::command_help_menu() {
  echo "Mock dna::command_help_menu called with args: $*"
  return 0
}

function dna::illegal_command_msg() {
  echo "Mock dna::illegal_command_msg called with args: $*"
  return 1
}

function dna::unknown_subcommand_msg() {
  echo "Mock dna::unknown_subcommand_msg called with args: $*"
  return 1
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

function n2st::print_msg_error() {
  echo "Mock n2st::print_msg_error called with args: $*"
  return 0
}

function n2st::norlab_splash() {
  echo "Mock n2st::norlab_splash called with args: $*"
  return 0
}

# ....Mock DNA network functions...................................................................
function dna::is_online() {
  # Default to online unless MOCK_OFFLINE is set
  if [[ "${MOCK_OFFLINE:-false}" == "true" ]]; then
    return 1
  else
    return 0
  fi
}

# ....Mock Docker functions........................................................................
function docker() {
  # Mock docker command behavior
  case "$1" in
    "system")
      if [[ "$2" == "info" ]]; then
        echo "Registry: https://index.docker.io/v1/"
        return 0
      fi
      ;;
    "search")
      # Mock docker search - return success if MOCK_DOCKER_LOGIN is true
      if [[ "${MOCK_DOCKER_LOGIN:-false}" == "true" ]]; then
        echo "NAME DESCRIPTION STARS OFFICIAL AUTOMATED"
        echo "hello-world Hello World! (an example of minimal Dockerization) 1234 [OK]"
        return 0
      else
        return 1
      fi
      ;;
    *)
      echo "Mock docker command called with: $*"
      return 0
      ;;
  esac
}

# ....Mock additional N2ST functions...............................................................
function n2st::print_msg_done() {
  echo "Mock n2st::print_msg_done called with args: $*"
  return 0
}

# ....Export mock functions........................................................................
for func in $(compgen -A function | grep -e dna:: -e n2st:: -e docker -e command); do
  # shellcheck disable=SC2163
  export -f "${func}"
done

# ....Mock command utility.........................................................................
function command() {
  case "$1" in
    "-v")
      case "$2" in
        "docker")
          # Always return success for docker availability check
          return 0
          ;;
        *)
          # For other commands, use the real command
          builtin command "$@"
          ;;
      esac
      ;;
    *)
      # For other command usages, use the real command
      builtin command "$@"
      ;;
  esac
}

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

  # Copy the build.bash file to the temporary directory
  cp "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" "${MOCK_DNA_DIR}/src/lib/commands/"
#  cp "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/ui.bash" "${MOCK_DNA_DIR}/src/lib/core/utils/"

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

@test "dna::build_command with no arguments › expect default behavior" {
  # Test case: When build command is called without arguments, it should build all images with native architecture
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "all images native build"
  assert_output --partial "Mock dna::build_services called with args:"
}

@test "dna::build_command with --multiarch › expect multiarch build" {
  # Test case: When build command is called with --multiarch, it should build all images with multiarch
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command --multiarch"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "all images multiarch build"
#  assert_output --regexp "Mock dna::build_services_multiarch called with args:"
}

@test "dna::build_command with --online-build › expect force push flag" {
  # Test case: When build command is called with --online-build, it should pass the flag to build_services
  run bash -c "export MOCK_DOCKER_LOGIN=true && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command --online-build"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "all images native build"
  assert_output --regexp "Mock dna::build_services called with args:".*"--force-push-project-core"
}

@test "dna::build_command with develop service › expect develop images only" {
  # Test case: When build command is called with develop service, it should build develop images only
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command develop"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "develop images native build"
  assert_output --regexp "Mock dna::build_services called with args:".*"--service-names project-core,project-develop"
}

@test "dna::build_command with ci-tests service › expect CI tests images only" {
  # Test case: When build command is called with ci-tests service, it should build CI tests images only
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command ci-tests"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "CI tests images native build"
  assert_output --regexp "Mock dna::build_services called with args:".*"--service-names project-core,project-ci-tests,project-ci-tests-no-gpu"
}

@test "dna::build_command with slurm service › expect slurm images only" {
  # Test case: When build command is called with slurm service, it should build slurm images only
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command slurm"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "slurm images native build"
  assert_output --regexp "Mock dna::build_services called with args:".*"--service-names project-core,project-slurm,project-slurm-no-gpu"
}

@test "dna::build_command with deploy service › expect deploy images only" {
  # Test case: When build command is called with deploy service, it should build deploy images only
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command deploy"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "deploy images native build"
  assert_output --partial "Mock dna::build_project_deploy_service called with args:"
}

@test "dna::build_command with deploy service and --push › expect deploy images with push" {
  # Test case: When build command is called with deploy service and --push, it should build and push deploy images
  run bash -c "export MOCK_DOCKER_LOGIN=true && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command deploy --push"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "deploy images native build"
  assert_output --partial "Mock dna::build_project_deploy_service called with args: --push"
}

@test "dna::build_command with deploy service and --multiarch --push › expect deploy images with push" {
  # Test case: When build command is called with deploy service and --multiarch --push, it should build and push deploy images
  run bash -c "export MOCK_DOCKER_LOGIN=true && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command deploy --multiarch --push"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "deploy images multiarch build procedure"
  assert_output --partial "Mock dna::build_project_deploy_service called with args: --multiarch --push"
}

@test "dna::build_command with --save flag but no service › expect error" {
  # Test case: When build command is called with --save flag but no service, it should show error
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command --save ${MOCK_DNA_DIR}"

  # Should fail
  assert_failure

  # Should output error message
  assert_output --partial "The --save flag can only be used with SERVICE=develop or SERVICE=deploy"
}

@test "dna::build_command with --save flag and invalid service › expect error" {
  # Test case: When build command is called with --save flag and invalid service, it should show error
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command ci-tests --save ${MOCK_DNA_DIR}"

  # Should fail
  assert_failure

  # Should output error message
  assert_output --partial "The --save flag can only be used with SERVICE=develop or SERVICE=deploy"
}

@test "dna::build_command with --save flag and non-existent directory › expect error" {
  # Test case: When build command is called with --save flag and non-existent directory, it should show error
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command develop --save /non/existent/dir"

  # Should fail
  assert_failure

  # Should output error message
  assert_output --partial "Mock n2st::print_msg_error called with args: The DIRPATH does not exist: /non/existent/dir\n"
}

@test "dna::build_command with --save flag missing DIRPATH › expect error" {
  # Test case: When build command is called with --save flag but missing DIRPATH, it should show error
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command develop --save"

  # Should fail
  assert_failure

  # Should output error message
  assert_output --partial "The --save flag requires a DIRPATH argument"
}

@test "dna::build_command with develop service and --save › expect build and save" {
  # Test case: When build command is called with develop service and --save, it should build and then save

  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command develop --save ${MOCK_DNA_DIR}"

  # Should succeed
  assert_success

  # Should output expected messages
  assert_output --partial "develop images native build"
  assert_output --partial "Mock dna::build_services called"
  assert_output --partial "Mock n2st::print_msg: Executing save command as requested"
  assert_output --partial "Mock dna::save_command called with args: ${MOCK_DNA_DIR} develop"
}

@test "dna::build_command with deploy service and --save › expect build and save" {
  # Test case: When build command is called with deploy service and --save, it should build and then save

  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command deploy --save ${MOCK_DNA_DIR}"

  # Should succeed
  assert_success

  # Should output expected messages
  assert_output --partial "deploy images native build"
  assert_output --partial "Mock dna::build_project_deploy_service called"
  assert_output --partial "Mock n2st::print_msg: Executing save command as requested"
  assert_output --partial "Mock dna::save_command called with args: ${MOCK_DNA_DIR} deploy"
}

@test "dna::build_command with --push without deploy service › expect error" {
  # Test case: When build command is called with --push without deploy service, it should show an error
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command --push"

  # Should fail
  assert_failure

  # Should output the error message
  assert_output --partial "Mock dna::illegal_command_msg called with args: build"
  assert_output --partial "The --push flag can only be used with SERVICE=deploy"
}


@test "dna::build_command with --multiarch --online-build › expect multiarch build with force push" {
  # Test case: When build command is called with --multiarch --online-build, it should build multiarch images with force push
  run bash -c "export MOCK_DOCKER_LOGIN=true && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command --multiarch --online-build"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "all images multiarch build"
  assert_output --partial "Mock dna::build_services_multiarch called with args:"
  assert_output --regexp "Mock dna::build_services_multiarch called with args:".*"--force-push-project-core"
}

@test "dna::build_command with --help › expect help menu" {
  # Test case: When build command is called with --help, it should show the help menu
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command --help"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}

@test "dna::build_command with -h › expect help menu" {
  # Test case: When build command is called with -h, it should show the help menu
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command -h"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}

@test "dna::build_command with unknown option › expect error" {
  # Test case: When build command is called with an unknown option, it should show an error
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command --unknown-option"

  # Should fail
  assert_failure

  # Should output the error message
  assert_output --partial "Mock dna::unknown_subcommand_msg called with args: build"
}

@test "dna::build_command with -- and docker arguments › expect arguments passed to build function" {
  # Test case: When build command is called with -- and docker arguments, it should pass them to the build function
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command -- --no-cache --pull"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "all images native build"
  assert_output --regexp "Mock dna::build_services called with args:".*"--no-cache --pull"
}

@test "dna::build_command with develop service and --multiarch › expect multiarch develop images" {
  # Test case: When build command is called with develop service and --multiarch, it should build multiarch develop images
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command --multiarch develop"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "develop images multiarch build"
  assert_output --regexp "Mock dna::build_services_multiarch called with args:".*"--service-names project-core,project-develop"
}

@test "dna::build_command with multiple services › expect error" {
  # Test case: When build command is called with multiple services, it should show an error
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command develop ci-tests"

  # Should fail
  assert_failure

  # Should output the error message
  assert_output --partial "Mock dna::illegal_command_msg called with args: build"
  assert_output --partial "Only one SERVICE can be specified"
}

@test "dna::build_command with unknown service › expect error" {
  # Test case: When build command is called with an unknown service, it should show an error
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command unknown-service"

  # Should fail
  assert_failure

  # Should output the error message
  assert_output --partial "Mock dna::illegal_command_msg called with args: build"
  assert_output --partial "Unknown SERVICE: unknown-service. Valid services are: ci-tests, deploy, develop, slurm"
}

@test "dna::build_command when offline › expect error" {
  # Test case: When build command is called while offline, it should show an error message
  # Mock offline state by setting MOCK_OFFLINE environment variable
  run bash -c "export MOCK_OFFLINE=true && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command"

  # Should fail
  assert_failure

  # Should output the error message about being offline
  assert_output --partial "Mock n2st::print_msg_error called with args: Be advised, you are currently, offline. Executing dna build require internet connection."
}

@test "dna::build_command when offline with service › expect error" {
  # Test case: When build command is called with a service while offline, it should show an error message
  # Mock offline state by setting MOCK_OFFLINE environment variable
  run bash -c "export MOCK_OFFLINE=true && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command develop"

  # Should fail
  assert_failure

  # Should output the error message about being offline
  assert_output --partial "Mock n2st::print_msg_error called with args: Be advised, you are currently, offline. Executing dna build require internet connection."
}

@test "dna::build_command when offline with options › expect error" {
  # Test case: When build command is called with options while offline, it should show an error message
  # Mock offline state by setting MOCK_OFFLINE environment variable
  run bash -c "export MOCK_OFFLINE=true && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command --multiarch"

  # Should fail
  assert_failure

  # Should output the error message about being offline
  assert_output --partial "Mock n2st::print_msg_error called with args: Be advised, you are currently, offline. Executing dna build require internet connection."
}

@test "dna::build_command with ci-tests service and --multiarch › expect multiarch CI tests images" {
  # Test case: When build command is called with ci-tests service and --multiarch, it should build multiarch CI tests images
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command --multiarch ci-tests"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "CI tests images multiarch build"
  assert_output --regexp "Mock dna::build_services_multiarch called with args:".*"--service-names project-core,project-ci-tests,project-ci-tests-no-gpu"
}

@test "dna::build_command with slurm service and --multiarch › expect multiarch slurm images" {
  # Test case: When build command is called with slurm service and --multiarch, it should build multiarch slurm images
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command --multiarch slurm"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "slurm images multiarch build"
  assert_output --regexp "Mock dna::build_services_multiarch called with args:".*"--service-names project-core,project-slurm,project-slurm-no-gpu"
}

@test "dna::build_command with ci-tests service and --online-build › expect CI tests images with force push" {
  # Test case: When build command is called with ci-tests service and --online-build, it should build CI tests images with force push
  run bash -c "export MOCK_DOCKER_LOGIN=true && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command --online-build ci-tests"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "CI tests images native build"
  assert_output --regexp "Mock dna::build_services called with args:".*"--force-push-project-core --service-names project-core,project-ci-tests,project-ci-tests-no-gpu"
}

@test "dna::build_command with slurm service and --online-build › expect slurm images with force push" {
  # Test case: When build command is called with slurm service and --online-build, it should build slurm images with force push
  run bash -c "export MOCK_DOCKER_LOGIN=true && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command --online-build slurm"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "slurm images native build"
  assert_output --regexp "Mock dna::build_services called with args:".*"--force-push-project-core --service-names project-core,project-slurm,project-slurm-no-gpu"
}

@test "dna::build_command with develop service and -- docker args › expect develop images with docker args" {
  # Test case: When build command is called with develop service and docker arguments, it should pass them to the build function
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command develop -- --no-cache --pull"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "develop images native build"
  assert_output --regexp "Mock dna::build_services called with args:".*"--service-names project-core,project-develop --no-cache --pull"
}

@test "dna::build_command with deploy service and -- docker args › expect deploy images with docker args" {
  # Test case: When build command is called with deploy service and docker arguments, it should pass them to the build function
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command deploy -- --no-cache --pull"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "deploy images native build"
  assert_output --regexp "Mock dna::build_project_deploy_service called with args:".*"--no-cache --pull"
}

# ====Docker Hub Login Check Tests================================================================

@test "dna::check_user_is_login_dockerhub when logged in › expect success" {
  # Test case: When user is logged into Docker Hub, the function should return success
  run bash -c "export MOCK_DOCKER_LOGIN=true && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::check_user_is_login_dockerhub"

  # Should succeed
  assert_success
}

@test "dna::check_user_is_login_dockerhub when not logged in › expect failure" {
  # Test case: When user is not logged into Docker Hub, the function should return failure
  run bash -c "export MOCK_DOCKER_LOGIN=false && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::check_user_is_login_dockerhub"

  # Should fail
  assert_failure
}

@test "dna::build_command with --online-build when not logged in › expect failure" {
  # Test case: When build command is called with --online-build but user is not logged into Docker Hub, it should fail
  run bash -c "export MOCK_DOCKER_LOGIN=false && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command --online-build"

  # Should fail
  assert_failure

  # Should output the expected error messages
  assert_output --partial "Online build mode detected (--online-build flag)"
  assert_output --partial "Checking Docker Hub authentication..."
  assert_output --regexp "Build flag".*"--online-build".*"require Docker Hub authentication but user is not logged in!"
  assert_output --partial "Please run docker login to authenticate with Docker Hub before using this command."
}

@test "dna::build_command with --online-build when logged in › expect success" {
  # Test case: When build command is called with --online-build and user is logged into Docker Hub, it should succeed
  run bash -c "export MOCK_DOCKER_LOGIN=true && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command --online-build"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Online build mode detected (--online-build flag)"
  assert_output --partial "Checking Docker Hub authentication..."
  assert_output --partial "Docker Hub authentication verified"
  assert_output --partial "all images native build"
  assert_output --regexp "Mock dna::build_services called with args:".*"--force-push-project-core"
}

@test "dna::build_command with deploy --push when not logged in › expect failure" {
  # Test case: When build command is called with deploy --push but user is not logged into Docker Hub, it should fail
  run bash -c "export MOCK_DOCKER_LOGIN=false && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command deploy --push"

  # Should fail
  assert_failure

  # Should output the expected error messages
  assert_output --partial "Deploy push mode detected (deploy --push flag)"
  assert_output --partial "Checking Docker Hub authentication..."
  assert_output --regexp "Build flag".*"deploy --push".*"require Docker Hub authentication but user is not logged in!"
  assert_output --partial "Please run docker login to authenticate with Docker Hub before using this command."
}

@test "dna::build_command with deploy --push when logged in › expect success" {
  # Test case: When build command is called with deploy --push and user is logged into Docker Hub, it should succeed
  run bash -c "export MOCK_DOCKER_LOGIN=true && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command deploy --push"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Deploy push mode detected (deploy --push flag)"
  assert_output --partial "Checking Docker Hub authentication..."
  assert_output --partial "Docker Hub authentication verified"
  assert_output --partial "deploy images native build"
  assert_output --partial "Mock dna::build_project_deploy_service called with args: --push"
}

@test "dna::build_command with deploy --multiarch --push when not logged in › expect failure" {
  # Test case: When build command is called with deploy --multiarch --push but user is not logged into Docker Hub, it should fail
  run bash -c "export MOCK_DOCKER_LOGIN=false && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command deploy --multiarch --push"

  # Should fail
  assert_failure

  # Should output the expected error messages
  assert_output --partial "Deploy push mode detected (deploy --push flag)"
  assert_output --partial "Checking Docker Hub authentication..."
  assert_output --regexp "Build flag".*"deploy --push".*"require Docker Hub authentication but user is not logged in!"
  assert_output --partial "Please run docker login to authenticate with Docker Hub before using this command."
}

@test "dna::build_command with deploy --multiarch --push when logged in › expect success" {
  # Test case: When build command is called with deploy --multiarch --push and user is logged into Docker Hub, it should succeed
  run bash -c "export MOCK_DOCKER_LOGIN=true && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command deploy --multiarch --push"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Deploy push mode detected (deploy --push flag)"
  assert_output --partial "Checking Docker Hub authentication..."
  assert_output --partial "Docker Hub authentication verified"
  assert_output --partial "deploy images multiarch build procedure"
  assert_output --partial "Mock dna::build_project_deploy_service called with args: --multiarch --push"
}

@test "dna::build_command with --online-build and service when not logged in › expect failure" {
  # Test case: When build command is called with --online-build and a service but user is not logged into Docker Hub, it should fail
  run bash -c "export MOCK_DOCKER_LOGIN=false && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command --online-build develop"

  # Should fail
  assert_failure

  # Should output the expected error messages
  assert_output --partial "Online build mode detected (--online-build flag)"
  assert_output --partial "Checking Docker Hub authentication..."
  assert_output --regexp "Build flag".*"--online-build".*"require Docker Hub authentication but user is not logged in!"
  assert_output --partial "Please run docker login to authenticate with Docker Hub before using this command."
}

@test "dna::build_command with --online-build and service when logged in › expect success" {
  # Test case: When build command is called with --online-build and a service and user is logged into Docker Hub, it should succeed
  run bash -c "export MOCK_DOCKER_LOGIN=true && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command --online-build develop"

  # Should succeed
  assert_success

  # Should output the expected messages
  assert_output --partial "Online build mode detected (--online-build flag)"
  assert_output --partial "Checking Docker Hub authentication..."
  assert_output --partial "Docker Hub authentication verified"
  assert_output --partial "develop images native build"
  assert_output --regexp "Mock dna::build_services called with args:".*"--force-push-project-core --service-names project-core,project-develop"
}

@test "dna::build_command with deploy without --push › expect no login check" {
  # Test case: When build command is called with deploy service but without --push, it should not check Docker Hub login
  run bash -c "export MOCK_DOCKER_LOGIN=false && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command deploy"

  # Should succeed (no login check required)
  assert_success

  # Should not output login check messages
  refute_output --partial "Checking Docker Hub authentication..."
  refute_output --partial "Docker Hub authentication required"

  # Should output the expected build message
  assert_output --partial "deploy images native build"
  assert_output --partial "Mock dna::build_project_deploy_service called with args:"
}

@test "dna::build_command with regular service › expect no login check" {
  # Test case: When build command is called with regular service (not deploy --push or --online-build), it should not check Docker Hub login
  run bash -c "export MOCK_DOCKER_LOGIN=false && source ${MOCK_DNA_DIR}/src/lib/commands/build.bash && dna::build_command develop"

  # Should succeed (no login check required)
  assert_success

  # Should not output login check messages
  refute_output --partial "Checking Docker Hub authentication..."
  refute_output --partial "Docker Hub authentication required"

  # Should output the expected build message
  assert_output --partial "develop images native build"
  assert_output --partial "Mock dna::build_services called with args:"
}
