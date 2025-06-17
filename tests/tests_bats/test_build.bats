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
  export MOCK_DNP_DIR=$(temp_make)

  # Create mock functions directory in the temporary directory
  mkdir -p "${MOCK_DNP_DIR}/src/lib/core/execute/"
  mkdir -p "${MOCK_DNP_DIR}/src/lib/core/utils/"


  # Create mock functions for dependencies
  cat > "${MOCK_DNP_DIR}/src/lib/core/utils/load_super_project_config.bash" << 'EOF'
#!/bin/bash
# Mock load_super_project_config.bash
echo "Mock load_super_project_config.bash loaded"
return 0
EOF

  cat > "${MOCK_DNP_DIR}/src/lib/core/execute/build.all.bash" << 'EOF'
#!/bin/bash
# Mock build.all.bash
function dnp::build_services() {
  echo "Mock dnp::build_services called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNP_DIR}/src/lib/core/execute/build.all.multiarch.bash" << 'EOF'
#!/bin/bash
# Mock build.all.multiarch.bash
function dnp::build_services_multiarch() {
  echo "Mock dnp::build_services_multiarch called with args: $*"
  return 0
}
EOF

  cat > "${MOCK_DNP_DIR}/src/lib/core/execute/build.deploy.bash" << 'EOF'
#!/bin/bash
# Mock build.deploy.bash
function dnp::build_project_deploy_service() {
  echo "Mock dnp::build_project_deploy_service called with args: $*"
  return 0
}
EOF

  # Create a mock import_dnp_lib.bash that sets up the environment
  cat > "${MOCK_DNP_DIR}/src/lib/core/utils/import_dnp_lib.bash" << 'EOF'
#!/bin/bash
# Mock import_dnp_lib.bash

# ....Setup........................................................................................

# Set message formatting variables
export MSG_DIMMED_FORMAT=""
export MSG_END_FORMAT=""
export MSG_LINE_CHAR_BUILDER_LVL1="-"

# Set up environment variables
export DNP_ROOT="${MOCK_DNP_DIR}"
export DNP_LIB_PATH="${MOCK_DNP_DIR}/src/lib"
export DNP_LIB_EXEC_PATH="${MOCK_DNP_DIR}/src/lib/core/execute"

# ....Mock dependencies loading test functions.....................................................
function dnp::import_lib_and_dependencies() {
  return 0
}

function n2st::print_msg() {
  return 0
}

# ....Mock ui.bash functions.......................................................................
function dnp::command_help_menu() {
  echo "Mock dnp::command_help_menu called with args: $*"
  return 0
}

function dnp::illegal_command_msg() {
  echo "Mock dnp::illegal_command_msg called with args: $*"
  return 1
}

function dnp::unknown_subcommand_msg() {
  echo "Mock dnp::unknown_subcommand_msg called with args: $*"
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
  mkdir -p "${MOCK_DNP_DIR}/src/lib/core/utils"
  mkdir -p "${MOCK_DNP_DIR}/src/lib/core/execute"

  # Copy the build.bash file to the temporary directory
  cp "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" "${MOCK_DNP_DIR}/src/lib/commands/"
#  cp "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/ui.bash" "${MOCK_DNP_DIR}/src/lib/core/utils/"

  source "${MOCK_DNP_DIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1

#  # Set up environment variables
#  export DNP_ROOT="${MOCK_DNP_DIR}"
#  export DNP_LIB_PATH="${MOCK_DNP_DIR}/src/lib"
#  export DNP_LIB_EXEC_PATH="${MOCK_DNP_DIR}/src/lib/core/execute"

  # Change to the temporary directory
  cd "${MOCK_DNP_DIR}" || exit 1
}

# ....Teardown.....................................................................................
teardown() {
  bats_print_run_env_variable_on_error
}

teardown_file() {
  # Clean up temporary directory
  temp_del "${MOCK_DNP_DIR}"
}

# ====Test cases==================================================================================

@test "dnp::build_command with no arguments › expect default behavior" {
  # Test case: When build command is called without arguments, it should build all images with native architecture
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/build.bash && dnp::build_command"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "all images (native) build"
  assert_output --partial "Mock dnp::build_services called with args:"
}

@test "dnp::build_command with --multiarch › expect multiarch build" {
  # Test case: When build command is called with --multiarch, it should build all images with multiarch
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/build.bash && dnp::build_command --multiarch"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "all images (multiarch) build"
  assert_output --partial "Mock dnp::build_services_multiarch called with args: --no-force-push-project-core"
}

@test "dnp::build_command with --online-build › expect force push flag" {
  # Test case: When build command is called with --online-build, it should pass the flag to build_services
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/build.bash && dnp::build_command --online-build"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "all images (native) build"
  assert_output --partial "Mock dnp::build_services called with args: --force-push-project-core"
}

@test "dnp::build_command with develop service › expect develop images only" {
  # Test case: When build command is called with develop service, it should build develop images only
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/build.bash && dnp::build_command develop"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "develop images (native) build"
  assert_output --partial "Mock dnp::build_services called with args: --service-names project-core,project-develop"
}

@test "dnp::build_command with ci-tests service › expect CI tests images only" {
  # Test case: When build command is called with ci-tests service, it should build CI tests images only
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/build.bash && dnp::build_command ci-tests"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "CI tests images (native) build"
  assert_output --partial "Mock dnp::build_services called with args: --service-names project-core,project-ci-tests,project-ci-tests-no-gpu"
}

@test "dnp::build_command with slurm service › expect slurm images only" {
  # Test case: When build command is called with slurm service, it should build slurm images only
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/build.bash && dnp::build_command slurm"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "slurm images (native) build"
  assert_output --partial "Mock dnp::build_services called with args: --service-names project-core,project-slurm,project-slurm-no-gpu"
}

@test "dnp::build_command with deploy service › expect deploy images only" {
  # Test case: When build command is called with deploy service, it should build deploy images only
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/build.bash && dnp::build_command deploy"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "deploy images (native) build"
  assert_output --partial "Mock dnp::build_project_deploy_service called with args:"
}

@test "dnp::build_command with deploy service and --push › expect deploy images with push" {
  # Test case: When build command is called with deploy service and --push, it should build and push deploy images
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/build.bash && dnp::build_command deploy --push"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "deploy images (native) build"
  assert_output --partial "Mock dnp::build_project_deploy_service called with args: --push"
}

@test "dnp::build_command with deploy service and --multiarch --push › expect deploy images with push" {
  # Test case: When build command is called with deploy service and --multiarch --push, it should build and push deploy images
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/build.bash && dnp::build_command deploy --multiarch --push"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "deploy images (multiarch) build procedure"
  assert_output --partial "Mock dnp::build_project_deploy_service called with args: --multiarch --push"
}

@test "dnp::build_command with --push without deploy service › expect error" {
  # Test case: When build command is called with --push without deploy service, it should show an error
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/build.bash && dnp::build_command --push"

  # Should fail
  assert_failure

  # Should output the error message
  assert_output --partial "Mock dnp::illegal_command_msg called with args: build"
  assert_output --partial "The --push flag can only be used with SERVICE=deploy"
}


@test "dnp::build_command with --multiarch --online-build › expect multiarch build with force push" {
  # Test case: When build command is called with --multiarch --online-build, it should build multiarch images with force push
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/build.bash && dnp::build_command --multiarch --online-build"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "all images (multiarch) build"
  assert_output --partial "Mock dnp::build_services_multiarch called with args:"
  # Note: --no-force-push-project-core flag should not be present when --online-build is specified
  refute_output --partial "Mock dnp::build_services_multiarch called with args: --no-force-push-project-core"
}

@test "dnp::build_command with --help › expect help menu" {
  # Test case: When build command is called with --help, it should show the help menu
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/build.bash && dnp::build_command --help"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dnp::command_help_menu called with args:"
}

@test "dnp::build_command with -h › expect help menu" {
  # Test case: When build command is called with -h, it should show the help menu
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/build.bash && dnp::build_command -h"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dnp::command_help_menu called with args:"
}

@test "dnp::build_command with unknown option › expect error" {
  # Test case: When build command is called with an unknown option, it should show an error
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/build.bash && dnp::build_command --unknown-option"

  # Should fail
  assert_failure

  # Should output the error message
  assert_output --partial "Mock dnp::unknown_subcommand_msg called with args: build"
}

@test "dnp::build_command with -- and docker arguments › expect arguments passed to build function" {
  # Test case: When build command is called with -- and docker arguments, it should pass them to the build function
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/build.bash && dnp::build_command -- --no-cache --pull"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "all images (native) build"
  assert_output --partial "Mock dnp::build_services called with args: --no-cache --pull"
}

@test "dnp::build_command with develop service and --multiarch › expect multiarch develop images" {
  # Test case: When build command is called with develop service and --multiarch, it should build multiarch develop images
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/build.bash && dnp::build_command --multiarch develop"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "develop images (multiarch) build"
  assert_output --partial "Mock dnp::build_services_multiarch called with args: --no-force-push-project-core --service-names project-core,project-develop"
}

@test "dnp::build_command with multiple services › expect error" {
  # Test case: When build command is called with multiple services, it should show an error
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/build.bash && dnp::build_command develop ci-tests"

  # Should fail
  assert_failure

  # Should output the error message
  assert_output --partial "Mock dnp::illegal_command_msg called with args: build"
  assert_output --partial "Only one SERVICE can be specified"
}

@test "dnp::build_command with unknown service › expect error" {
  # Test case: When build command is called with an unknown service, it should show an error
  run bash -c "source ${MOCK_DNP_DIR}/src/lib/commands/build.bash && dnp::build_command unknown-service"

  # Should fail
  assert_failure

  # Should output the error message
  assert_output --partial "Mock dnp::illegal_command_msg called with args: build"
  assert_output --partial "Unknown SERVICE: unknown-service. Valid services are: ci-tests, deploy, develop, slurm"
}
