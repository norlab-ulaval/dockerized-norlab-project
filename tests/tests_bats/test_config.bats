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

TESTED_FILE="config.bash"
TESTED_FILE_PATH="src/lib/commands"

# ....Setup........................................................................................
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
  export MOCK_PROJECT_PATH="${BATS_DOCKER_WORKDIR}/utilities/tmp/dockerized-norlab-project-mock"

  # Create temporary directory for tests
  export MOCK_DNA_DIR=$(temp_make)

  # Create mock functions directory in the temporary directory
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/utils/"
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/docker/"

  # Create mock load_super_project_config.bash
  cat > "${MOCK_DNA_DIR}/src/lib/core/utils/load_super_project_config.bash" << 'EOF'
#!/bin/bash
# Mock load_super_project_config.bash
echo "Mock load_super_project_config.bash loaded"
return 0
EOF

  # Create mock docker compose files
  for compose_file in "docker-compose.build.native.yaml" \
                      "docker-compose.build.multiarch.yaml" \
                      "docker-compose.run.darwin.yaml" \
                      "docker-compose.run.linux-x86.yaml" \
                      "docker-compose.run.jetson.yaml" \
                      "docker-compose.run.ci-tests.yaml" \
                      "docker-compose.run.slurm.yaml"; do
    cat > "${MOCK_DNA_DIR}/src/lib/core/docker/${compose_file}" << 'EOF'
version: '3.8'
services:
  mock-service:
    image: mock:latest
EOF
  done

  # Create a mock import_dna_lib.bash that sets up the environment
  cat > "${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash" << 'EOF'
#!/bin/bash
# Mock import_dna_lib.bash

# ....Setup........................................................................................

# Set message formatting variables
export MSG_DIMMED_FORMAT=""
export MSG_END_FORMAT=""

# Set up environment variables
export DNA_ROOT="${MOCK_DNA_DIR}"
export DNA_LIB_PATH="${DNA_ROOT}/src/lib"

# ....Mock dependencies loading test functions.....................................................
function dna::import_lib_and_dependencies() {
  return 0
}

# ....Mock N2ST functions..........................................................................
function n2st::print_msg() {
  echo "Mock n2st::print_msg called with args: $*"
  return 0
}

function n2st::print_msg_warning() {
  echo "Mock n2st::print_msg_warning called with args: $*"
  return 0
}

# ....Mock ui.bash functions.......................................................................
function dna::command_help_menu() {
  echo "Mock dna::command_help_menu called with documentation"
  return 0
}

function dna::illegal_command_msg() {
  echo "Mock dna::illegal_command_msg called with command: $1, args: $2, error: $3"
  return 1
}

# ....Mock docker commands.........................................................................
function docker() {
  echo "Mock docker command called with args: $*"
  return 0
}

# ....Export mock functions........................................................................
for func in $(compgen -A function | grep -e dna:: -e n2st::); do
  # shellcheck disable=SC2163
  export -f "${func}"
done

# Export docker mock function
export -f docker

# ....Teardown.....................................................................................
# Print a message to indicate that the mock import_dna_lib.bash has been loaded
echo "[dna done] Mock import_dna_lib.bash and its librairies loaded"
EOF
}

setup() {
  # Create necessary directories in the temporary directory
  mkdir -p "${MOCK_DNA_DIR}/src/lib/commands"
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/utils"
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/docker"

  # Copy the config.bash file to the temporary directory
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

# ....Help and error cases........................................................................

@test "dna::config_command with no arguments › expect help message and exit with code 1" {
  # Test case: When config command is called without arguments, it should show help and exit with code 1
  # Expected behavior: Shows help menu and exits with failure code
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command"

  # Should fail (exit code 1)
  assert_failure

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with documentation"
}

@test "dna::config_command with --help › expect help menu and exit with code 0" {
  # Test case: When config command is called with --help, it should show the help menu and exit successfully
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command --help"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with documentation"
}

@test "dna::config_command with -h › expect help menu and exit with code 0" {
  # Test case: When config command is called with -h, it should show the help menu and exit successfully
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command -h"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with documentation"
}

@test "dna::config_command with unknown mode › expect illegal command message and exit with code 1" {
  # Test case: When config command is called with unknown mode, it should show error message and exit with code 1
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command unknown-mode"

  # Should fail (exit code 1)
  assert_failure

  # Should output the illegal command message
  assert_output --partial "Mock dna::illegal_command_msg called with command: config"
}

# ....Build modes tests...........................................................................

@test "dna::config_command build-core › expect native build config with correct services" {
  # Test case: When config command is called with build-core mode, it should use native build config with core services
  # Expected behavior: Uses docker-compose.build.native.yaml with project-core-pre, project-core-user, project-core services
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command build-core"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show configuration message
  assert_output --partial "Mock n2st::print_msg called with args: Showing build-core mode configuration from docker-compose.build.native.yaml"

  # Should call docker compose config with correct file and services
  assert_output --partial "Mock docker command called with args: compose --file ${DNA_LIB_PATH}/core/docker/docker-compose.build.native.yaml config"
  assert_output --partial "project-core-pre project-core-user project-core"
}

@test "dna::config_command build-core-ma › expect multiarch build config with correct services" {
  # Test case: When config command is called with build-core-ma mode, it should use multiarch build config with core services
  # Expected behavior: Uses docker-compose.build.multiarch.yaml with project-core-pre, project-core-user, project-core services
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command build-core-ma"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show configuration message
  assert_output --partial "Mock n2st::print_msg called with args: Showing build-core-ma mode configuration from docker-compose.build.multiarch.yaml"

  # Should call docker compose config with correct file and services
  assert_output --partial "Mock docker command called with args: compose --file ${DNA_LIB_PATH}/core/docker/docker-compose.build.multiarch.yaml config"
  assert_output --partial "project-core-pre project-core-user project-core"
}

@test "dna::config_command build › expect native build config for all services" {
  # Test case: When config command is called with build mode, it should use native build config for all services
  # Expected behavior: Uses docker-compose.build.native.yaml for all services
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command build"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show configuration message
  assert_output --partial "Mock n2st::print_msg called with args: Showing build mode configuration from docker-compose.build.native.yaml"

  # Should call docker compose config with correct file
  assert_output --partial "Mock docker command called with args: compose --file ${DNA_LIB_PATH}/core/docker/docker-compose.build.native.yaml config"
}

@test "dna::config_command build-ma › expect multiarch build config for all services" {
  # Test case: When config command is called with build-ma mode, it should use multiarch build config for all services
  # Expected behavior: Uses docker-compose.build.multiarch.yaml for all services
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command build-ma"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show configuration message
  assert_output --partial "Mock n2st::print_msg called with args: Showing build-ma mode configuration from docker-compose.build.multiarch.yaml"

  # Should call docker compose config with correct file
  assert_output --partial "Mock docker command called with args: compose --file ${DNA_LIB_PATH}/core/docker/docker-compose.build.multiarch.yaml config"
}

# ....Development mode tests with platforms.......................................................

@test "dna::config_command dev › expect linux development config by default" {
  # Test case: When config command is called with dev mode without platform, it should use linux config
  # Expected behavior: Uses docker-compose.run.linux-x86.yaml with project-develop service
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command dev"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show configuration message
  assert_output --partial "Mock n2st::print_msg called with args: Showing dev mode configuration from docker-compose.run.linux-x86.yaml"

  # Should call docker compose config with correct file and service
  assert_output --partial "Mock docker command called with args: compose --file ${DNA_LIB_PATH}/core/docker/docker-compose.run.linux-x86.yaml config"
  assert_output --partial "project-develop"
}

@test "dna::config_command dev darwin › expect darwin development config" {
  # Test case: When config command is called with dev mode and darwin platform, it should use darwin config
  # Expected behavior: Uses docker-compose.run.darwin.yaml with project-develop service
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command dev darwin"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show configuration message
  assert_output --partial "Mock n2st::print_msg called with args: Showing dev mode configuration from docker-compose.run.darwin.yaml"

  # Should call docker compose config with correct file and service
  assert_output --partial "Mock docker command called with args: compose --file ${DNA_LIB_PATH}/core/docker/docker-compose.run.darwin.yaml config"
  assert_output --partial "project-develop"
}

@test "dna::config_command dev linux › expect linux development config" {
  # Test case: When config command is called with dev mode and linux platform, it should use linux config
  # Expected behavior: Uses docker-compose.run.linux-x86.yaml with project-develop service
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command dev linux"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show configuration message
  assert_output --partial "Mock n2st::print_msg called with args: Showing dev mode configuration from docker-compose.run.linux-x86.yaml"

  # Should call docker compose config with correct file and service
  assert_output --partial "Mock docker command called with args: compose --file ${DNA_LIB_PATH}/core/docker/docker-compose.run.linux-x86.yaml config"
  assert_output --partial "project-develop"
}

@test "dna::config_command dev jetson › expect jetson development config" {
  # Test case: When config command is called with dev mode and jetson platform, it should use jetson config
  # Expected behavior: Uses docker-compose.run.jetson.yaml with project-develop service
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command dev jetson"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show configuration message
  assert_output --partial "Mock n2st::print_msg called with args: Showing dev mode configuration from docker-compose.run.jetson.yaml"

  # Should call docker compose config with correct file and service
  assert_output --partial "Mock docker command called with args: compose --file ${DNA_LIB_PATH}/core/docker/docker-compose.run.jetson.yaml config"
  assert_output --partial "project-develop"
}

# ....Deployment mode tests with platforms........................................................

@test "dna::config_command deploy › expect linux deployment config by default" {
  # Test case: When config command is called with deploy mode without platform, it should use linux config
  # Expected behavior: Uses docker-compose.run.linux-x86.yaml with project-deploy service
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command deploy"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show configuration message
  assert_output --partial "Mock n2st::print_msg called with args: Showing deploy mode configuration from docker-compose.run.linux-x86.yaml"

  # Should call docker compose config with correct file and service
  assert_output --partial "Mock docker command called with args: compose --file ${DNA_LIB_PATH}/core/docker/docker-compose.run.linux-x86.yaml config"
  assert_output --partial "project-deploy"
}

@test "dna::config_command deploy darwin › expect darwin deployment config" {
  # Test case: When config command is called with deploy mode and darwin platform, it should use darwin config
  # Expected behavior: Uses docker-compose.run.darwin.yaml with project-deploy service
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command deploy darwin"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show configuration message
  assert_output --partial "Mock n2st::print_msg called with args: Showing deploy mode configuration from docker-compose.run.darwin.yaml"

  # Should call docker compose config with correct file and service
  assert_output --partial "Mock docker command called with args: compose --file ${DNA_LIB_PATH}/core/docker/docker-compose.run.darwin.yaml config"
  assert_output --partial "project-deploy"
}

@test "dna::config_command deploy jetson › expect jetson deployment config" {
  # Test case: When config command is called with deploy mode and jetson platform, it should use jetson config
  # Expected behavior: Uses docker-compose.run.jetson.yaml with project-deploy service
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command deploy jetson"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show configuration message
  assert_output --partial "Mock n2st::print_msg called with args: Showing deploy mode configuration from docker-compose.run.jetson.yaml"

  # Should call docker compose config with correct file and service
  assert_output --partial "Mock docker command called with args: compose --file ${DNA_LIB_PATH}/core/docker/docker-compose.run.jetson.yaml config"
  assert_output --partial "project-deploy"
}

# ....CI and SLURM mode tests.....................................................................

@test "dna::config_command ci-tests › expect ci-tests configuration" {
  # Test case: When config command is called with ci-tests mode, it should use ci-tests config
  # Expected behavior: Uses docker-compose.run.ci-tests.yaml with project-ci-tests service
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command ci-tests"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show configuration message
  assert_output --partial "Mock n2st::print_msg called with args: Showing ci-tests mode configuration from docker-compose.run.ci-tests.yaml"

  # Should call docker compose config with correct file and service
  assert_output --partial "Mock docker command called with args: compose --file ${DNA_LIB_PATH}/core/docker/docker-compose.run.ci-tests.yaml config"
  assert_output --partial "project-ci-tests"
}

@test "dna::config_command slurm › expect slurm configuration" {
  # Test case: When config command is called with slurm mode, it should use slurm config
  # Expected behavior: Uses docker-compose.run.slurm.yaml with project-slurm service
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command slurm"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show configuration message
  assert_output --partial "Mock n2st::print_msg called with args: Showing slurm mode configuration from docker-compose.run.slurm.yaml"

  # Should call docker compose config with correct file and service
  assert_output --partial "Mock docker command called with args: compose --file ${DNA_LIB_PATH}/core/docker/docker-compose.run.slurm.yaml config"
  assert_output --partial "project-slurm"
}

@test "dna::config_command release › expect warning message about not released yet" {
  # Test case: When config command is called with release mode, it should show warning about not being released yet
  # Expected behavior: Shows warning message and exits successfully
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command release"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show warning message
  assert_output --partial "Mock n2st::print_msg_warning called with args: Command dna config release is not released yet, stay tuned!"
}

# ....Option tests.................................................................................

@test "dna::config_command build --bake › expect buildx bake command instead of compose config" {
  # Test case: When config command is called with --bake option, it should use docker buildx bake instead of compose config
  # Expected behavior: Changes directory and uses docker buildx bake --file --print
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command build --bake"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show configuration message
  assert_output --partial "Mock n2st::print_msg called with args: Showing build mode configuration from docker-compose.build.native.yaml"

  # Should call docker buildx bake instead of compose config
  assert_output --partial "Mock docker command called with args: buildx bake --file docker-compose.build.native.yaml --print"
}

@test "dna::config_command dev --bake › expect warning about pointless bake with non-build mode" {
  # Test case: When config command is called with --bake option and non-build mode, it should show warning
  # Expected behavior: Shows warning and returns 0
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command dev --bake"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show warning about pointless usage
  assert_output --partial "Mock n2st::print_msg_warning called with args: Using --bake flag with non-build mode dev is pointless"
}

@test "dna::config_command build --compose-to-bake › expect compose build --print command" {
  # Test case: When config command is called with --compose-to-bake option, it should use compose build --print
  # Expected behavior: Uses docker compose build --print
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command build --compose-to-bake"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show configuration message
  assert_output --partial "Mock n2st::print_msg called with args: Showing build mode configuration from docker-compose.build.native.yaml"

  # Should call docker compose build --print
  assert_output --partial "Mock docker command called with args: compose --file ${DNA_LIB_PATH}/core/docker/docker-compose.build.native.yaml build --print"
}

@test "dna::config_command deploy --compose-to-bake › expect warning about pointless compose-to-bake with non-build mode" {
  # Test case: When config command is called with --compose-to-bake option and non-build mode, it should show warning
  # Expected behavior: Shows warning and returns 0
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command deploy --compose-to-bake"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show warning about pointless usage
  assert_output --partial "Mock n2st::print_msg_warning called with args: Using --compose-to-bake flag with non-build mode deploy is pointless"
}

@test "dna::config_command build --quiet › expect no dna messages, only docker command output" {
  # Test case: When config command is called with --quiet or -q option, it should skip dna messages
  # Expected behavior: Skips n2st::print_msg calls and loads config silently
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command build --quiet"

  # Should succeed
  assert_success

  # Should load super project config (but output should be redirected to /dev/null in actual implementation)
  # Note: Our mock still outputs, but real implementation redirects to /dev/null

  # Should NOT show configuration message (because dna_quiet=true)
  refute_output --partial "Mock n2st::print_msg called with args: Showing build mode configuration"

  # Should call docker compose config
  assert_output --partial "Mock docker command called with args: compose --file ${DNA_LIB_PATH}/core/docker/docker-compose.build.native.yaml config"
}

@test "dna::config_command build -q › expect no dna messages, only docker command output" {
  # Test case: When config command is called with -q option, it should skip dna messages
  # Expected behavior: Skips n2st::print_msg calls and loads config silently
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command build -q"

  # Should succeed
  assert_success

  # Should load super project config (but output should be redirected to /dev/null in actual implementation)
  # Note: Our mock still outputs, but real implementation redirects to /dev/null

  # Should NOT show configuration message (because dna_quiet=true)
  refute_output --partial "Mock n2st::print_msg called with args: Showing build mode configuration"

  # Should call docker compose config
  assert_output --partial "Mock docker command called with args: compose --file ${DNA_LIB_PATH}/core/docker/docker-compose.build.native.yaml config"
}

# ....Docker flags passthrough tests..............................................................

@test "dna::config_command build -- --services › expect docker flags passed through" {
  # Test case: When config command is called with -- separator, remaining arguments should be passed to docker
  # Expected behavior: Additional arguments are passed to docker command
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command build -- --services"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show configuration message
  assert_output --partial "Mock n2st::print_msg called with args: Showing build mode configuration from docker-compose.build.native.yaml"

  # Should call docker compose config with additional flags
  assert_output --partial "Mock docker command called with args: compose --file ${DNA_LIB_PATH}/core/docker/docker-compose.build.native.yaml config --services"
}

@test "dna::config_command build --no-interpolate › expect docker flags passed through without separator" {
  # Test case: When config command is called with docker flags without --, they should be passed through
  # Expected behavior: Unrecognized arguments are passed to docker command
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/config.bash && dna::config_command build --no-interpolate"

  # Should succeed
  assert_success

  # Should load super project config
  assert_output --partial "Mock load_super_project_config.bash loaded"

  # Should show configuration message
  assert_output --partial "Mock n2st::print_msg called with args: Showing build mode configuration from docker-compose.build.native.yaml"

  # Should call docker compose config with additional flags
  assert_output --partial "Mock docker command called with args: compose --file ${DNA_LIB_PATH}/core/docker/docker-compose.build.native.yaml config --no-interpolate"
}
