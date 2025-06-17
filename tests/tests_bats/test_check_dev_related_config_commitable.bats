#!/usr/bin/env bats
#
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

BATS_HELPER_PATH=/usr/lib/bats
if [[ -d ${BATS_HELPER_PATH} ]]; then
  load "${BATS_HELPER_PATH}/bats-support/load"
  load "${BATS_HELPER_PATH}/bats-assert/load"
  load "${BATS_HELPER_PATH}/bats-file/load"
  load "${SRC_CODE_PATH}/${N2ST_BATS_TESTING_TOOLS_RELATIVE_PATH}/bats_helper_functions"
  load "${SRC_CODE_PATH}/tests/tests_bats/bats_testing_tools/bats_helper_functions_local"
  #load "${BATS_HELPER_PATH}/bats-detik/load" # << Kubernetes support
else
  echo -e "\n[\033[1;31mERROR\033[0m] $0 path to bats-core helper library unreachable at \"${BATS_HELPER_PATH}\"!" 1>&2
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi

# ====Setup========================================================================================
TESTED_FILE_PATH="src/lib/core/docker"
MOCK_PROJECT_PATH="utilities/tmp/dockerized-norlab-project-mock"
TESTED_FILE_PATH2="${MOCK_PROJECT_PATH}/.dockerized_norlab_project/configuration/"
TESTED_FILE_PATH3="./"

# executed once before starting the first test (valide for all test in that file)
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR

#  # Uncomment the following for debug, the ">&3" is for printing bats msg to stdin
#  echo -e "\033[1;2m
#  \n...N2ST bats tests environment.................................................................
#  \n$( tree -L 1 -a -hug $PWD && printenv )
#  \n...............................................................................................
#  \033[0m"  >&3
#
#  echo -e "
#  \n...DNP related environment varaibles...........................................................
#  \n$(printenv | grep -e DNP_)
#  \n...............................................................................................
#  \n" >&3
}

# executed before each test
#setup() {
#}

# ====Teardown=====================================================================================

# executed after each test
teardown() {
  bats_print_run_env_variable_on_error
}

## executed once after finishing the last test (valide for all test in that file)
#teardown_file() {
#
#}

# ====Helper functions=============================================================================
function helper::setup_compose_related_tests() {
  local tested_file=$1
  local tested_file_path=$2
  cd "${tested_file_path}" || exit 1
  echo -e "\n\n
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
[DN ERROR] -> docker-compose yaml file misconfiguration error.

A DN_ENTRYPOINT_TRACE_EXECUTION environment variable in ${tested_file} is set to true i.e.,

environment:
  DN_ENTRYPOINT_TRACE_EXECUTION: true

Its OK for developement but MAKE SURE ITS SET TO FALSE for PUSH TO CI BUILD.
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
\n"
}

function helper::setup_dotenv_related_tests() {
  local tested_file=$1
  local tested_env_var=$2
  local expected_value=$3
  local tested_file_path=$4
  cd "${tested_file_path}" || exit 1
  echo -e "\n\n
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
[DN ERROR] -> dotenv file ${tested_file} misconfiguration error.
Path: ${tested_file_path}${tested_file}

Actual:
  $( cat "${tested_file}" | grep -e "^${tested_env_var}" )

Expected:
  ${tested_env_var}=${expected_value}

Its OK for developement but MAKE SURE ITS SET TO THE EXPECTED VALUE for PUSH TO CI BUILD.
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
\n"
}

function helper::setup_ignore_related_tests() {
  local tested_file=$1
  local tested_entry=$2
  local tested_file_path=$3
  cd "${tested_file_path}" || exit 1
  echo -e "\n\n
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
[DN ERROR] -> ignore file ${tested_file} misconfiguration error.
Path: ${tested_file_path}${tested_file}

Expected entry:
  $( cat "${tested_file}" | grep -e "^${tested_entry}" )

This entry is required, MAKE SURE ITS SET TO THE EXPECTED VALUE for PUSH TO CI BUILD.
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
\n"
}

# ====Test casses==================================================================================

# ....docker-compose tests.........................................................................
@test "check dev configs are muted in docker-compose.project.build.native.yaml › expect pass" {
  helper::setup_compose_related_tests "docker-compose.project.build.native.yaml" "${TESTED_FILE_PATH}"
  assert_file_exist "docker-compose.project.build.native.yaml"
  assert_file_not_contains "docker-compose.project.build.native.yaml" "DN_ENTRYPOINT_TRACE_EXECUTION: true"
}

@test "check dev configs are muted in docker-compose.project.run.darwin.yaml › expect pass" {
  helper::setup_compose_related_tests "docker-compose.project.run.darwin.yaml" "${TESTED_FILE_PATH}"
  assert_file_exist "docker-compose.project.run.darwin.yaml"
  assert_file_not_contains "docker-compose.project.run.darwin.yaml" "DN_ENTRYPOINT_TRACE_EXECUTION: true"
}

@test "check dev configs are muted in docker-compose.project.run.jetson.yaml › expect pass" {
  helper::setup_compose_related_tests "docker-compose.project.run.jetson.yaml" "${TESTED_FILE_PATH}"
  assert_file_exist "docker-compose.project.run.jetson.yaml"
  assert_file_not_contains "docker-compose.project.run.jetson.yaml" "DN_ENTRYPOINT_TRACE_EXECUTION: true"
}

@test "check dev configs are muted in docker-compose.project.run.linux-x86.yaml › expect pass" {
  helper::setup_compose_related_tests "docker-compose.project.run.linux-x86.yaml" "${TESTED_FILE_PATH}"
  assert_file_exist "docker-compose.project.run.linux-x86.yaml"
  assert_file_not_contains "docker-compose.project.run.linux-x86.yaml" "DN_ENTRYPOINT_TRACE_EXECUTION: true"
}

@test "check dev configs are muted in docker-compose.project.run.slurm.yaml › expect pass" {
  helper::setup_compose_related_tests "docker-compose.project.run.slurm.yaml" "${TESTED_FILE_PATH}"
  assert_file_exist "docker-compose.project.run.slurm.yaml"
  assert_file_not_contains "docker-compose.project.run.slurm.yaml" "DN_ENTRYPOINT_TRACE_EXECUTION: true"
}

@test "check dev configs are muted in docker-compose.project.run.ci-tests.yaml › expect pass" {
  helper::setup_compose_related_tests "docker-compose.project.run.ci-tests.yaml" "${TESTED_FILE_PATH}"
  assert_file_exist "docker-compose.project.run.ci-tests.yaml"
  assert_file_not_contains "docker-compose.project.run.ci-tests.yaml" "DN_ENTRYPOINT_TRACE_EXECUTION: true"
}

# ....dotenv tests.................................................................................
@test "check dev configs in .env.dnp | DN_GIT_BRANCH=dev › expect pass" {
  helper::setup_dotenv_related_tests '.env.dnp' 'DN_GIT_BRANCH' 'dev' "${TESTED_FILE_PATH2}"
  assert_file_exist ".env.dnp"
  assert_file_contains ".env.dnp" "^DN_GIT_BRANCH=dev"
}

@test "check dev configs in .env.dockerized-norlab-project | DNP_DEBUG=false › expect pass" {
  helper::setup_dotenv_related_tests '.env.dockerized-norlab-project' 'DNP_DEBUG' 'false' "${TESTED_FILE_PATH3}"
  assert_file_exist ".env.dockerized-norlab-project"
  assert_file_contains ".env.dockerized-norlab-project" "^DNP_DEBUG=false"
}

@test "check dev configs in .env.dockerized-norlab-project | DNP_CLEAR_CONSOLE_ACTIVATED=false › expect pass" {
  helper::setup_dotenv_related_tests '.env.dockerized-norlab-project' 'DNP_CLEAR_CONSOLE_ACTIVATED' 'false' "${TESTED_FILE_PATH3}"
  assert_file_exist ".env.dockerized-norlab-project"
  assert_file_contains ".env.dockerized-norlab-project" "^DNP_CLEAR_CONSOLE_ACTIVATED=false"
}

# ....DNP app .dockerignore files tests............................................................

@test "check dev configs in .dockerignore | !**/.dockerized_norlab_project/ › expect pass" {
  helper::setup_ignore_related_tests '.dockerignore' '!**/.dockerized_norlab_project/' "${TESTED_FILE_PATH3}"
  assert_file_exist ".dockerignore"
  assert_file_contains ".dockerignore" "^\!\*\*\/\.dockerized_norlab_project\/"
}

@test "check dev configs in .dockerignore | !**/version.txt › expect pass" {
  helper::setup_ignore_related_tests '.dockerignore' '!**/version.txt' "${TESTED_FILE_PATH3}"
  assert_file_exist ".dockerignore"
  assert_file_contains ".dockerignore" "^\!\*\*\/version.txt"
}

@test "check dev configs in .dockerignore | !**/.git › expect pass" {
  helper::setup_ignore_related_tests '.dockerignore' '!**/.git' "${TESTED_FILE_PATH3}"
  assert_file_exist ".dockerignore"
  assert_file_contains ".dockerignore" "^\!\*\*\/\.git"
}

@test "check dev configs in .dockerignore | !**/external_data/ › expect pass" {
  helper::setup_ignore_related_tests '.dockerignore' '!**/external_data/' "${TESTED_FILE_PATH3}"
  assert_file_exist ".dockerignore"
  assert_file_contains ".dockerignore" "^\!\*\*\/external_data\/"
}

@test "check dev configs in .dockerignore | !**/artifact/ › expect pass" {
  helper::setup_ignore_related_tests '.dockerignore' '!**/artifact/' "${TESTED_FILE_PATH3}"
  assert_file_exist ".dockerignore"
  assert_file_contains ".dockerignore" "^\!\*\*\/artifact\/"
}

@test "check dev configs in .dockerignore | !/utilities/tmp/dockerized-norlab-project-mock/ › expect pass" {
  helper::setup_ignore_related_tests '.dockerignore' '!/utilities/tmp/dockerized-norlab-project-mock/' "${TESTED_FILE_PATH3}"
  assert_file_exist ".dockerignore"
  assert_file_contains ".dockerignore" "^\!\/utilities\/tmp\/dockerized-norlab-project-mock\/"
}

# ....DNP app .gitignore files tests...............................................................

@test "check dev configs in .gitignore | /utilities/tmp/dockerized-norlab-project-mock › expect pass" {
  helper::setup_ignore_related_tests '.gitignore' '/utilities/tmp/dockerized-norlab-project-mock' "${TESTED_FILE_PATH3}"
  assert_file_exist ".gitignore"
  assert_file_contains ".gitignore" "^\/utilities\/tmp\/dockerized-norlab-project-mock"
}

@test "check dev configs in .gitignore | !**/external_data/README.md › expect pass" {
  helper::setup_ignore_related_tests '.gitignore' '!**/external_data/README.md' "${TESTED_FILE_PATH3}"
  assert_file_exist ".gitignore"
  assert_file_contains ".gitignore" "^\!\*\*\/external_data\/README\.md"
}

@test "check dev configs in .gitignore | !**/artifact/README.md › expect pass" {
  helper::setup_ignore_related_tests '.gitignore' '!**/artifact/README.md' "${TESTED_FILE_PATH3}"
  assert_file_exist ".gitignore"
  assert_file_contains ".gitignore" "^\!\*\*\/artifact\/README\.md"
}

@test "check dev configs in .gitignore | !**/artifact/optuna_storage/README.md › expect pass" {
  helper::setup_ignore_related_tests '.gitignore' '!**/artifact/optuna_storage/README.md' "${TESTED_FILE_PATH3}"
  assert_file_exist ".gitignore"
  assert_file_contains ".gitignore" "^\!\*\*\/artifact\/optuna_storage\/README\.md"
}
