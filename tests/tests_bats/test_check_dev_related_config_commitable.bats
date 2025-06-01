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
TESTED_FILE_PATH2=".dockerized_norlab_project/configuration/"

# executed once before starting the first test (valide for all test in that file)
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
  export MOCK_PROJECT_PATH="${BATS_DOCKER_WORKDIR}/utilities/tmp/dockerized-norlab-project-mock"

#  # Uncomment the following for debug, the ">&3" is for printing bats msg to stdin
#  echo -e "\033[1;2m
#  \n...N2ST bats tests environment.................................................................
#  \n$( pwd && tree -L 1 -a -hug && printenv )
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
  cd "${TESTED_FILE_PATH}" || exit 1
  echo -e "\n\n
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
[DN ERROR] -> docker-compose yaml file misconfiguration error.

A DN_ENTRYPOINT_TRACE_EXECUTION environment variable in a docker-compose yaml is set to true i.e.,

environment:
  DN_ENTRYPOINT_TRACE_EXECUTION: true

Its OK for developement but MAKE SURE ITS SET TO FALSE for PUSH TO CI BUILD.
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
\n"
}

function helper::setup_dotenv_related_tests() {
  TESTED_FILE=$1
  TESTED_ENV_VAR=$2
  EXPECTED_VALUE=$3
  cd "${MOCK_PROJECT_PATH}/${TESTED_FILE_PATH2}" || exit 1
  echo -e "\n\n
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
[DN ERROR] -> dotenv file ${TESTED_FILE} misconfiguration error.
Path: ${MOCK_PROJECT_PATH}/${TESTED_FILE_PATH2}${TESTED_FILE}

Actual:
  $( cat "${TESTED_FILE}" | grep -e "^${TESTED_ENV_VAR}" )

Expected:
  ${TESTED_ENV_VAR}=${EXPECTED_VALUE}

Its OK for developement but MAKE SURE ITS SET TO THE EXPECTED VALUE for PUSH TO CI BUILD.
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
\n"
}

# ====Test casses==================================================================================

# ....docker-compose tests.........................................................................
@test "check dev configs are muted in docker-compose.project.build.native.yaml › expect pass" {
  helper::setup_compose_related_tests
  assert_file_exist "docker-compose.project.build.native.yaml"
  assert_file_not_contains "docker-compose.project.build.native.yaml" "DN_ENTRYPOINT_TRACE_EXECUTION: true"
}

@test "check dev configs are muted in docker-compose.project.run.darwin.yaml › expect pass" {
  helper::setup_compose_related_tests
  assert_file_exist "docker-compose.project.run.darwin.yaml"
  assert_file_not_contains "docker-compose.project.run.darwin.yaml" "DN_ENTRYPOINT_TRACE_EXECUTION: true"
}

@test "check dev configs are muted in docker-compose.project.run.jetson.yaml › expect pass" {
  helper::setup_compose_related_tests
  assert_file_exist "docker-compose.project.run.jetson.yaml"
  assert_file_not_contains "docker-compose.project.run.jetson.yaml" "DN_ENTRYPOINT_TRACE_EXECUTION: true"
}

@test "check dev configs are muted in docker-compose.project.run.linux-x86.yaml › expect pass" {
  helper::setup_compose_related_tests
  assert_file_exist "docker-compose.project.run.linux-x86.yaml"
  assert_file_not_contains "docker-compose.project.run.linux-x86.yaml" "DN_ENTRYPOINT_TRACE_EXECUTION: true"
}

@test "check dev configs are muted in docker-compose.project.run.slurm.yaml › expect pass" {
  helper::setup_compose_related_tests
  assert_file_exist "docker-compose.project.run.slurm.yaml"
  assert_file_not_contains "docker-compose.project.run.slurm.yaml" "DN_ENTRYPOINT_TRACE_EXECUTION: true"
}

# ....dotenv tests.................................................................................
@test "check dev configs in .env.dnp | DN_GIT_BRANCH=dev › expect pass" {
  helper::setup_dotenv_related_tests '.env.dnp' 'DN_GIT_BRANCH' 'dev'
  assert_file_exist ".env.dnp"
  assert_file_contains ".env.dnp" "^DN_GIT_BRANCH=dev"
}

@test "check dev configs in .env.dnp | DN_PROJECT_DEPLOY_REPO_BRANCH=dev › expect pass" {
  helper::setup_dotenv_related_tests '.env.dnp' 'DN_PROJECT_DEPLOY_REPO_BRANCH' 'dev'
  assert_file_exist ".env.dnp"
  assert_file_contains ".env.dnp" "^DN_PROJECT_DEPLOY_REPO_BRANCH=dev"
}

