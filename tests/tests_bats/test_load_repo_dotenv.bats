#!/usr/bin/env bats
# =================================================================================================
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
# =================================================================================================

TESTED_FILE="load_repo_dotenv.bash"

# ====Load ressources==============================================================================
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

# executed once before starting the first test (valide for all test in that file)
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR

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
#  cd "${MOCK_PROJECT_PATH}" || exit 1
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

# ====Test casses==================================================================================

@test "assess execute with \"source $TESTED_FILE\" › expect pass" {
  assert_not_exist "${PROJECT_PROMPT_NAME}"
  assert_not_exist "${PROJECT_PATH}"
  assert_not_exist "${DNP_PROMPT_NAME}"
  assert_not_exist "${DNP_GIT_REMOTE_URL}"
  assert_not_exist "${DNP_SRC_NAME}"
  assert_not_exist "${N2ST_PATH}"
  assert_not_exist "${NBS_PATH}"
  assert_not_exist "${DNP_ROOT}"
  assert_not_exist "${DNP_CONFIG_SCHEME_VERSION}"
  assert_not_exist "${DNP_LIB_EXEC_PATH}"
  assert_not_exist "${DNP_MOCK_SUPER_PROJECT_ROOT}"

  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE}"

#  printenv | grep -e DNP_ >&3

  assert_not_empty "${PROJECT_PROMPT_NAME}"
  assert_not_empty "${PROJECT_PATH}"
  assert_not_empty "${DNP_PROMPT_NAME}"
  assert_not_empty "${DNP_GIT_REMOTE_URL}"
  assert_not_empty "${DNP_SRC_NAME}"
  assert_not_empty "${N2ST_PATH}"
  assert_not_empty "${NBS_PATH}"
  assert_not_empty "${DNP_ROOT}"
  assert_not_empty "${DNP_CONFIG_SCHEME_VERSION}"
  assert_not_empty "${DNP_LIB_EXEC_PATH}"
  assert_not_empty "${DNP_MOCK_SUPER_PROJECT_ROOT}"

  assert_equal "${PROJECT_PROMPT_NAME}" "DNP"
  assert_equal "${DNP_PROMPT_NAME}" "DNP"
  assert_equal "${PROJECT_PATH}" "${BATS_DOCKER_WORKDIR}"
  assert_equal "${DNP_GIT_REMOTE_URL}" "https://github.com/norlab-ulaval/dockerized-norlab-project"
  assert_equal "${DNP_SRC_NAME}" "dockerized-norlab-project"
  assert_equal "${N2ST_PATH}" "${BATS_DOCKER_WORKDIR}/utilities/norlab-shell-script-tools"
  assert_equal "${NBS_PATH}" "${BATS_DOCKER_WORKDIR}/utilities/norlab-build-system"
  assert_equal "${DNP_LIB_EXEC_PATH}" "${BATS_DOCKER_WORKDIR}/src/lib/core/execute"
  assert_equal "${DNP_MOCK_SUPER_PROJECT_ROOT}" "${BATS_DOCKER_WORKDIR}/utilities/tmp/dockerized-norlab-project-mock"

}

@test "assess execute from arbitrary location with \"source $TESTED_FILE\"  › expect pass" {
  cd  src
  assert_equal "$(pwd)" "${BATS_DOCKER_WORKDIR}/src"
  run source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE}"
  assert_equal "$(pwd)" "${BATS_DOCKER_WORKDIR}/src" # Validate that it returned to the original dir
  assert_success
}

@test "assess execute feedback with \"source $TESTED_FILE\"  › expect pass" {
  run source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE}" --debug

  assert_success
  assert_output --regexp "[DNP]".*".env.dockerized-norlab-project loaded"
}

@test "assess execute with \"bash $TESTED_FILE\" › expect fail" {
  run bash "${BATS_DOCKER_WORKDIR}/${TESTED_FILE}"

  assert_failure
  assert_output --regexp "[DNP error]".*"This script must be sourced i.e.:".*"source".*"$TESTED_FILE"
}


