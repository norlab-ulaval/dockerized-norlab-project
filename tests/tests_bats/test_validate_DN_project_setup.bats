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
TESTED_FILE="validate_super_project_dnp_setup.bash"
TESTED_FILE_PATH="src/lib/core/utils"

# executed once before starting the first test (valide for all test in that file)
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR

  export DNP_MOCK_PROJECT_PATH="${BATS_DOCKER_WORKDIR}/dockerized-norlab-project-mock"

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
setup() {
  cd "${DNP_MOCK_PROJECT_PATH}" || exit 1
}

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

# (!) Notes: the following tests assume `setup_host_for_this_super_project.bash` was already
#  executed via `test_setup_host_for_this_dnp_user_project.bats`.

@test "source $TESTED_FILE › expect pass" {
  assert_dir_exist "${DNP_MOCK_PROJECT_PATH}/.dockerized_norlab_project"
  assert_file_exist "${DNP_MOCK_PROJECT_PATH}/.dockerignore"

  run bash -c "source ${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  assert_success
  assert_output --regexp "[DNP done].*Super project dockerized-norlab-project-mock setup is OK"
}

@test "dnp::validate_super_project_dnp_setup (source at setup step) › expect pass" {
  assert_dir_exist "${DNP_MOCK_PROJECT_PATH}/.dockerized_norlab_project"
  assert_file_exist "${DNP_MOCK_PROJECT_PATH}/.dockerignore"

  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  run dnp::validate_super_project_dnp_setup
  assert_success
  assert_output --regexp "[DNP done].*Super project dockerized-norlab-project-mock setup is OK"
}

