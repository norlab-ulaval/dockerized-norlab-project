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
  echo -e "\n{error_prefix} $0 path to bats-core helper library unreachable at \"${bats_path}\"!"
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi

# ====Tests file configuration=====================================================================

TESTED_FILE="super_project_dnp_sanity_check.bash"
TESTED_FILE_PATH="src/lib/core/utils"

# executed once before starting the first test (valide for all test in that file)
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR

  export MOCK_PROJECT_PATH="${BATS_DOCKER_WORKDIR}/utilities/tmp/dockerized-norlab-project-mock"

#  tree -L 3 -a "${MOCK_PROJECT_PATH}" >&3
#  cat "${MOCK_PROJECT_PATH}/.git" >&3

#  # Uncomment the following for debug, the ">&3" is for printing bats msg to stdin
#  echo -e "\033[1;2m
#  \n...N2ST bats tests environment.................................................................
#  \n$( tree -L 1 -a -hug $PWD
#  cat .dockerignore
#  )
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
  cd "${MOCK_PROJECT_PATH}" || exit 1
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

# (!) Notes: the following tests assume `setup_host_for_running_this_super_project.bash` was already
#  executed via `test_setup_host_for_this_dnp_user_project.bats`.

@test "source $TESTED_FILE › expect pass" {
  assert_dir_exist "${MOCK_PROJECT_PATH}/.dockerized_norlab_project"
  assert_file_exist "${MOCK_PROJECT_PATH}/.dockerignore"

  run bash "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  assert_success
  assert_output --regexp "[DNP done].*Super project dockerized-norlab-project-mock setup is OK"
}

@test "dnp::super_project_dnp_sanity_check (source at setup step) › expect pass" {
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/load_super_project_config.bash" || exit 1

  assert_dir_exist "${MOCK_PROJECT_PATH}/.dockerized_norlab_project"
  assert_file_exist "${MOCK_PROJECT_PATH}/.dockerignore"

  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  run dnp::super_project_dnp_sanity_check
  assert_success
  assert_output --regexp "[DNP done].*Super project dockerized-norlab-project-mock setup is OK"
}

@test "dnp::super_project_dnp_sanity_check | DNP lib not loaded › expect fail" {
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  run dnp::super_project_dnp_sanity_check
  assert_failure
  assert_output --partial 'DNP libs are not loaded, run import_dnp_lib.bash first'
}
