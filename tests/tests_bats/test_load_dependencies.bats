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

#TODO: setup the following variable
TESTED_FILE="load_dependencies.bash"
TESTED_FILE_PATH=".dockerized_norlab_project/utilities"

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

  ## Uncomment the following for debug, the ">&3" is for printing bats msg to stdin
#  pwd >&3 && tree -L 1 -a -hug >&3
#  printenv >&3
}

# executed before each test
setup() {
  cd "$TESTED_FILE_PATH" || exit 1
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

@test "dnp::source_project_shellscript_dependencies (explicitly source $TESTED_FILE) › expect pass" {
  assert_not_exist "${SUPER_PROJECT_ROOT}"
  assert_not_exist "${N2ST_PATH}"
  assert_not_exist "${NBS_PATH}"
  assert_not_exist "${DN_PROJECT_HUB}"

  assert_equal "$(pwd)" "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}"

  source ./$TESTED_FILE

  assert_equal "$(pwd)" "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}" # Validate that it returned to the original

  assert_not_empty "${SUPER_PROJECT_ROOT}"
  assert_not_empty "${N2ST_PATH}"
  assert_not_empty "${NBS_PATH}"
  assert_not_empty "${DN_PROJECT_HUB}"

  run n2st::norlab_splash
  assert_success

}

@test "assess execute with \"bash $TESTED_FILE\" › expect fail" {
  # ....Import N2ST library........................................................................
  run bash "$TESTED_FILE"

  # ....Tests......................................................................................
  assert_failure
  assert_output --regexp "[ERROR]".*"This script must be sourced i.e.:".*"source".*"$TESTED_FILE"
}


