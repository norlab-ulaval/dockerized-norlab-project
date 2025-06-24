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

TESTED_FILE="import_dna_lib.bash"
TESTED_FILE_PATH="src/lib/core/utils"

# executed once before starting the first test (valide for all test in that file)
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR

  # This is the path to the mock super project (the user side)
  export MOCK_PROJECT_PATH="${BATS_DOCKER_WORKDIR}/utilities/tmp/dockerized-norlab-project-mock"

  ## Uncomment the following for debug, the ">&3" is for printing bats msg to stdin
  #echo -e "\033[1;2m
  #...N2ST bats tests environment.................................................................
  #\n$(
  #  echo -e "\nfind"
  #  find / -type d -name "src" -exec tree -L 2 -a {} \;
  #  echo
  #  tree -L 2 -a -I .git $PWD
  #)
  #\n...............................................................................................
  #\033[0m"  >&3
  #
  #echo -e "
  #\n...DNA related environment varaibles...........................................................
  #\n$(printenv | grep -e DNA_)
  #\n...............................................................................................
  #\n" >&3

}

# executed before each test
setup() {
  # Change cwd to the mock super project directory
  cd "${MOCK_PROJECT_PATH}" || exit 1
}

# ====Teardown=====================================================================================

# executed after each test
teardown() {
  bats_print_run_env_variable_on_error
}

## executed once after finishing the last test (valide for all test in that file)
#teardown_file() {
#  echo "We are done" >&3
#}

# ====Test casses==================================================================================

@test "dna::import_lib_and_dependencies (explicitly source $TESTED_FILE) › expect pass" {
  assert_not_exist "${N2ST_PATH}"
  assert_not_exist "${NBS_PATH}"
  assert_not_exist "${DNA_ROOT}"

  assert_equal "$(pwd)" "${MOCK_PROJECT_PATH}"

  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"

  assert_equal "$(pwd)" "${MOCK_PROJECT_PATH}" # Validate that it returned to the original dir

#  printenv | grep -e DNA_ >&3

  assert_not_empty "${DNA_ROOT}"
  assert_not_empty "${N2ST_PATH}"
  assert_not_empty "${NBS_PATH}"

  run n2st::norlab_splash
  assert_success

}

@test "assess execute with \"source $TESTED_FILE\"  › expect pass" {
  run source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" --debug

  assert_success
  assert_output --regexp "[DNA done]".*"librairies loaded"
}

@test "assess execute with \"bash $TESTED_FILE\" › expect fail" {
  run bash "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"

  assert_failure
  assert_output --regexp "[DNA error]".*"This script must be sourced i.e.:".*"source".*"$TESTED_FILE"
}


