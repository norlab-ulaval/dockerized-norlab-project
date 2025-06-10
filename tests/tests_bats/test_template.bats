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

#TODO: setup the following variables: the script to test and its path
TESTED_FILE="dummy.bash"
TESTED_FILE_PATH="tests"

# ....Setup........................................................................................
# TODO: configure setup_file and setup function

setup_file() {
  echo -e "  Executed once before starting the first test (valide for all test in that file)" >&3
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR

  ## Uncomment the following for debug, the ">&3" is for printing bats msg to stdin
  #tree -L 1 -a -hug $(pwd) >&3
  #printenv >&3
}

setup() {
  echo -e "   ↳ Executed before each test" >&3
  cd "$TESTED_FILE_PATH" || exit 1
  source ./$TESTED_FILE
}

# ....Teardown.....................................................................................
# TODO: configure teardown and teardown_file function

teardown() {
  echo -e "   ↳ Executed after each test" >&3
  bats_print_run_env_variable_on_error
}

teardown_file() {
  echo -e "  Executed once after finishing the last test (valide for all test in that file)\n" >&3
}

# ====Test casses==================================================================================
# TODO: write tests cases

@test "dnp::good_morning_norlab (environment variable set) › expect pass" {
  assert_empty $GREETING
  export GREETING='Goooooooood morning NorLab'
  assert_not_empty $GREETING

  run "dnp::good_morning_norlab"
  assert_success
  assert_output --partial " ... there's nothing like the smell of a snow storm in the morning!"
  unset GREETING
}

@test "dnp::good_morning_norlab (command executed in a subshell) › expect pass" {
  assert_empty $GREETING
  run bash -c "source ./$TESTED_FILE && GREETING='Goooooooood morning NorLab' && dnp::good_morning_norlab"
  assert_empty $GREETING
  assert_success
  assert_output --partial "Goooooooood morning NorLab ... there's nothing like the smell of a snow storm in the morning!"
}


