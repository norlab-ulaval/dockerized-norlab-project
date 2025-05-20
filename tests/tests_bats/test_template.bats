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

#TODO: setup the following variable
TESTED_FILE="dummy.bash"
TESTED_FILE_PATH="src"

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
  source ./$TESTED_FILE
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

@test "test me like a boss" {
  echo "TODO: write some test"
}

@test 'fail()' {
  skip "Comment this line to run this test"
  fail 'this test always fails'
}

@test "n2st::this_is_not_cool (explicitly source $TESTED_FILE) › expect fail" {
  run bash -c "source ./$TESTED_FILE && n2st::this_is_not_cool"
  assert_failure 1
  assert_output --partial 'Noooooooooooooo!'
}

@test "n2st::this_is_not_cool (source at setup step) › expect fail" {
  run n2st::this_is_not_cool
  assert_failure 1
  assert_output --partial 'Noooooooooooooo!'
}

@test "n2st::good_morning_norlab (environment variable not set) › expect fail" {
  run "n2st::good_morning_norlab"
  assert_failure 1
  assert_output --partial 'Error: Environment variable not set'
  unset GREETING
}

@test "n2st::good_morning_norlab (environment variable set) › expect pass" {
  assert_empty $GREETING
  export GREETING='Goooooooood morning NorLab'
  assert_not_empty $GREETING

  run "n2st::good_morning_norlab"
  assert_success
  assert_output --partial " ... there's nothing like the smell of a snow storm in the morning!"
  unset GREETING
}

@test "n2st::good_morning_norlab (command executed in a subshell) › expect pass" {
  assert_empty $GREETING
  run bash -c "source ./$TESTED_FILE && GREETING='Goooooooood morning NorLab' && n2st::good_morning_norlab"
  assert_empty $GREETING
  assert_success
  assert_output --partial "Goooooooood morning NorLab ... there's nothing like the smell of a snow storm in the morning!"
}

@test "n2st::talk_to_me_or_not › expect fail" {
  # Note:
  #  - "echo 'Y'" is for sending an keyboard input to the 'read' command which expect a single character
  #    run bash -c "echo 'Y' | source ./function_library/$TESTED_FILE"
  #  - Alt: Use the 'yes [n]' command which optionaly send n time
#  run bash -c "yes 1 | n2st::talk_to_me_or_not"
  run bash -c "source ./$TESTED_FILE && yes 1 | n2st::talk_to_me_or_not"
  assert_failure 1
  assert_output --partial '(press any key to exit)'
}


