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
  #load "${BATS_HELPER_PATH}/bats-detik/load" # << Kubernetes support
else
  echo -e "\n[\033[1;31mERROR\033[0m] $0 path to bats-core helper library unreachable at \"${BATS_HELPER_PATH}\"!" 1>&2
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi

# ====Setup========================================================================================

setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
#  pwd >&3 && tree -L 1 -a -hug >&3
#  printenv >&3
}

#setup() {
#}

# ====Teardown=====================================================================================

teardown() {
  bats_print_run_env_variable_on_error
#  printenv
}

#teardown_file() {
#    echo "executed once after finishing the last test"
#}

# ====Test casses==================================================================================
# Livetemplate shortcut: @test

function _source_dotenv() {
  local DOTENV_FILE="$1"
  set -o allexport
  # shellcheck disable=SC1090
  source "${BATS_DOCKER_WORKDIR:?err}/${DOTENV_FILE}"
  set +o allexport
}


function source_dotenv_npt() {
  _source_dotenv ".env.template-norlab-project.template"
}

# ----.env.n2st------------------------------------------------------------------------------------
@test ".env.n2st › Env variables set ok" {

  # ....Pre-condition..............................................................................
  assert_empty ${PROJECT_PROMPT_NAME}
  assert_empty ${PROJECT_GIT_REMOTE_URL}
  assert_empty ${PROJECT_GIT_NAME}
  assert_empty ${PROJECT_SRC_NAME}
  assert_empty ${PROJECT_PATH}

  assert_empty ${PLACEHOLDER_PROMPT_NAME}
  assert_empty ${PLACEHOLDER_GIT_REMOTE_URL}
  assert_empty ${PLACEHOLDER_GIT_NAME}
  assert_empty ${PLACEHOLDER_SRC_NAME}
  assert_empty ${PLACEHOLDER_PATH}

  assert_empty ${N2ST_PATH}

  # ....Source .env.project........................................................................
  source_dotenv_npt
#  printenv | grep -e 'CONTAINER_PROJECT_' -e 'PROJECT_' >&3

  # ....Tests......................................................................................
  assert_equal "${PROJECT_PROMPT_NAME}" "Norlab-Project-Template"
  assert_regex "${PROJECT_GIT_REMOTE_URL}" "https://github.com/norlab-ulaval/template-norlab-project"'(".git")?'
  assert_equal "${PROJECT_GIT_NAME}" "template-norlab-project"
  assert_equal "${PROJECT_SRC_NAME}" "template-norlab-project"

  assert_equal "${PLACEHOLDER_PROMPT_NAME}" "Norlab-Project-Template"
  assert_regex "${PLACEHOLDER_GIT_REMOTE_URL}" "https://github.com/norlab-ulaval/template-norlab-project"'(".git")?'
  assert_equal "${PLACEHOLDER_GIT_NAME}" "template-norlab-project"
  assert_equal "${PLACEHOLDER_SRC_NAME}" "template-norlab-project"
  assert_equal "${PLACEHOLDER_PATH}" "/code/template-norlab-project"

  assert_equal "${N2ST_PATH}" "/code/template-norlab-project/utilities/norlab-shell-script-tools"

}

