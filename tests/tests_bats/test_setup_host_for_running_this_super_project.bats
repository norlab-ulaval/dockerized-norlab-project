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

TESTED_FILE="setup_host_for_running_this_super_project.bash"
TESTED_FILE_PATH="src/lib/core/utils"

# executed once before starting the first test (valide for all test in that file)
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR

  # This is the path to the mock super project (the user side)
  export MOCK_PROJECT_PATH="${BATS_DOCKER_WORKDIR}/utilities/tmp/dockerized-norlab-project-mock"

#  # Uncomment the following for debug, the ">&3" is for printing bats msg to stdin
#  echo -e "\033[1;2m
#  \n...N2ST bats tests environment.................................................................
#  \n$( tree -L 1 -a -hug $PWD && printenv )
#  \n...............................................................................................
#  \033[0m"  >&3
#
#  echo -e "
#  \n...DNA related environment varaibles...........................................................
#  \n$(printenv | grep -e DNA_)
#  \n...............................................................................................
#  \n" >&3

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
#
#}

# ====Test casses==================================================================================

@test "dna::setup_host_for_this_super_project (test .bashrc expected append › expect pass" {
  T_DN_PROJECT_ALIAS_PREFIX="umock"
  T_DN_PROJECT_ALIAS_PREFIX_CAP="UMOCK"
#  cat ${HOME}/.bashrc

  assert_file_not_contains "${HOME}/.bashrc" "#>>>>DNA dockerized-norlab-project-mock aliases and env variable"
  assert_file_not_contains "${HOME}/.bashrc" "#<<<<DNA dockerized-norlab-project-mock aliases and env variable end"

  run bash "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  assert_success

  assert_file_contains "${HOME}/.bashrc" "^#>>>>DNA dockerized-norlab-project-mock aliases and env variable$"
  assert_file_contains "${HOME}/.bashrc" "^export _DNA_${T_DN_PROJECT_ALIAS_PREFIX_CAP}_PATH=${MOCK_PROJECT_PATH}/.dockerized_norlab$"
  assert_file_contains "${HOME}/.bashrc" "^alias dna-${T_DN_PROJECT_ALIAS_PREFIX}-cd='cd ${MOCK_PROJECT_PATH}'$"
  assert_file_contains "${HOME}/.bashrc" "^alias dna-${T_DN_PROJECT_ALIAS_PREFIX}-cdd='cd ${MOCK_PROJECT_PATH}/.dockerized_norlab'$"
  assert_file_contains "${HOME}/.bashrc" "^alias dna-${T_DN_PROJECT_ALIAS_PREFIX}-cds='cd ${MOCK_PROJECT_PATH}/src'$"
  assert_file_contains "${HOME}/.bashrc" "^alias dna-${T_DN_PROJECT_ALIAS_PREFIX}-cdt='cd ${MOCK_PROJECT_PATH}/tests'$"
  assert_file_contains "${HOME}/.bashrc" "^alias dna-${T_DN_PROJECT_ALIAS_PREFIX}-cda='cd ${MOCK_PROJECT_PATH}/artifact'$"
  assert_file_contains "${HOME}/.bashrc" "^alias dna-${T_DN_PROJECT_ALIAS_PREFIX}-cde='cd ${MOCK_PROJECT_PATH}/external_data'$"
  assert_file_contains "${HOME}/.bashrc" "^#<<<<DNA dockerized-norlab-project-mock aliases and env variable end$"

  assert_output --partial "Adding project aliases to .bashrc..."

  assert_output --partial "Setup completed!

    New available alias added to ~/.bashrc:
      - dna-${T_DN_PROJECT_ALIAS_PREFIX}-cd  -> cd to dockerized-norlab-project-mock root
      - dna-${T_DN_PROJECT_ALIAS_PREFIX}-cdd -> cd to dockerized-norlab-project-mock .dockerized_norlab dir
      - dna-${T_DN_PROJECT_ALIAS_PREFIX}-cds -> cd to dockerized-norlab-project-mock src dir
      - dna-${T_DN_PROJECT_ALIAS_PREFIX}-cdt -> cd to dockerized-norlab-project-mock tests dir
      - dna-${T_DN_PROJECT_ALIAS_PREFIX}-cda -> cd to dockerized-norlab-project-mock artifact dir
      - dna-${T_DN_PROJECT_ALIAS_PREFIX}-cde -> cd to dockerized-norlab-project-mock external data dir

    New available environment variable added to ~/.bashrc for convenience:
      - _DNA_${T_DN_PROJECT_ALIAS_PREFIX_CAP}_PATH=${MOCK_PROJECT_PATH}"
}

@test "dna::setup_host_for_this_super_project › expect pass" {
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dna_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/load_super_project_config.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  run dna::setup_host_for_this_super_project
  assert_success
  assert_output --partial 'Adding project aliases to .bashrc...'
  assert_output --partial 'Setup completed!'
  assert_not_empty "${DN_PROJECT_ALIAS_PREFIX}"
}

@test "dna::setup_host_for_this_super_project | DNA lib not loaded › expect fail" {
  run bash -c "source ${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE} && dna::setup_host_for_this_super_project"
  assert_failure
  assert_output --partial 'The DNA lib is not loaded'
}

@test "dna::setup_host_for_this_super_project | super project not loaded › expect fail" {
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dna_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  run dna::setup_host_for_this_super_project
  assert_failure
  assert_output --partial 'Super project configs are not loaded, run load_super_project_config.bash first'
}


