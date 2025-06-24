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

TESTED_FILE="super_project_dnp_sanity_check.bash"
TESTED_FILE_PATH="src/lib/core/utils"

# executed once before starting the first test (valide for all test in that file)
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR

  # This is the path to the mock super project (the user side)
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

  # Create a temporary directory for test files
  export TEST_TEMP_DIR=$(temp_make)
}

# executed before each test
setup() {
  # Change cwd to the mock super project directory
  cd "${MOCK_PROJECT_PATH}" || exit 1

  # Source the required files for testing
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" || exit 1

  # Set up environment variables for testing
  export SUPER_PROJECT_ROOT="${MOCK_PROJECT_PATH}"
  export SUPER_PROJECT_REPO_NAME="dockerized-norlab-project-mock"

}

# ====Teardown=====================================================================================

# executed after each test
teardown() {
  bats_print_run_env_variable_on_error
}

# executed once after finishing the last test (valide for all test in that file)
teardown_file() {
  # Clean up temporary directory
  if [[ -n "${TEST_TEMP_DIR}" && -d "${TEST_TEMP_DIR}" ]]; then
    temp_del "${TEST_TEMP_DIR}"
  fi
}

# ====Test casses==================================================================================

# ....Test script dependencies logic...............................................................
@test "dnp::super_project_dnp_sanity_check (sourced) › expect pass" {
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/load_super_project_config.bash" || exit 1

  assert_exist "${DNP_ROOT}"
  assert_not_empty "${DNP_ROOT}"
  assert_dir_exist "${DNP_ROOT}"
  assert_dir_exist "${MOCK_PROJECT_PATH}/.dockerized_norlab"
  assert_file_exist "${MOCK_PROJECT_PATH}/.dockerignore"

#  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  run dnp::super_project_dnp_sanity_check
  assert_success
  assert_output --regexp "[DNP done].*Super project dockerized-norlab-project-mock setup is OK"
}

@test "dnp::super_project_dnp_sanity_check (sourced) but missing super project | super project unreachable › expect fail" {
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1
  assert_exist "${DNP_ROOT}"
  assert_not_empty "${DNP_ROOT}"
  assert_dir_exist "${DNP_ROOT}"

  unset SUPER_PROJECT_ROOT

  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  run dnp::super_project_dnp_sanity_check
  assert_failure
  assert_output --partial 'Super project configs are not loaded, run load_super_project_config.bash first'
}

# ....Test individual check functions...............................................................
@test "dnp::check_super_project_dir_structure › expect pass with valid structure" {
  # Test case: When the super project has a valid directory structure, the function should pass
  run dnp::check_super_project_dir_structure
  assert_success
}

@test "dnp::check_super_project_dir_structure › expect fail with missing .dockerized_norlab" {
  # Test case: When .dockerized_norlab is missing, the function should fail
  # Create a temporary directory with an incomplete structure
  mkdir -p "${TEST_TEMP_DIR}"
  cd "${TEST_TEMP_DIR}" || exit 1

  run dnp::check_super_project_dir_structure
  assert_failure
  assert_output --partial "'.dockerized_norlab' is not installed at super-project repository root"
}

@test "dnp::check_super_project_dir_structure › expect fail with missing src directory" {
  # Test case: When src directory is missing, the function should fail
  # Create a temporary directory with an incomplete structure
  mkdir -p "${TEST_TEMP_DIR}/.dockerized_norlab"
  cd "${TEST_TEMP_DIR}" || exit 1

  run dnp::check_super_project_dir_structure
  assert_failure
  assert_output --partial "The 'src' directory is not installed at super-project repository root"
}

@test "dnp::check_dockerized_project_configuration_dir_structure › expect pass with valid structure" {
  # Test case: When the .dockerized_norlab directory has a valid structure, the function should pass
  run dnp::check_dockerized_project_configuration_dir_structure
  assert_success
}

@test "dnp::check_dockerized_project_configuration_dir_structure › expect fail with missing configuration directory" {
  # Test case: When configuration directory is missing, the function should fail
  # Create a temporary directory with an incomplete structure
  mkdir -p "${TEST_TEMP_DIR}/.dockerized_norlab"

  # Set up environment for the test
  export SUPER_PROJECT_ROOT="${TEST_TEMP_DIR}"
  cd "${TEST_TEMP_DIR}" || exit 1

  run dnp::check_dockerized_project_configuration_dir_structure
  assert_failure
  assert_output --partial "The '.dockerized_norlab/configuration/' directory is not installed"
}

@test "dnp::check_project_configuration › expect pass with valid configuration" {
  # Test case: When the project configuration is valid, the function should pass
  run dnp::check_project_configuration
  assert_success
}

@test "dnp::check_project_configuration › expect fail with missing project_requirements directory" {
  # Test case: When project_requirements directory is missing, the function should fail
  # Create a temporary directory with an incomplete structure
  mkdir -p "${TEST_TEMP_DIR}/.dockerized_norlab/configuration"

  # Set up environment for the test
  export SUPER_PROJECT_ROOT="${TEST_TEMP_DIR}"
  cd "${TEST_TEMP_DIR}" || exit 1

  run dnp::check_project_configuration
  assert_failure
  assert_output --partial "The '.dockerized_norlab/configuration/project_requirements/' directory is not installed"
}

@test "dnp::check_project_entrypoints › expect pass with valid entrypoints" {
  # Test case: When the project entrypoints are valid, the function should pass
  run dnp::check_project_entrypoints
  assert_success
}

@test "dnp::check_project_entrypoints › expect fail with missing project-ci-tests directory" {
  # Test case: When project-ci-tests directory is missing, the function should fail
  # Create a temporary directory with an incomplete structure
  mkdir -p "${TEST_TEMP_DIR}/.dockerized_norlab/configuration/project_entrypoints"

  # Set up environment for the test
  export SUPER_PROJECT_ROOT="${TEST_TEMP_DIR}"
  cd "${TEST_TEMP_DIR}" || exit 1

  run dnp::check_project_entrypoints
  assert_failure
  assert_output --partial "The '.dockerized_norlab/configuration/project_entrypoints/project-ci-tests/' directory is not installed"
}

@test "dnp::check_gitignore › expect pass with valid gitignore entries" {
  # Test case: When the .gitignore file has all required entries, the function should pass
  run dnp::check_gitignore
  assert_success
}

@test "dnp::check_gitignore › expect fail with missing required entries" {
  # Test case: When .gitignore is missing required entries, the function should fail
  # Create a temporary directory with an incomplete .gitignore
  mkdir -p "${TEST_TEMP_DIR}"
  echo "# Test .gitignore" > "${TEST_TEMP_DIR}/.gitignore"

  # Set up environment for the test
  export SUPER_PROJECT_ROOT="${TEST_TEMP_DIR}"
  cd "${TEST_TEMP_DIR}" || exit 1

  run dnp::check_gitignore
  assert_failure
  assert_output --partial "The line '**/.dockerized_norlab/dn_container_env_variable/' is not present in .gitignore"
}

@test "dnp::check_dockerignore › expect pass with valid dockerignore entries" {
  # Test case: When the .dockerignore file has all required entries, the function should pass
  run dnp::check_dockerignore
  assert_success
}

@test "dnp::check_dockerignore › expect fail with missing required entries" {
  # Test case: When .dockerignore is missing required entries, the function should fail
  # Create a temporary directory with an incomplete .dockerignore
  mkdir -p "${TEST_TEMP_DIR}"
  echo "# Test .dockerignore" > "${TEST_TEMP_DIR}/.dockerignore"

  # Set up environment for the test
  export SUPER_PROJECT_ROOT="${TEST_TEMP_DIR}"
  cd "${TEST_TEMP_DIR}" || exit 1

  run dnp::check_dockerignore
  assert_failure
  assert_output --partial "The line '!**/.dockerized_norlab/' is not present in .dockerignore"
}

# ....Test integration of dnp::super_project_dnp_sanity_check.....................................
@test "dnp::super_project_dnp_sanity_check integration › expect pass with valid super project" {
  # Test case: When the super project is valid, the function should pass without mocking any check functions

  # Ensure we're in the mock project directory
  cd "${MOCK_PROJECT_PATH}" || exit 1

  # Source the required files
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/load_super_project_config.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" || exit 1

  # Run the function without mocking any check functions
  run dnp::super_project_dnp_sanity_check

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --regexp "[DNP done].*Super project dockerized-norlab-project-mock setup is OK"
}

@test "dnp::super_project_dnp_sanity_check integration › expect fail with invalid super project" {
  # Test case: When the super project is invalid, the function should fail without mocking any check functions

  # Create a temporary directory with an incomplete structure
  mkdir -p "${TEST_TEMP_DIR}/.dockerized_norlab"

  # Set up environment for the test
  export SUPER_PROJECT_ROOT="${TEST_TEMP_DIR}"
  export SUPER_PROJECT_REPO_NAME="test-project"
  cd "${TEST_TEMP_DIR}" || exit 1

  # Mock git command to avoid actual git operations
  function git() {
    if [[ "$1" == "-C" && "$3" == "rev-parse" ]]; then
      echo "true"
      return 0
    elif [[ "$1" == "status" ]]; then
      return 0
    else
      command git "$@"
    fi
  }
  export -f git

  # Source the required files
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" || exit 1

  # Run the function without mocking any check functions
  run dnp::super_project_dnp_sanity_check

  # Should fail
  assert_failure

  # Should output an error message
  assert_output --regexp  "[DNP error].* The 'src' directory is not installed at super-project repository root.*"
}

# ....Test script business logic...................................................................
@test "run $TESTED_FILE › expect pass" {
  run bash "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  assert_success
  assert_output --regexp "[DNP done].*Super project dockerized-norlab-project-mock setup is OK"
}
