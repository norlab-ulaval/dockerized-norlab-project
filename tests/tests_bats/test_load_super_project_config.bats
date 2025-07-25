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

TESTED_FILE="load_super_project_config.bash"
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
#  \n...DN/DNA/SUPER related environment varaibles..................................................
#  \n$( printenv | grep -e DN_ -e DNA_ -e SUPER_ )
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

@test "explicitly source $TESTED_FILE › expect pass" {
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/import_dna_lib.bash" || exit 1
  assert_exist "${DNA_ROOT}"
  assert_not_empty "${DNA_ROOT}"

  assert_not_exist "${SUPER_PROJECT_ROOT}"
  assert_not_exist "${SUPER_PROJECT_REPO_NAME}"
  assert_not_exist "${DN_PROJECT_GIT_NAME}"
  assert_not_exist "${DN_PROJECT_GIT_REMOTE_URL}"
  assert_not_exist "${DN_PROJECT_HUB}"
  assert_not_exist "${DNA_URL}"
  assert_not_exist "${DN_PROJECT_USER}"
  assert_not_exist "${HYDRA_FULL_ERROR}"
  assert_not_exist "${MOCK_TEST_WAS_LOADED}"

  assert_equal "$(pwd)" "${MOCK_PROJECT_PATH}"



  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"

  echo -e "
  \n...DN/DNA/SUPER related environment varaibles..................................................
  \n$( printenv | grep -e DN_ -e DNA_ -e SUPER_ )
  \n...............................................................................................
  \n"

  assert_equal "$(pwd)" "${MOCK_PROJECT_PATH}" # Validate that it returned to the original dir

  assert_equal "${SUPER_PROJECT_ROOT}" "${MOCK_PROJECT_PATH}"
  assert_equal "${SUPER_PROJECT_REPO_NAME}" "dockerized-norlab-project-mock"
  assert_equal "${DN_PROJECT_GIT_NAME}" "dockerized-norlab-project-mock"
  assert_equal "${DN_PROJECT_GIT_REMOTE_URL}" "https://github.com/norlab-ulaval/dockerized-norlab-project-mock.git"
  assert_equal "${DN_PROJECT_HUB}" "norlabulaval"

  assert_equal "${DN_PROJECT_USER}" 'root'
  assert_equal "${DNA_CONFIG_SCHEME_VERSION}" 1
  assert_equal "${MOCK_TEST_WAS_LOADED}" 1
  assert_equal "${HYDRA_FULL_ERROR}" 1


}

@test "assess execute with \"source $TESTED_FILE\" › expect pass" {
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/import_dna_lib.bash" || exit 1
  run source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" --debug

  assert_success
  assert_output --regexp "[DNA done]".*"dockerized-norlab-project-mock project configurations loaded"
}


@test "assess execute with \"source $TESTED_FILE\" but miss requirement › expect fail" {
#  export N2ST_PATH="$BATS_DOCKER_WORKDIR/utilities/norlab-shell-script-tools"
  run source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" --debug

  assert_failure
  assert_output --regexp "[DNA error]".*"The N2ST lib is not loaded!"
}


@test "assess execute with \"bash $TESTED_FILE\" › expect fail" {
  run bash "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"

  assert_failure
  assert_output --regexp "[DNA error]".*"This script must be sourced i.e.:".*"source".*"$TESTED_FILE"
}

# ====Tests for dna::check_offline_deploy_service_discovery function==============================

@test "dna::check_offline_deploy_service_discovery with valid meta.txt and loaded image › expect success" {
  # Setup: Load DNA lib and source the tested file
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/import_dna_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"

  # Create temporary directory for test
  local test_dir
  test_dir=$(temp_make)

  # Create mock meta.txt file
  cat > "${test_dir}/meta.txt" << 'EOF'
SERVICE=deploy
IMAGE_NAME=test-image-deploy.latest
SUPER_PROJECT_REPO_NAME=test-project
DN_PROJECT_ALIAS_PREFIX=test
TAR_FILENAME=test-image-deploy.latest.tar
EOF

  # Mock docker command to simulate image being loaded
  function docker() {
    case "$1" in
      "image")
        case "$2" in
          "ls")
            echo "test-image-deploy.latest"
            return 0
            ;;
          *)
            return 0
            ;;
        esac
        ;;
      *)
        return 0
        ;;
    esac
  }
  export -f docker

  # Change to test directory and run function
  cd "${test_dir}" || exit 1
  run dna::check_offline_deploy_service_discovery

  assert_success
  assert_output "deploy"

  # Cleanup
  temp_del "${test_dir}"
}

@test "dna::check_offline_deploy_service_discovery with valid meta.txt but image not loaded › expect failure" {
  # Setup: Load DNA lib and source the tested file
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/import_dna_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"

  # Create temporary directory for test
  local test_dir
  test_dir=$(temp_make)

  # Create mock meta.txt file
  cat > "${test_dir}/meta.txt" << 'EOF'
SERVICE=deploy
IMAGE_NAME=test-image-deploy.latest
SUPER_PROJECT_REPO_NAME=test-project
DN_PROJECT_ALIAS_PREFIX=test
TAR_FILENAME=test-image-deploy.latest.tar
EOF

  # Mock docker command to simulate image NOT being loaded
  function docker() {
    case "$1" in
      "image")
        case "$2" in
          "ls")
            echo ""  # No images loaded
            return 0
            ;;
          *)
            return 0
            ;;
        esac
        ;;
      *)
        return 0
        ;;
    esac
  }
  export -f docker

  # Change to test directory and run function
  cd "${test_dir}" || exit 1
  run dna::check_offline_deploy_service_discovery

  assert_failure
  assert_output --regexp "Docker image 'test-image-deploy.latest' is not loaded"

  # Cleanup
  temp_del "${test_dir}"
}

@test "dna::check_offline_deploy_service_discovery with missing meta.txt › expect failure" {
  # Setup: Load DNA lib and source the tested file
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/import_dna_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"

  # Create temporary directory for test (without meta.txt)
  local test_dir
  test_dir=$(temp_make)

  # Change to test directory and run function
  cd "${test_dir}" || exit 1
  run dna::check_offline_deploy_service_discovery

  assert_failure
  refute_output

  # Cleanup
  temp_del "${test_dir}"
}

@test "dna::check_offline_deploy_service_discovery with invalid meta.txt (missing SERVICE) › expect failure" {
  # Setup: Load DNA lib and source the tested file
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/import_dna_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"

  # Create temporary directory for test
  local test_dir
  test_dir=$(temp_make)

  # Create invalid meta.txt file (missing SERVICE)
  cat > "${test_dir}/meta.txt" << 'EOF'
IMAGE_NAME=test-image-deploy.latest
SUPER_PROJECT_REPO_NAME=test-project
DN_PROJECT_ALIAS_PREFIX=test
TAR_FILENAME=test-image-deploy.latest.tar
EOF

  # Change to test directory and run function
  cd "${test_dir}" || exit 1
  run dna::check_offline_deploy_service_discovery

  assert_failure
  assert_output --regexp "Invalid meta.txt: missing SERVICE or IMAGE_NAME field"

  # Cleanup
  temp_del "${test_dir}"
}

@test "dna::check_offline_deploy_service_discovery with invalid meta.txt (missing IMAGE_NAME) › expect failure" {
  # Setup: Load DNA lib and source the tested file
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/import_dna_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"

  # Create temporary directory for test
  local test_dir
  test_dir=$(temp_make)

  # Create invalid meta.txt file (missing IMAGE_NAME)
  cat > "${test_dir}/meta.txt" << 'EOF'
SERVICE=deploy
SUPER_PROJECT_REPO_NAME=test-project
DN_PROJECT_ALIAS_PREFIX=test
TAR_FILENAME=test-image-deploy.latest.tar
EOF

  # Change to test directory and run function
  cd "${test_dir}" || exit 1
  run dna::check_offline_deploy_service_discovery

  assert_failure
  assert_output --regexp "Invalid meta.txt: missing SERVICE or IMAGE_NAME field"

  # Cleanup
  temp_del "${test_dir}"
}

@test "dna::check_offline_deploy_service_discovery with develop service › expect success" {
  # Setup: Load DNA lib and source the tested file
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/import_dna_lib.bash" || exit 1
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"

  # Create temporary directory for test
  local test_dir
  test_dir=$(temp_make)

  # Create mock meta.txt file for develop service
  cat > "${test_dir}/meta.txt" << 'EOF'
SERVICE=develop
IMAGE_NAME=test-image-develop.latest
SUPER_PROJECT_REPO_NAME=test-project
DN_PROJECT_ALIAS_PREFIX=test
TAR_FILENAME=test-image-develop.latest.tar
EOF

  # Mock docker command to simulate image being loaded
  function docker() {
    case "$1" in
      "image")
        case "$2" in
          "ls")
            echo "test-image-develop.latest"
            return 0
            ;;
          *)
            return 0
            ;;
        esac
        ;;
      *)
        return 0
        ;;
    esac
  }
  export -f docker

  # Change to test directory and run function
  cd "${test_dir}" || exit 1
  run dna::check_offline_deploy_service_discovery

  assert_success
  assert_output "develop"

  # Cleanup
  temp_del "${test_dir}"
}
