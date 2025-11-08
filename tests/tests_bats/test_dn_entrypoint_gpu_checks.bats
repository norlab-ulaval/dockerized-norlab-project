#!/usr/bin/env bats
# =================================================================================================
#
# Unit tests for src/lib/core/utils/cuda_tools.bash
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

TESTED_FILE="dn_entrypoint_gpu_checks.bash"
TESTED_FILE_PATH="src/lib/core/docker/container-tools/entrypoints"

# Executed once before starting the first test (valide for all test in that file)
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
}

# executed before each test
setup() {
  # Source the import_dna_lib.bash to load N2ST and DNA libraries
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dna_lib.bash" || exit 1
  
  # Create temporary directory for mock commands
  MOCK_BIN_DIR=$(mktemp -d)
  export MOCK_BIN_DIR
  export PATH="${MOCK_BIN_DIR}:${PATH}"
  
  # Mock n2st::print_msg to suppress debug output during tests
  function n2st::print_msg() {
    # Suppress output during tests
    return 0
  }
  export -f n2st::print_msg
}

# ====Teardown=====================================================================================

# executed after each test
teardown() {
  bats_print_run_env_variable_on_error
  # Clean up mock commands
  if [[ -n "${MOCK_BIN_DIR}" && -d "${MOCK_BIN_DIR}" ]]; then
    rm -rf "${MOCK_BIN_DIR}"
  fi
}

# ====Helper Functions==============================================================================

# Helper function to create mock pip command
create_mock_pip_show() {
  local behavior="$1"  # "has_torch", "no_torch"
  
  case "$behavior" in
    "has_torch")
      cat > "${MOCK_BIN_DIR}/pip" << EOF
#!/bin/bash
if [[ "\$*" == *"-qq show torch"* ]]; then
  exit 0
elif [[ "\$*" == *"show torch"* ]]; then
  echo "Name: torch"
  echo "Version: 1.12.0"
  exit 0
fi
exit 1
EOF
      ;;
    "no_torch")
      cat > "${MOCK_BIN_DIR}/pip" << EOF
#!/bin/bash
if [[ "\$*" == *"show torch"* ]]; then
  exit 1
fi
exit 1
EOF
      ;;
  esac
  
  chmod +x "${MOCK_BIN_DIR}/pip"
}

# Helper function to create mock python3 command
create_mock_python3_is_host_gpu_to_container_torch_compatible() {
  local torch_result="$1"  # "true", "false"
  
  cat > "${MOCK_BIN_DIR}/python3" << EOF
#!/bin/bash
if [[ "\$*" == *"import torch"* ]]; then
  echo "${torch_result}"
  exit 0
fi
exit 1
EOF
  
  chmod +x "${MOCK_BIN_DIR}/python3"
}

# ====Test cases===================================================================================

# ....Tests for dna::test_container_torch_supported_architecture function..........................

@test "dna::test_container_torch_supported_architecture › torch available and compatible › expect true" {
  create_mock_pip_show "has_torch"
  create_mock_python3_is_host_gpu_to_container_torch_compatible "true"
  
  run bash "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" "sm_75"
  assert_success
  assert_output "true"
}

@test "dna::test_container_torch_supported_architecture › torch available but incompatible › expect false" {
  create_mock_pip_show "has_torch"
  create_mock_python3_is_host_gpu_to_container_torch_compatible "false"
  
  run bash "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" "sm_75"
  assert_success
  assert_output "false"
}

@test "dna::test_container_torch_supported_architecture › torch not available › expect no-torch" {
  create_mock_pip_show "no_torch"
  
  run bash "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" "sm_75"
  assert_success
  assert_output "no-torch"
}

@test "dna::test_container_torch_supported_architecture › missing argument › expect failure" {
  run bash "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}"
  assert_failure
}
