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

TESTED_FILE="cuda_tools.bash"
TESTED_FILE_PATH="src/lib/core/utils"

# executed once before starting the first test (valide for all test in that file)
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
}

# executed before each test
setup() {
  # Source the import_dna_lib.bash to load N2ST and DNA libraries
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/import_dna_lib.bash" || exit 1
  
  # Source the file under test
  source "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" || exit 1
  
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
  unset NVIDIA_VISIBLE_DEVICES
  unset NVIDIA_DRIVER_CAPABILITIES
  unset DN_DOCKER_RUNTIME
  bats_print_run_env_variable_on_error
  # Clean up mock commands
  if [[ -n "${MOCK_BIN_DIR}" && -d "${MOCK_BIN_DIR}" ]]; then
    rm -rf "${MOCK_BIN_DIR}"
  fi
}

# ====Helper Functions==============================================================================

# Helper function to create mock nvidia-container-cli command
create_mock_nvidia_container_cli() {
  local behavior="$1"  # "success", "fail", "not_found"
  local arch="$2"      # architecture to return (e.g., "7.5")
  
  case "$behavior" in
    "success")
      cat > "${MOCK_BIN_DIR}/nvidia-container-cli" << EOF
#!/bin/bash
if [[ "\$1" == "-V" ]]; then
  echo "nvidia-container-cli version 1.0.0"
  exit 0
elif [[ "\$1" == "info" ]]; then
  echo "Architecture: ${arch}"
  exit 0
fi
exit 1
EOF
      ;;
    "fail")
      cat > "${MOCK_BIN_DIR}/nvidia-container-cli" << EOF
#!/bin/bash
exit 1
EOF
      ;;
    "not_found")
      # Don't create the command - simulate command not found
      return 0
      ;;
  esac
  
  chmod +x "${MOCK_BIN_DIR}/nvidia-container-cli"
}

# Helper function to create mock nvcc command
create_mock_nvcc() {
  local behavior="$1"  # "success", "fail", "not_found"
  local version="$2"   # CUDA version to return
  
  case "$behavior" in
    "success")
      cat > "${MOCK_BIN_DIR}/nvcc" << EOF
#!/bin/bash
if [[ "\$1" == "-V" ]]; then
  echo "nvcc: NVIDIA (R) Cuda compiler driver"
  echo "Copyright (c) 2005-2021 NVIDIA Corporation"
  echo "Built on Sun_Mar_21_19:15:46_PDT_2021"
  echo "Cuda compilation tools, release ${version}, V${version}"
  exit 0
fi
exit 1
EOF
      ;;
    "fail")
      cat > "${MOCK_BIN_DIR}/nvcc" << EOF
#!/bin/bash
exit 1
EOF
      ;;
    "not_found")
      # Don't create the command - simulate command not found
      return 0
      ;;
  esac
  
  chmod +x "${MOCK_BIN_DIR}/nvcc"
}

# Helper function to create mock nvidia-smi command
create_mock_nvidia_smi() {
  local behavior="$1"  # "success", "fail", "not_found"
  local compute_cap="$2"  # compute capability to return (e.g., "7.5")
  
  case "$behavior" in
    "success")
      cat > "${MOCK_BIN_DIR}/nvidia-smi" << EOF
#!/bin/bash
if [[ "\$*" == *"--query-gpu=compute_cap"* ]]; then
  echo "${compute_cap}"
  exit 0
fi
exit 1
EOF
      ;;
    "fail")
      cat > "${MOCK_BIN_DIR}/nvidia-smi" << EOF
#!/bin/bash
exit 1
EOF
      ;;
    "not_found")
      # Don't create the command - simulate command not found
      return 0
      ;;
  esac
  
  chmod +x "${MOCK_BIN_DIR}/nvidia-smi"
}

# Helper function to create mock docker compose command with command validation
create_mock_docker_compose() {
  local behavior="$1"    # "success", "fail", "not_found"
  local return_value="$2" # value to return for torch compatibility
  local expected_compose_path="$3"     # expected compose path (optional)
  local expected_compose_file="$4"     # expected compose file (optional)
  local expected_service="$5"          # expected service (optional)
  local expected_host_gpu_arch="$6"    # expected host gpu architecture (optional)
  
  case "$behavior" in
    "success")
      cat > "${MOCK_BIN_DIR}/docker" << 'EOF'
#!/bin/bash

# Save all arguments to a file for validation
echo "DOCKER_COMPOSE_ARGS: $*" > /tmp/docker_compose_call.log

if [[ "$1" == "compose" ]]; then
  # Validate the command structure if validation parameters are provided
  if [[ -n "$EXPECTED_COMPOSE_PATH" && -n "$EXPECTED_COMPOSE_FILE" && -n "$EXPECTED_SERVICE" && -n "$EXPECTED_HOST_GPU_ARCH" ]]; then
    expected_cmd="compose -f ${EXPECTED_COMPOSE_PATH}/${EXPECTED_COMPOSE_FILE} run --rm --entrypoint /bin/bash -c ${EXPECTED_SERVICE} /dna-lib-container-tools/project_entrypoints/dn_entrypoint_gpu_checks.bash '${EXPECTED_HOST_GPU_ARCH}'"

    # Log the expected command for debugging
    echo "EXPECTED_CMD: $expected_cmd" >> /tmp/docker_compose_call.log

    # Validate key components of the command
    if [[ "$*" == *"-f ${EXPECTED_COMPOSE_PATH}/${EXPECTED_COMPOSE_FILE}"* ]] && \
       [[ "$*" == *"run"* ]] && \
       [[ "$*" == *"--rm"* ]] && \
       [[ "$*" == *"--entrypoint"* ]] && \
       [[ "$*" == *"/bin/bash -c"* ]] && \
       [[ "$*" == *"${EXPECTED_SERVICE}"* ]] && \
       [[ "$*" == *"/dna-lib-container-tools/project_entrypoints/dn_entrypoint_gpu_checks.bash"* ]] && \
       [[ "$*" == *"'${EXPECTED_HOST_GPU_ARCH}'"* ]]; then
      echo "VALIDATION: PASSED" >> /tmp/docker_compose_call.log
      echo "$RETURN_VALUE"
      exit 0
    else
      echo "VALIDATION: FAILED" >> /tmp/docker_compose_call.log
      echo "Unexpected docker compose command structure" >&2
      exit 1
    fi
  else
    # No validation, just return the value
    echo "$RETURN_VALUE"
    exit 0
  fi
fi
exit 1
EOF
      ;;
    "fail")
      cat > "${MOCK_BIN_DIR}/docker" << 'EOF'
#!/bin/bash
echo "DOCKER_COMPOSE_ARGS: $*" > /tmp/docker_compose_call.log
if [[ "$1" == "compose" ]]; then
  exit 1
fi
exit 1
EOF
      ;;
    "not_found")
      # Don't create the command - simulate command not found
      return 0
      ;;
  esac
  
  # Set environment variables for the mock to use
  if [[ -n "$expected_compose_path" ]]; then
    export EXPECTED_COMPOSE_PATH="$expected_compose_path"
    export EXPECTED_COMPOSE_FILE="$expected_compose_file"
    export EXPECTED_SERVICE="$expected_service"
    export EXPECTED_HOST_GPU_ARCH="$expected_host_gpu_arch"
    export RETURN_VALUE="$return_value"
  fi
  
  chmod +x "${MOCK_BIN_DIR}/docker"
}

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

# ....Tests for dna::fetch_host_nvidia_gpu_architecture function..........................................

@test "dna::fetch_host_nvidia_gpu_architecture › linux/x86_64 with nvidia-container-cli › expect sm_75" {
  create_mock_nvidia_container_cli "success" "7.5"
  
  run dna::fetch_host_nvidia_gpu_architecture "linux/x86"
  assert_success
  assert_output "sm_75"
}

@test "dna::fetch_host_nvidia_gpu_architecture › linux/x86_64 with nvidia-container-cli › expect sm_86" {
  create_mock_nvidia_container_cli "success" "8.6"
  
  run dna::fetch_host_nvidia_gpu_architecture "linux/x86"
  assert_success
  assert_output "sm_86"
}

@test "dna::fetch_host_nvidia_gpu_architecture › l4t/arm64 fallback to nvidia-smi › expect sm_72" {
  create_mock_nvidia_container_cli "not_found"
  create_mock_nvcc "success" "11.4"
  create_mock_nvidia_smi "success" "7.2"
  
  run dna::fetch_host_nvidia_gpu_architecture "l4t/arm64"
  assert_success
  assert_output "sm_72"
}

@test "dna::fetch_host_nvidia_gpu_architecture › l4t/arm64 fallback to nvidia-smi › expect sm_87" {
  create_mock_nvidia_container_cli "not_found"
  create_mock_nvcc "success" "11.4"
  create_mock_nvidia_smi "success" "8.7"
  
  run dna::fetch_host_nvidia_gpu_architecture "l4t/arm64"
  assert_success
  assert_output "sm_87"
}

@test "dna::fetch_host_nvidia_gpu_architecture › darwin/arm64 no GPU support › expect NO-NVIDIA-GPU-SUPPORT" {
  create_mock_nvidia_container_cli "not_found"
  create_mock_nvcc "not_found"
  create_mock_nvidia_smi "not_found"
  
  run dna::fetch_host_nvidia_gpu_architecture "darwin/arm64"
  assert_success
  assert_output "NO-NVIDIA-GPU-SUPPORT"
}

@test "dna::fetch_host_nvidia_gpu_architecture › nvidia-container-cli fails but nvidia-smi works › expect sm_75" {
  create_mock_nvidia_container_cli "fail"
  create_mock_nvcc "success" "11.4"
  create_mock_nvidia_smi "success" "7.5"
  
  run dna::fetch_host_nvidia_gpu_architecture "l4t/arm64"
  assert_success
  assert_output "sm_75"
}

@test "dna::fetch_host_nvidia_gpu_architecture › all commands fail › expect NO-NVIDIA-GPU-SUPPORT" {
  create_mock_nvidia_container_cli "fail"
  create_mock_nvcc "fail"
  create_mock_nvidia_smi "fail"
  
  run dna::fetch_host_nvidia_gpu_architecture "linux/x86"
  assert_success
  assert_output "NO-NVIDIA-GPU-SUPPORT"
}

# ....Tests for dna::check_nvidia_cuda_support function............................................

@test "dna::check_nvidia_cuda_support › nvcc available › expect success" {
  create_mock_nvcc "success" "11.4"
  
  run dna::check_nvidia_cuda_support
  assert_success
}

@test "dna::check_nvidia_cuda_support › nvcc not available › expect failure" {
  create_mock_nvcc "not_found"
  
  run dna::check_nvidia_cuda_support
  assert_failure
}

@test "dna::check_nvidia_cuda_support › nvcc fails › expect failure" {
  create_mock_nvcc "fail"
  
  run dna::check_nvidia_cuda_support
  assert_failure
}


# ....Tests for dna::test_host_gpu_to_container_torch_compatibility function.......................

@test "dna::test_host_gpu_to_container_torch_compatibility › docker compose success with validation › expect true" {
  create_mock_docker_compose "success" "true" "/tmp" "docker-compose.yml" "gpu-service" "sm_75"
  
  run dna::test_host_gpu_to_container_torch_compatibility "/tmp" "docker-compose.yml" "gpu-service" "sm_75"
  assert_success
  assert_output "true"
  
  # Verify the docker compose command was called with correct arguments
  run cat /tmp/docker_compose_call.log
  assert_success
  assert_line --partial "DOCKER_COMPOSE_ARGS: compose -f /tmp/docker-compose.yml run --rm --entrypoint /bin/bash -c gpu-service /dna-lib-container-tools/project_entrypoints/dn_entrypoint_gpu_checks.bash"
  assert_line --partial "VALIDATION: PASSED"
}

@test "dna::test_host_gpu_to_container_torch_compatibility › docker compose success with validation › expect false" {
  create_mock_docker_compose "success" "false" "/tmp" "docker-compose.yml" "gpu-service" "sm_75"
  
  run dna::test_host_gpu_to_container_torch_compatibility "/tmp" "docker-compose.yml" "gpu-service" "sm_75"
  assert_success
  assert_output "false"
  
  # Verify the docker compose command was called with correct arguments
  run cat /tmp/docker_compose_call.log
  assert_success
  assert_line --partial "DOCKER_COMPOSE_ARGS: compose -f /tmp/docker-compose.yml run --rm --entrypoint /bin/bash -c gpu-service /dna-lib-container-tools/project_entrypoints/dn_entrypoint_gpu_checks.bash"
  assert_line --partial "VALIDATION: PASSED"
}

@test "dna::test_host_gpu_to_container_torch_compatibility › docker compose success with validation › expect no-torch" {
  create_mock_docker_compose "success" "no-torch" "/tmp" "docker-compose.yml" "gpu-service" "sm_75"
  
  run dna::test_host_gpu_to_container_torch_compatibility "/tmp" "docker-compose.yml" "gpu-service" "sm_75"
  assert_success
  assert_output "no-torch"
  
  # Verify the docker compose command was called with correct arguments
  run cat /tmp/docker_compose_call.log
  assert_success
  assert_line --partial "DOCKER_COMPOSE_ARGS: compose -f /tmp/docker-compose.yml run --rm --entrypoint /bin/bash -c gpu-service /dna-lib-container-tools/project_entrypoints/dn_entrypoint_gpu_checks.bash"
  assert_line --partial "VALIDATION: PASSED"
}

@test "dna::test_host_gpu_to_container_torch_compatibility › validate complete command structure › expect true" {
  create_mock_docker_compose "success" "true" "/tmp" "docker-compose.yml" "gpu-service" "sm_75"
  
  run dna::test_host_gpu_to_container_torch_compatibility "/tmp" "docker-compose.yml" "gpu-service" "sm_75"
  assert_success
  assert_output "true"
  
  # Verify the complete command structure as specified in the issue
  run cat /tmp/docker_compose_call.log #>&3
  assert_success
  assert_line --partial "compose -f /tmp/docker-compose.yml run --rm --entrypoint /bin/bash -c gpu-service /dna-lib-container-tools/project_entrypoints/dn_entrypoint_gpu_checks.bash"
  assert_line --partial "'sm_75'"
  assert_line --partial "VALIDATION: PASSED"
}

@test "dna::test_host_gpu_to_container_torch_compatibility › docker compose fails › expect failure" {
  create_mock_docker_compose "fail"
  
  run dna::test_host_gpu_to_container_torch_compatibility "/tmp" "docker-compose.yml" "gpu-service" "sm_75"
  assert_failure
}

@test "dna::test_host_gpu_to_container_torch_compatibility › missing arguments › expect failure" {
  run dna::test_host_gpu_to_container_torch_compatibility
  assert_failure
}

@test "dna::test_host_gpu_to_container_torch_compatibility › missing compose_path › expect failure" {
  run dna::test_host_gpu_to_container_torch_compatibility "" "docker-compose.yml" "gpu-service" "sm_75"
  assert_failure
}

@test "dna::test_host_gpu_to_container_torch_compatibility › missing compose_file › expect failure" {
  run dna::test_host_gpu_to_container_torch_compatibility "/tmp" "" "gpu-service" "sm_75"
  assert_failure
}

@test "dna::test_host_gpu_to_container_torch_compatibility › missing service › expect failure" {
  run dna::test_host_gpu_to_container_torch_compatibility "/tmp" "docker-compose.yml" "" "sm_75"
  assert_failure
}

@test "dna::test_host_gpu_to_container_torch_compatibility › missing host_gpu_arch › expect failure" {
  run dna::test_host_gpu_to_container_torch_compatibility "/tmp" "docker-compose.yml" "gpu-service" ""
  assert_failure
}

# ....Tests for dna::configure_gpu_capabilities function..............................................

@test "dna::configure_gpu_capabilities › darwin/arm64 › expect no GPU support" {
  # Mock the required functions
  function dna::fetch_host_nvidia_gpu_architecture() {
    echo "NO-NVIDIA-GPU-SUPPORT"
  }
  export -f dna::fetch_host_nvidia_gpu_architecture
  
  function n2st::print_msg_warning() {
    echo "WARNING: $*"
  }
  export -f n2st::print_msg_warning
  
  # Set initial environment
  export NVIDIA_VISIBLE_DEVICES="all"
  export NVIDIA_DRIVER_CAPABILITIES="all"
  export DN_DOCKER_RUNTIME="runc"
  
  # Execute the function directly (not in subshell)
  dna::configure_gpu_capabilities "darwin/arm64" "/tmp" "docker-compose.yml" "gpu-service"
  
  # Verify environment variables are set correctly
  assert_equal "${NVIDIA_VISIBLE_DEVICES}" "void"
  assert_equal "${NVIDIA_DRIVER_CAPABILITIES}" ""
  assert_equal "${DN_DOCKER_RUNTIME}" "runc"
}

@test "dna::configure_gpu_capabilities › linux/x86 with no GPU support › expect void settings" {
  # Mock the required functions
  function dna::fetch_host_nvidia_gpu_architecture() {
    echo "NO-NVIDIA-GPU-SUPPORT"
  }
  export -f dna::fetch_host_nvidia_gpu_architecture
  
  # Set initial environment
  export NVIDIA_VISIBLE_DEVICES="all"
  export NVIDIA_DRIVER_CAPABILITIES="all"
  export DN_DOCKER_RUNTIME="runc"
  
  # Execute the function directly (not in subshell)
  dna::configure_gpu_capabilities "linux/x86" "/tmp" "docker-compose.yml" "gpu-service"
  
  # Verify environment variables are set correctly
  assert_equal "${NVIDIA_VISIBLE_DEVICES}" "void"
  assert_equal "${NVIDIA_DRIVER_CAPABILITIES}" ""
  assert_equal "${DN_DOCKER_RUNTIME}" "runc"
}

@test "dna::configure_gpu_capabilities › linux/x86 with GPU support and torch compatible › expect nvidia runtime" {
  # Mock the required functions
  function dna::fetch_host_nvidia_gpu_architecture() {
    echo "sm_75"
  }
  export -f dna::fetch_host_nvidia_gpu_architecture
  
  function dna::test_host_gpu_to_container_torch_compatibility() {
    echo "true"
  }
  export -f dna::test_host_gpu_to_container_torch_compatibility
  
  # Set initial environment
  export NVIDIA_VISIBLE_DEVICES="all"
  export NVIDIA_DRIVER_CAPABILITIES="all"
  export DN_DOCKER_RUNTIME="runc"
  
  # Execute the function directly (not in subshell)
  dna::configure_gpu_capabilities "linux/x86" "/tmp" "docker-compose.yml" "gpu-service"
  
  # Verify environment variables are set correctly
  assert_equal "${NVIDIA_VISIBLE_DEVICES}" "all"
  assert_equal "${NVIDIA_DRIVER_CAPABILITIES}" "all"
  assert_equal "${DN_DOCKER_RUNTIME}" "nvidia"
}

@test "dna::configure_gpu_capabilities › linux/x86 with GPU support and no torch › expect nvidia runtime" {
  # Mock the required functions
  function dna::fetch_host_nvidia_gpu_architecture() {
    echo "sm_75"
  }
  export -f dna::fetch_host_nvidia_gpu_architecture
  
  function dna::test_host_gpu_to_container_torch_compatibility() {
    echo "no-torch"
  }
  export -f dna::test_host_gpu_to_container_torch_compatibility
  
  # Set initial environment
  export NVIDIA_VISIBLE_DEVICES="all"
  export NVIDIA_DRIVER_CAPABILITIES="all"
  export DN_DOCKER_RUNTIME="runc"
  
  # Execute the function directly (not in subshell)
  dna::configure_gpu_capabilities "linux/x86" "/tmp" "docker-compose.yml" "gpu-service"
  
  # Verify environment variables are set correctly
  assert_equal "${NVIDIA_VISIBLE_DEVICES}" "all"
  assert_equal "${NVIDIA_DRIVER_CAPABILITIES}" "all"
  assert_equal "${DN_DOCKER_RUNTIME}" "nvidia"
}

@test "dna::configure_gpu_capabilities › linux/x86 with GPU support but torch incompatible › expect void settings" {
  # Mock the required functions
  function dna::fetch_host_nvidia_gpu_architecture() {
    echo "sm_75"
  }
  export -f dna::fetch_host_nvidia_gpu_architecture
  
  function dna::test_host_gpu_to_container_torch_compatibility() {
    echo "false"
  }
  export -f dna::test_host_gpu_to_container_torch_compatibility
  
  function n2st::print_msg_warning() {
    echo "WARNING: $*"
  }
  export -f n2st::print_msg_warning
  
  # Set initial environment
  export NVIDIA_VISIBLE_DEVICES="all"
  export NVIDIA_DRIVER_CAPABILITIES="all"
  export DN_DOCKER_RUNTIME="runc"
  
  # Execute the function directly (not in subshell)
  dna::configure_gpu_capabilities "linux/x86" "/tmp" "docker-compose.yml" "gpu-service"
  
  # Verify environment variables are set correctly
  assert_equal "${NVIDIA_VISIBLE_DEVICES}" "void"
  assert_equal "${NVIDIA_DRIVER_CAPABILITIES}" ""
  assert_equal "${DN_DOCKER_RUNTIME}" "runc"
}

@test "dna::configure_gpu_capabilities › l4t/arm64 with GPU support and torch compatible › expect nvidia runtime" {
  # Mock the required functions
  function dna::fetch_host_nvidia_gpu_architecture() {
    echo "sm_72"
  }
  export -f dna::fetch_host_nvidia_gpu_architecture
  
  function dna::test_host_gpu_to_container_torch_compatibility() {
    echo "true"
  }
  export -f dna::test_host_gpu_to_container_torch_compatibility
  
  # Set initial environment
  export NVIDIA_VISIBLE_DEVICES="all"
  export NVIDIA_DRIVER_CAPABILITIES="all"
  export DN_DOCKER_RUNTIME="runc"
  
  # Execute the function directly (not in subshell)
  dna::configure_gpu_capabilities "l4t/arm64" "/tmp" "docker-compose.yml" "gpu-service"
  
  # Verify environment variables are set correctly
  assert_equal "${NVIDIA_VISIBLE_DEVICES}" "all"
  assert_equal "${NVIDIA_DRIVER_CAPABILITIES}" "all"
  assert_equal "${DN_DOCKER_RUNTIME}" "nvidia"
}

@test "dna::configure_gpu_capabilities › user sets NVIDIA_VISIBLE_DEVICES to void › expect void settings" {
  # Mock the required functions
  function dna::fetch_host_nvidia_gpu_architecture() {
    echo "sm_75"
  }
  export -f dna::fetch_host_nvidia_gpu_architecture
  
  # Set initial environment with user preference
  export NVIDIA_VISIBLE_DEVICES="void"
  export NVIDIA_DRIVER_CAPABILITIES="all"
  export DN_DOCKER_RUNTIME="runc"
  
  # Execute the function directly (not in subshell)
  dna::configure_gpu_capabilities "linux/x86" "/tmp" "docker-compose.yml" "gpu-service"
  
  # Verify environment variables respect user setting
  assert_equal "${NVIDIA_VISIBLE_DEVICES}" "void"
  assert_equal "${NVIDIA_DRIVER_CAPABILITIES}" ""
  assert_equal "${DN_DOCKER_RUNTIME}" "runc"
}

@test "dna::configure_gpu_capabilities › user did not set NVIDIA_VISIBLE_DEVICES › expect void settings" {
  # Mock the required functions
  function dna::fetch_host_nvidia_gpu_architecture() {
    echo "sm_75"
  }
  export -f dna::fetch_host_nvidia_gpu_architecture

  # Set initial environment with user preference
  unset NVIDIA_VISIBLE_DEVICES
  unset NVIDIA_DRIVER_CAPABILITIES
  #export DN_DOCKER_RUNTIME="runc"

  # Execute the function directly (not in subshell)
  dna::configure_gpu_capabilities "linux/x86" "/tmp" "docker-compose.yml" "gpu-service"

  # Verify environment variables respect user setting
  assert_equal "${NVIDIA_VISIBLE_DEVICES}" "void"
  assert_equal "${NVIDIA_DRIVER_CAPABILITIES}" ""
  assert_equal "${DN_DOCKER_RUNTIME}" "runc"
}

@test "dna::configure_gpu_capabilities › torch compatibility test fails › expect failure" {
  # Mock the required functions
  function dna::fetch_host_nvidia_gpu_architecture() {
    echo "sm_75"
  }
  export -f dna::fetch_host_nvidia_gpu_architecture
  
  function dna::test_host_gpu_to_container_torch_compatibility() {
    return 1  # Simulate failure
  }
  export -f dna::test_host_gpu_to_container_torch_compatibility
  
  # Set initial environment
  export NVIDIA_VISIBLE_DEVICES="all"
  export NVIDIA_DRIVER_CAPABILITIES="all"
  export DN_DOCKER_RUNTIME="runc"
  
  run dna::configure_gpu_capabilities "linux/x86" "/tmp" "docker-compose.yml" "gpu-service"
  assert_failure
}

@test "dna::configure_gpu_capabilities › missing arguments › expect failure" {
  run dna::configure_gpu_capabilities
  assert_failure
}

@test "dna::configure_gpu_capabilities › missing image_arch_and_os › expect failure" {
  run dna::configure_gpu_capabilities "" "/tmp" "docker-compose.yml" "gpu-service"
  assert_failure
}

@test "dna::configure_gpu_capabilities › missing compose_path › expect failure" {
  run dna::configure_gpu_capabilities "linux/x86" "" "docker-compose.yml" "gpu-service"
  assert_failure
}

@test "dna::configure_gpu_capabilities › missing compose_file › expect failure" {
  run dna::configure_gpu_capabilities "linux/x86" "/tmp" "" "gpu-service"
  assert_failure
}

@test "dna::configure_gpu_capabilities › missing service › expect failure" {
  run dna::configure_gpu_capabilities "linux/x86" "/tmp" "docker-compose.yml" ""
  assert_failure
}
