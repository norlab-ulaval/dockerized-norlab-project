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
  echo -e "\n{error_prefix} $0 path to bats-core helper library unreachable at \"${bats_path}\"!"
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi

# ====Tests file configuration=====================================================================

TESTED_FILE="setup_host_dnp_requirements.bash"
TESTED_FILE_PATH="src/lib/core/utils"

# ....Setup........................................................................................
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
  export MOCK_PROJECT_PATH="${BATS_DOCKER_WORKDIR}/utilities/tmp/dockerized-norlab-project-mock"

  # Create temporary directory for tests
  export MOCK_DNP_DIR=$(temp_make)

  # Create mock functions directory in the temporary directory
  mkdir -p "${MOCK_DNP_DIR}/src/lib/core/utils/"
  mkdir -p "${MOCK_DNP_DIR}/utilities/norlab-shell-script-tools/src/utility_scripts"

  # Create a mock import_dnp_lib.bash that sets up the environment
  cat > "${MOCK_DNP_DIR}/src/lib/core/utils/import_dnp_lib.bash" << 'EOF'
#!/bin/bash
# Mock import_dnp_lib.bash

# ....Setup........................................................................................

# Set message formatting variables
export MSG_DIMMED_FORMAT=""
export MSG_END_FORMAT=""

# Set up environment variables
export DNP_ROOT="${MOCK_DNP_DIR}"
export DNP_LIB_PATH="${MOCK_DNP_DIR}/src/lib"

# ....Mock dependencies loading test functions.....................................................
function dnp::import_lib_and_dependencies() {
  return 0
}

function n2st::print_msg() {
  echo "Mock n2st::print_msg called with args: $*"
  return 0
}

function n2st::print_msg_done() {
  echo "Mock n2st::print_msg_done called with args: $*"
  return 0
}

function n2st::print_msg_warning() {
  echo "Mock n2st::print_msg_warning called with args: $*"
  return 0
}

function n2st::print_msg_error() {
  echo "Mock n2st::print_msg_error called with args: $*"
  return 1
}

# ....Export mock functions........................................................................
for func in $(compgen -A function | grep -e dnp:: -e n2st::); do
  export -f "$func"
done

# ....Teardown.....................................................................................
# Print a message to indicate that the mock import_dnp_lib.bash has been loaded
echo "[DNP done] Mock import_dnp_lib.bash and its librairies loaded"
EOF

}

setup() {
  # Create necessary directories in the temporary directory
  mkdir -p "${MOCK_DNP_DIR}/src/lib/core/utils"
  mkdir -p "${MOCK_DNP_DIR}/utilities/norlab-shell-script-tools/src/utility_scripts"
  mkdir -p "${MOCK_DNP_DIR}/utilities/norlab-shell-script-tools/src/function_library"

  # Copy the setup_host_dnp_requirements.bash file to the temporary directory
  cp "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" "${MOCK_DNP_DIR}/src/lib/core/utils/"

  # Create a mock install_docker_tools.bash file
  # Note: We're not mocking the function itself, but creating a simplified version that doesn't actually install Docker
  cat > "${MOCK_DNP_DIR}/utilities/norlab-shell-script-tools/src/utility_scripts/install_docker_tools.bash" << 'EOF'
#!/bin/bash
# Mock install_docker_tools.bash that doesn't actually install Docker

# Load helper functions
source ../../.env.n2st
source ../function_library/prompt_utilities.bash
source ../function_library/docker_utilities.bash

# Print headers and messages
n2st::print_formated_script_header 'install_docker_tools.bash'
n2st::print_msg "Install utilities"
n2st::print_msg "Install Docker tools"
n2st::print_msg "Configure docker"
n2st::add_user_to_the_docker_group "$(whoami)"
n2st::print_formated_script_footer 'install_docker_tools.bash'

# Return success
exit 0
EOF

  # Create a mock .env.n2st file
  cat > "${MOCK_DNP_DIR}/utilities/norlab-shell-script-tools/.env.n2st" << 'EOF'
#!/bin/bash
# Mock .env.n2st file
export N2ST_PATH="${MOCK_DNP_DIR}/utilities/norlab-shell-script-tools"
EOF

  # Create mock function library files
  cat > "${MOCK_DNP_DIR}/utilities/norlab-shell-script-tools/src/function_library/prompt_utilities.bash" << 'EOF'
#!/bin/bash
# Mock prompt_utilities.bash

function n2st::print_formated_script_header() {
  echo "Mock n2st::print_formated_script_header called with args: $*"
  return 0
}

function n2st::print_formated_script_footer() {
  echo "Mock n2st::print_formated_script_footer called with args: $*"
  return 0
}

# Export functions
for func in $(compgen -A function | grep -e n2st::); do
  export -f "$func"
done
EOF

  cat > "${MOCK_DNP_DIR}/utilities/norlab-shell-script-tools/src/function_library/docker_utilities.bash" << 'EOF'
#!/bin/bash
# Mock docker_utilities.bash

function n2st::add_user_to_the_docker_group() {
  echo "Mock n2st::add_user_to_the_docker_group called with args: $*"
  return 0
}

# Export functions
for func in $(compgen -A function | grep -e n2st::); do
  export -f "$func"
done
EOF

  source "${MOCK_DNP_DIR}/src/lib/core/utils/import_dnp_lib.bash" || exit 1

  # Mock docker command
  function docker() {
    if [[ "$1" == "buildx" && "$2" == "create" ]]; then
      echo "Mock docker buildx create called with args: $*"
      return 0
    elif [[ "$1" == "buildx" && "$2" == "ls" ]]; then
      echo "Mock docker buildx ls called"
      return 0
    else
      echo "Mock docker called with args: $*"
      return 0
    fi
  }
  export -f docker

  # Mock git command
  function git() {
    echo "Mock git called with args: $*"
    return 0
  }
  export -f git

  # Mock source command for shell config files
  function source() {
    if [[ "$1" == "$HOME/.bashrc" || "$1" == "$HOME/.zshrc" ]]; then
      echo "Mock source called with: $1"
      return 0
    else
      builtin source "$@"
    fi
  }
  export -f source

  # Mock nvcc command
  function nvcc() {
    if [[ "$1" == "-V" ]]; then
      if [[ "${MOCK_NVCC_INSTALLED}" == "true" ]]; then
        echo "nvcc: NVIDIA (R) Cuda compiler driver"
        echo "Copyright (c) 2005-2023 NVIDIA Corporation"
        echo "Built on Tue_Jul_11_02:20:44_PDT_2023"
        echo "Cuda compilation tools, release 12.2, V12.2.128"
      else
        return 1
      fi
    fi
    return 0
  }
  export -f nvcc

  # Mock command function
  function command() {
    if [[ "$1" == "-v" && "$2" == "nvcc" ]]; then
      if [[ "${MOCK_NVCC_COMMAND_EXISTS}" == "true" ]]; then
        return 0
      else
        return 1
      fi
    else
      builtin command "$@"
    fi
  }
  export -f command

  # Mock uname function
  function uname() {
    if [[ "$1" == "-s" ]]; then
      echo "${MOCK_OS:-Linux}"
    else
      builtin uname "$@"
    fi
  }
  export -f uname
}

# ....Teardown.....................................................................................
teardown() {
  bats_print_run_env_variable_on_error
  cd "${BATS_DOCKER_WORKDIR}" || exit 1
}

teardown_file() {
  # Clean up temporary directories
  temp_del "${MOCK_DNP_DIR}"
}

# ====Test cases==================================================================================

# Test cases for installing docker requirements
@test "dnp::setup_host_dnp_requirements - Install docker requirements › expect success" {
  # Test case: When the function is called, it should install docker requirements
  # This test verifies that the function calls install_docker_tools.bash and returns successfully

  # Mock the install_docker_tools.bash to return success
  function bash() {
    if [[ "$1" == "install_docker_tools.bash" ]]; then
      echo "Mock bash install_docker_tools.bash called"
      return 0
    else
      command bash "$@"
    fi
  }
  export -f bash

  run bash -c "source ${MOCK_DNP_DIR}/src/lib/core/utils/setup_host_dnp_requirements.bash && dnp::setup_host_dnp_requirements"

  # Should succeed
  assert_success

  # Should call install_docker_tools.bash
  assert_output --partial "Mock bash install_docker_tools.bash called"

  # Should call docker buildx create
  assert_output --partial "Mock docker buildx create called with args: buildx create --name local-builder-multiarch-virtualization"

  # Should call docker buildx ls
  assert_output --partial "Mock docker buildx ls called"
}

@test "dnp::setup_host_dnp_requirements - Install docker requirements failure › expect failure" {
  # Test case: When install_docker_tools.bash fails, the function should return an error
  # This test verifies that the function handles failures from install_docker_tools.bash correctly

  # Mock the install_docker_tools.bash to return failure
  function bash() {
    if [[ "$1" == "install_docker_tools.bash" ]]; then
      echo "Mock bash install_docker_tools.bash called (with failure)"
      return 1
    else
      command bash "$@"
    fi
  }
  export -f bash

  run bash -c "source ${MOCK_DNP_DIR}/src/lib/core/utils/setup_host_dnp_requirements.bash && dnp::setup_host_dnp_requirements"

  # Should fail
  assert_failure

  # Should call install_docker_tools.bash
  assert_output --partial "Mock bash install_docker_tools.bash called (with failure)"
}

# Test cases for CUDA toolkit path
@test "dnp::setup_host_dnp_requirements - CUDA toolkit path on macOS › expect skipped" {
  # Test case: When running on macOS, CUDA setup should be skipped
  # This test verifies that the function correctly identifies macOS and skips CUDA configuration

  # Mock OS as Darwin (macOS)
  export MOCK_OS="Darwin"

  run bash -c "source ${MOCK_DNP_DIR}/src/lib/core/utils/setup_host_dnp_requirements.bash && dnp::setup_host_dnp_requirements"

  # Should succeed
  assert_success

  # Should show warning about CUDA not supported on Apple M1
  assert_output --partial "Mock n2st::print_msg_warning called with args: CUDA is not supported yet on Apple M1 computer. Skipping cuda configuration."
}

@test "dnp::setup_host_dnp_requirements - CUDA toolkit path with nvcc already working › expect success" {
  # Test case: When nvcc is already working, no changes should be made to .bashrc
  # This test verifies that the function correctly identifies when nvcc is already working

  # Mock OS as Linux
  export MOCK_OS="Linux"

  # Mock nvcc as installed and working
  export MOCK_NVCC_COMMAND_EXISTS="true"
  export MOCK_NVCC_INSTALLED="true"

  run bash -c "source ${MOCK_DNP_DIR}/src/lib/core/utils/setup_host_dnp_requirements.bash && dnp::setup_host_dnp_requirements"

  # Should succeed
  assert_success

  # Should not show warning about fixing CUDA path
  refute_output --partial "Mock n2st::print_msg_warning called with args: Fixing CUDA path for nvcc"

  # Should show success message for nvcc
  assert_output --partial "Mock n2st::print_msg_done called with args: nvcc installed properly"
}

@test "dnp::setup_host_dnp_requirements - CUDA toolkit path with nvcc not working › expect path fix" {
  # Test case: When nvcc is not working, the function should add CUDA paths to .bashrc
  # This test verifies that the function correctly adds CUDA paths to .bashrc when nvcc is not working

  # Mock OS as Linux
  export MOCK_OS="Linux"

  # Mock nvcc as not installed or not working
  export MOCK_NVCC_COMMAND_EXISTS="false"
  export MOCK_NVCC_INSTALLED="false"

  # Create a mock .bashrc file
  touch "${HOME}/.bashrc"

  run bash -c "source ${MOCK_DNP_DIR}/src/lib/core/utils/setup_host_dnp_requirements.bash && dnp::setup_host_dnp_requirements"

  # Should succeed
  assert_success

  # Should show warning about fixing CUDA path
  assert_output --partial "Mock n2st::print_msg_warning called with args: Fixing CUDA path for nvcc"

  # Should show message about nvcc CUDA path hack completed
  assert_output --partial "Mock n2st::print_msg_done called with args: nvcc CUDA path hack completed."
}

@test "dnp::setup_host_dnp_requirements - CUDA toolkit path with nvcc fixed but still not working › expect error" {
  # Test case: When nvcc is fixed but still not working, the function should show an error
  # This test verifies that the function correctly identifies when nvcc is still not working after fixing

  # Mock OS as Linux
  export MOCK_OS="Linux"

  # Mock nvcc command as existing but not properly installed
  export MOCK_NVCC_COMMAND_EXISTS="true"
  export MOCK_NVCC_INSTALLED="false"

  run bash -c "source ${MOCK_DNP_DIR}/src/lib/core/utils/setup_host_dnp_requirements.bash && dnp::setup_host_dnp_requirements"

  # Should succeed (the function itself doesn't fail, it just shows an error message)
  assert_success

  # Should show error about nvcc not installed properly
  assert_output --partial "Mock n2st::print_msg_error called with args:  Check your nvcc installation. It's stil NOT installed properly!"
}
