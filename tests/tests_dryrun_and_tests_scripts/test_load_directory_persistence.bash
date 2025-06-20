#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
export PATH="$PATH:${DNP_PATH:?err}"
bash "${DNP_ROOT:?err}/tests/setup_mock.bash"
function dnp::test_teardown_callback() {
  exit_code=$?
  cd "${DNP_ROOT:?err}" || exit 1
  bash tests/teardown_mock.bash
  exit ${exit_code:1}
}
trap dnp::test_teardown_callback EXIT
# Note: command `dnp COMMAND ...` require a `|| exit 1` instruction for trap to catch EXIT

PROJECT_PROMPT_NAME="${PROJECT_PROMPT_NAME}-TESTS"
cd "${N2ST_PATH:?'Variable not set'}" || exit 1
source "import_norlab_shell_script_tools_lib.bash" || exit 1

cd "${DNP_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1

# Mock docker command for testing
function docker() {
  case "$1" in
    "image")
      case "$2" in
        "load")
          echo "Mock docker image load called with args: $*"
          return 0
          ;;
        *)
          echo "Mock docker image command: $*"
          return 0
          ;;
      esac
      ;;
    *)
      echo "Mock docker command: $*"
      return 0
      ;;
  esac
}
export -f docker

# ====begin========================================================================================
echo "Testing load command directory persistence behavior"

# Create a temporary directory for load testing
temp_save_dir=$(mktemp -d)
echo "Using temporary directory: ${temp_save_dir}"

# Test Case 1: Deploy service should change directory and persist
echo ""
n2st::draw_horizontal_line_across_the_terminal_window "/"
n2st::print_msg "Test Case 1: Deploy service directory persistence\n"

# Create mock deploy save directory structure
mock_deploy_save_dir="${temp_save_dir}/dnp-save-deploy-test-project-202312151430"
project_dir="${mock_deploy_save_dir}/test-project"
mkdir -p "${project_dir}"

# Create mock metadata file for deploy service
cat > "${mock_deploy_save_dir}/meta.txt" << 'EOF'
# DNP Save Metadata
#   Generated on: Fri Dec 15 14:30:00 UTC 2023

# Configuration
DNP_CONFIG_SCHEME_VERSION=1.0
DN_PROJECT_GIT_REMOTE_URL=https://github.com/test/test-project.git
DN_PROJECT_ALIAS_PREFIX=test

# Project Information
SUPER_PROJECT_REPO_NAME=test-project
SERVICE=deploy
IMAGE_NAME=test-image-deploy.latest

# Git Information
BRANCH=main
COMMIT=abc123def456

# Save Information
SAVE_DATE=Fri Dec 15 14:30:00 UTC 2023
SAVE_TIMESTAMP=202312151430
TAR_FILENAME=test-image-deploy.latest.tar
EOF

# Create mock tar file
touch "${mock_deploy_save_dir}/test-image-deploy.latest.tar"

# Record current directory before load
initial_cwd=$(pwd)
echo "Initial working directory: ${initial_cwd}"

# Test load command for deploy service
echo "Testing 'dnp load ${mock_deploy_save_dir}'"
output=$(dnp load "${mock_deploy_save_dir}" 2>&1)
load_exit_code=$?

if [[ ${load_exit_code} -ne 0 ]]; then
    n2st::print_msg_error "✗ Load command failed with exit code: ${load_exit_code}"
    echo "${output}"
    exit 1
fi

echo "${output}"

# Test that the load command provides correct instructions for directory change
if echo "${output}" | grep -q "cd \"${project_dir}\""; then
    n2st::print_msg_done "✓ Load command provided correct directory change instructions: ${project_dir}"
else
    n2st::print_msg_error "✗ Load command did not provide correct directory change instructions"
    echo "Expected to find: cd \"${project_dir}\""
    echo "Actual output:"
    echo "${output}"
    exit 1
fi

# Verify that the current directory hasn't changed (realistic behavior)
final_cwd="$(pwd)"
if [[ "${final_cwd}" == "${initial_cwd}" ]]; then
    n2st::print_msg_done "✓ Load command correctly did not change the current shell directory (expected behavior)"
else
    n2st::print_msg_error "✗ Load command unexpectedly changed the current shell directory"
    echo "  Initial: ${initial_cwd}"
    echo "  Final: ${final_cwd}"
    exit 1
fi

# Test Case 2: Develop service should NOT change directory (for comparison)
echo ""
n2st::draw_horizontal_line_across_the_terminal_window "/"
n2st::print_msg "Test Case 2: Develop service should not change directory\n"

# Create mock develop save directory structure
mock_develop_save_dir="${temp_save_dir}/dnp-save-develop-test-project-202312151430"
mkdir -p "${mock_develop_save_dir}"

# Create mock metadata file for develop service
cat > "${mock_develop_save_dir}/meta.txt" << 'EOF'
# DNP Save Metadata
#   Generated on: Fri Dec 15 14:30:00 UTC 2023

# Configuration
DNP_CONFIG_SCHEME_VERSION=1.0
DN_PROJECT_GIT_REMOTE_URL=https://github.com/test/test-project.git
DN_PROJECT_ALIAS_PREFIX=test

# Project Information
SUPER_PROJECT_REPO_NAME=test-project
SERVICE=develop
IMAGE_NAME=test-image-develop.latest

# Git Information
BRANCH=main
COMMIT=abc123def456

# Save Information
SAVE_DATE=Fri Dec 15 14:30:00 UTC 2023
SAVE_TIMESTAMP=202312151430
TAR_FILENAME=test-image-develop.latest.tar
EOF

# Create mock tar file
touch "${mock_develop_save_dir}/test-image-develop.latest.tar"

# Test load command for develop service
echo "Testing 'dnp load ${mock_develop_save_dir}'"
dnp load "${mock_develop_save_dir}" || exit 1

# For develop service, we should still be in the same directory
current_cwd=$(pwd)
echo "Current working directory after develop load: ${current_cwd}"

if [[ "${current_cwd}" == "${DNP_MOCK_SUPER_PROJECT_ROOT}" ]]; then
    n2st::print_msg_done "✓ Develop service load did not change directory (expected behavior)"
else
    n2st::print_msg_error "✗ Develop service load unexpectedly changed directory"
    exit 1
fi

echo ""
n2st::draw_horizontal_line_across_the_terminal_window "/"
n2st::print_msg_done "Summary\n"
echo "✓ Deploy service load command correctly provides instructions for directory change to SAVE_DIR_PATH/SUPER_PROJECT_REPO_NAME"
echo "✓ Deploy service load command correctly does not change the current shell directory (realistic behavior)"
echo "✓ Develop service load command correctly does not change directory"
echo "✓ Directory persistence behavior test completed successfully"

# Clean up
rm -rf "${temp_save_dir}"
echo "Cleaned up temporary directory: ${temp_save_dir}"
