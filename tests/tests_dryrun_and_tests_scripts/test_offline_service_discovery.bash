#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
export PATH="$PATH:${DNA_PATH:?err}"
bash "${DNA_ROOT:?err}/tests/setup_mock.bash"
function dna::test_teardown_callback() {
  exit_code=$?
  cd "${DNA_ROOT:?err}" || exit 1
  bash tests/teardown_mock.bash
  exit ${exit_code:1}
}
trap dna::test_teardown_callback EXIT
# Note: command `dna COMMAND ...` require a `|| exit 1` instruction for trap to catch EXIT

PROJECT_PROMPT_NAME="${PROJECT_PROMPT_NAME}-TESTS"
cd "${N2ST_PATH:?'Variable not set'}" || exit 1
source "import_norlab_shell_script_tools_lib.bash" || exit 1

cd "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1

# Mock docker command for testing
function docker() {
  case "$1" in
    "image")
      case "$2" in
        "ls")
          # Check if format flag is present
          if [[ "$*" == *"--format"* ]]; then
            # Simulate that the test images are loaded with proper format
            echo "test-image-deploy.latest"
            echo "test-image-develop.latest"
            return 0
          else
            # Default ls output
            echo "REPOSITORY                TAG       IMAGE ID       CREATED       SIZE"
            echo "test-image-deploy.latest  latest    abc123def456   2 hours ago   1.2GB"
            echo "test-image-develop.latest latest    def456abc123   2 hours ago   1.1GB"
            return 0
          fi
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
echo "Testing offline service discovery functionality"

# Create a temporary directory for testing
temp_test_dir=$(mktemp -d)
echo "Using temporary directory: ${temp_test_dir}"

# ....Test Case 1: Test offline service discovery with deploy service..............................
echo ""
n2st::draw_horizontal_line_across_the_terminal_window "/"
n2st::print_msg "Test Case 1: Offline service discovery with deploy service\n"

# Create mock meta.txt file for deploy service
cat > "${temp_test_dir}/meta.txt" << 'EOF'
# DNA Save Metadata
#   Generated on: Fri Dec 15 14:30:00 UTC 2023

# Configuration
DNA_CONFIG_SCHEME_VERSION=1.0
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

# Copy meta.txt to the mock project directory
cp "${temp_test_dir}/meta.txt" "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}/meta.txt"

# Test dna up command without specifying service (should auto-detect deploy)
echo "Testing 'dna up' without service specification (should auto-detect deploy)"
output=$(dna up --dry-run 2>&1)
up_exit_code=$?

if [[ ${up_exit_code} -ne 0 ]]; then
    n2st::print_msg_error "✗ dna up command failed with exit code: ${up_exit_code}"
    echo "${output}"
    exit 1
fi

#n2st::print_msg "output:\n${output//[DN/   >>> [DN}"

# Check if the output indicates deploy service was used
if echo "${output}" | grep -q "Using offline deployment service: deploy"; then
    n2st::print_msg_done "✓ dna up correctly detected and used deploy service from meta.txt"
else
    n2st::print_msg_error "✗ dna up did not detect deploy service from meta.txt"
    echo "Expected to find: 'Using offline deployment service: deploy'"
    echo "Actual output:"
    echo "${output}"
    exit 1
fi

# ....Test Case 2: Test offline service discovery with develop service.............................
echo ""
n2st::draw_horizontal_line_across_the_terminal_window "/"
n2st::print_msg "Test Case 2: Offline service discovery with develop service\n"

# Create mock meta.txt file for develop service
cat > "${temp_test_dir}/meta.txt" << 'EOF'
# DNA Save Metadata
#   Generated on: Fri Dec 15 14:30:00 UTC 2023

# Configuration
DNA_CONFIG_SCHEME_VERSION=1.0
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

# Copy meta.txt to the mock project directory
cp "${temp_test_dir}/meta.txt" "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}/meta.txt"

# Test dna exec command without specifying service (should auto-detect develop)
echo "Testing 'dna exec' without service specification (should auto-detect develop)"
output=$(dna exec --dry-run -- echo "test" 2>&1)
exec_exit_code=$?

# The exec command may fail because service is not running, but we should still see the detection message
#n2st::print_msg "output:\n${output//[DN/   >>> [DN}"

# Check if the output indicates develop service was used (even if command failed)
if echo "${output}" | grep -q "Using offline deployment service: develop"; then
    n2st::print_msg_done "✓ dna exec correctly detected and used develop service from meta.txt"
else
    n2st::print_msg_error "✗ dna exec did not detect develop service from meta.txt"
    echo "Expected to find: 'Using offline deployment service: develop'"
    echo "Actual output:"
    echo "${output}"
    exit 1
fi

# ....Test Case 3: Test that explicit service specification overrides auto-detection...............
echo ""
n2st::draw_horizontal_line_across_the_terminal_window "/"
n2st::print_msg "Test Case 3: Explicit service specification should override auto-detection\n"

# Test dna attach with explicit deploy service (should not show auto-detection message)
echo "Testing 'dna attach deploy' with explicit service specification"
output=$(dna attach deploy 2>&1)
attach_exit_code=$?

# The attach command may fail because service is not running, but we should check the detection behavior
#n2st::print_msg "output:\n${output//[DN/   >>> [DN}"

# Check that auto-detection message is NOT present when service is explicitly specified
if echo "${output}" | grep -q "Using offline deployment service:"; then
    n2st::print_msg_error "✗ dna attach showed auto-detection message when service was explicitly specified"
    echo "Should not find: 'Using offline deployment service:'"
    echo "Actual output:"
    echo "${output}"
    exit 1
else
    n2st::print_msg_done "✓ dna attach correctly did not show auto-detection message when service was explicitly specified"
fi

# ....Test Case 4: Test dna run command with offline service discovery.............................
echo ""
n2st::draw_horizontal_line_across_the_terminal_window "/"
n2st::print_msg "Test Case 4: dna run command with offline service discovery\n"

# Test dna run without service specification (should auto-detect develop from current meta.txt)
echo "Testing 'dna run' without service specification (should auto-detect develop)"
output=$(dna run --dry-run -- echo "test" 2>&1)
run_exit_code=$?

# The run command may fail, but we should still see the detection message
#n2st::print_msg "output:\n${output//[DN/   >>> [DN}"

# Check if the output indicates develop service was used (even if command failed)
if echo "${output}" | grep -q "Using offline deployment service: develop"; then
    n2st::print_msg_done "✓ dna run correctly detected and used develop service from meta.txt"
else
    n2st::print_msg_error "✗ dna run did not detect develop service from meta.txt"
    echo "Expected to find: 'Using offline deployment service: develop'"
    echo "Actual output:"
    echo "${output}"
    exit 1
fi

# ....Test Case 5: Test behavior when meta.txt is missing..........................................
echo ""
n2st::draw_horizontal_line_across_the_terminal_window "/"
n2st::print_msg "Test Case 5: Behavior when meta.txt is missing\n"

# Remove meta.txt from mock project directory
rm -f "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}/meta.txt"

# Test dna up without meta.txt (should use default develop service)
echo "Testing 'dna up' without meta.txt (should use default develop service)"
output=$(dna up --dry-run 2>&1)
up_no_meta_exit_code=$?

# The up command may fail, but we should check the detection behavior
#n2st::print_msg "output:\n${output//[DN/   >>> [DN}"

# Check that auto-detection message is NOT present when meta.txt is missing
if echo "${output}" | grep -q "Using offline deployment service:"; then
    n2st::print_msg_error "✗ dna up showed auto-detection message when meta.txt was missing"
    echo "Should not find: 'Using offline deployment service:'"
    echo "Actual output:"
    echo "${output}"
    exit 1
else
    n2st::print_msg_done "✓ dna up correctly used default behavior when meta.txt was missing"
fi

# ====Teardown=====================================================================================
echo ""
n2st::print_msg_done "=== Summary ==="
echo "✓ dna up correctly detects and uses service from meta.txt when no service is specified"
echo "✓ dna exec correctly detects and uses service from meta.txt when no service is specified"
echo "✓ dna attach correctly ignores auto-detection when service is explicitly specified"
echo "✓ dna run correctly detects and uses service from meta.txt when no service is specified"
echo "✓ All commands correctly use default behavior when meta.txt is missing"
echo "✓ Offline service discovery functionality test completed successfully"

# Clean up
cd "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1
rm -rf "${temp_test_dir}"
echo "Cleaned up temporary directory: ${temp_test_dir}"
