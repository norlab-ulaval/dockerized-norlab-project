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

cd "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1

# Mock docker command for dryrun testing
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
# Create a temporary directory for load testing
temp_save_dir=$(mktemp -d)
echo "Testing load command with temporary directory: ${temp_save_dir}"

# Create mock save directory structure for testing
mock_save_dir="${temp_save_dir}/dna-save-develop-test-project-202312151430"
mkdir -p "${mock_save_dir}"

# Create mock metadata file
cat > "${mock_save_dir}/meta.txt" << 'EOF'
# DNA Save Metadata
#   Generated on: Fri Dec 15 14:30:00 UTC 2023
#   From host:
#     Name: MacBook-Pro-M3-Karen
#     Architecture and OS: darwin/arm64

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

# Create mock tar file
touch "${mock_save_dir}/test-image-develop.latest.tar"

# Test load command (this should work with mocked docker commands)
dna load "${mock_save_dir}" || exit 1

# Clean up
rm -rf "${temp_save_dir}"
