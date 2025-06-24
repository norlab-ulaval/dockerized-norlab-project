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
        "save")
          echo "Mock docker image save called with args: $*"
          # Create a mock tar file for testing
          touch "$4"  # $4 should be the output file path
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
# Create a temporary directory for save testing
TEMP_SAVE_DIR=$(mktemp -d)
echo "Testing save deploy command with temporary directory: ${TEMP_SAVE_DIR}"

# Test save deploy command (this should work with mocked docker commands)
dna save "${TEMP_SAVE_DIR}" deploy || exit 1

# Clean up
rm -rf "${TEMP_SAVE_DIR}"
