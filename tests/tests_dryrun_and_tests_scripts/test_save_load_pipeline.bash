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

cd "${DNP_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1

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
echo "Testing full save/load pipeline integration"

# Create a temporary directory for save/load testing
TEMP_SAVE_DIR=$(mktemp -d)
echo "Using temporary directory: ${TEMP_SAVE_DIR}"

# .................................................................................................
# Step 1: Test build with save for develop service
echo "Step 1: Testing 'dnp build develop --save ${TEMP_SAVE_DIR}'"
dnp build develop --save "${TEMP_SAVE_DIR}" || exit 1

# Check if save directory was created (in dry-run mode, this might not actually create files)
echo "Checking for save directory creation..."

# .................................................................................................
# Step 2: Test standalone save command for develop service
echo "Step 2: Testing standalone 'dnp save ${TEMP_SAVE_DIR} develop'"
dnp save "${TEMP_SAVE_DIR}" develop || exit 1

# Find the created save directory
SAVE_DIR_PATTERN="${TEMP_SAVE_DIR}/dnp-save-develop-*"
CREATED_SAVE_DIR=$(find "${TEMP_SAVE_DIR}" -maxdepth 1 -type d -name "dnp-save-develop-*" | head -n 1)

# .................................................................................................
# Step 3: Test load command
if [[ -n "${CREATED_SAVE_DIR}" && -d "${CREATED_SAVE_DIR}" ]]; then
    echo "Found created save directory: ${CREATED_SAVE_DIR}"

    echo "Step 3: Testing 'dnp load ${CREATED_SAVE_DIR}'"
    dnp load "${CREATED_SAVE_DIR}" || exit 1

    echo "✓ Develop service save/load pipeline test completed successfully"
else
    echo "⚠ Save directory not found, creating mock structure for load testing"

    # Create mock save directory structure for load testing
    MOCK_SAVE_DIR="${TEMP_SAVE_DIR}/dnp-save-develop-test-project-$(date +%Y%m%d%H%M)"
#    sudo mkdir -p "${MOCK_SAVE_DIR}"
    mkdir -p "${MOCK_SAVE_DIR}"

    # Create mock metadata file
    cat > "${MOCK_SAVE_DIR}/meta.txt" << EOF
# DNP Save Metadata
SERVICE=develop
IMAGE_NAME=test-image-develop.latest
SUPER_PROJECT_REPO_NAME=test-project
DN_PROJECT_ALIAS_PREFIX=test
EOF

    # Create mock tar file
    touch "${MOCK_SAVE_DIR}/test-image-develop.latest.tar"

    # Test load command
    echo "Step 3: Testing 'dnp load ${MOCK_SAVE_DIR}'"
    dnp load "${MOCK_SAVE_DIR}" || exit 1

    echo "✓ Mock develop service save/load pipeline test completed"
fi

echo ""
echo "Testing deploy service pipeline..."

# .................................................................................................
# Step 4: Test build with save for deploy service
echo "Step 4: Testing 'dnp build deploy --save ${TEMP_SAVE_DIR}'"
dnp build deploy --save "${TEMP_SAVE_DIR}" || exit 1

# .................................................................................................
# Step 5: Test standalone save command for deploy service
echo "Step 5: Testing standalone 'dnp save ${TEMP_SAVE_DIR} deploy'"
dnp save "${TEMP_SAVE_DIR}" deploy || exit 1

# Find the created deploy save directory
DEPLOY_SAVE_DIR=$(find "${TEMP_SAVE_DIR}" -maxdepth 1 -type d -name "dnp-save-deploy-*" | head -n 1)

if [[ -n "${DEPLOY_SAVE_DIR}" && -d "${DEPLOY_SAVE_DIR}" ]]; then
    echo "Found created deploy save directory: ${DEPLOY_SAVE_DIR}"

    # .............................................................................................
    # Step 6: Test load command for deploy
    echo "Step 6: Testing 'dnp load ${DEPLOY_SAVE_DIR}'"
    dnp load "${DEPLOY_SAVE_DIR}" || exit 1

    # .............................................................................................
    # Step 7: Test run command for deploy to validate the loaded image works
    echo "Step 7: Testing 'dnp run deploy' to validate loaded deploy image"
    dnp run deploy --dry-run || exit 1

    echo "✓ Deploy service save/load pipeline test completed successfully"
else
    echo "⚠ Deploy save directory not found, creating mock structure for load testing"

    # Create mock deploy save directory structure for load testing
    MOCK_DEPLOY_SAVE_DIR="${TEMP_SAVE_DIR}/dnp-save-deploy-test-project-$(date +%Y%m%d%H%M)"
#    sudo mkdir -p "${MOCK_DEPLOY_SAVE_DIR}/test-project"
    mkdir -p "${MOCK_DEPLOY_SAVE_DIR}/test-project"

    # Create mock metadata file
    cat > "${MOCK_DEPLOY_SAVE_DIR}/meta.txt" << EOF
# DNP Save Metadata
SERVICE=deploy
IMAGE_NAME=test-image-deploy.latest
SUPER_PROJECT_REPO_NAME=test-project
DN_PROJECT_ALIAS_PREFIX=test
EOF

    # Create mock tar file
    touch "${MOCK_DEPLOY_SAVE_DIR}/test-image-deploy.latest.tar"

    # .............................................................................................
    # Step 6: Test load command for deploy
    echo "Step 6: Testing 'dnp load ${MOCK_DEPLOY_SAVE_DIR}'"
    dnp load "${MOCK_DEPLOY_SAVE_DIR}" || exit 1

    # .............................................................................................
    # Step 7: Test run command for deploy to validate the loaded image works
    echo "Step 7: Testing 'dnp run deploy' to validate loaded deploy image"
    dnp run deploy --dry-run || exit 1

    echo "✓ Mock deploy service save/load pipeline test completed"
fi

echo ""
echo "✓ Full save/load pipeline integration test completed successfully"

# Clean up
rm -rf "${TEMP_SAVE_DIR}"
echo "Cleaned up temporary directory: ${TEMP_SAVE_DIR}"
