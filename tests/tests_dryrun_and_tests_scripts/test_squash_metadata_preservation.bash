#!/bin/bash
# =================================================================================================
# Integration test: dna::squash_docker_image — metadata preservation
#
# Builds a minimal test Docker image with known ENV, ENTRYPOINT, CMD, WORKDIR, LABEL, USER,
# calls dna::squash_docker_image against it, then verifies via docker inspect that all
# metadata is identical before and after squashing.
#
# Requires: Docker (running daemon) and python3 on host.
# Requires real docker build/run (fast — no multi-arch or online build steps).
# =================================================================================================
# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
export PATH="$PATH:${DNA_PATH:?err}"
bash "${DNA_ROOT:?err}/tests/setup_mock.bash"
function dna::test_teardown_callback() {
  local exit_code=$?
  # Clean up test images if they exist
  docker rmi "dna-squash-test:integration" 2>/dev/null || true
  cd "${DNA_ROOT:?err}" || exit 1
  bash tests/teardown_mock.bash
  exit ${exit_code:-1}
}
trap dna::test_teardown_callback EXIT

cd "${N2ST_PATH:?'Variable not set'}" || exit 1
source "import_norlab_shell_script_tools_lib.bash" || exit 1
cd "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1

# ====begin========================================================================================
n2st::print_msg "Integration test: dna::squash_docker_image — metadata preservation"

# ....Pre-condition check.......................................................................
if ! command -v docker &>/dev/null; then
  n2st::print_msg_error "Docker is not available. This test requires a running Docker daemon."
  exit 1
fi

if ! docker info &>/dev/null; then
  n2st::print_msg_error "Docker daemon is not running. This test requires a running Docker daemon."
  exit 1
fi

if ! command -v python3 &>/dev/null; then
  n2st::print_msg_error "python3 is not available. This test requires python3 for JSON metadata parsing."
  exit 1
fi

# ....Step 1: Build minimal test image with known metadata.....................................
n2st::draw_horizontal_line_across_the_terminal_window "/"
n2st::print_msg "Step 1: Building minimal test Docker image with known metadata"

TEST_IMAGE="dna-squash-test:integration"
EXPECTED_ENV_VAR="DNA_TEST_VAR=hello_squash"
EXPECTED_WORKDIR="/dna/test/workdir"
EXPECTED_LABEL_KEY="org.dna.test.label"
EXPECTED_LABEL_VALUE="squash-integration-test"
EXPECTED_USER="nobody"
EXPECTED_ENTRYPOINT='["/bin/sh","-c"]'
EXPECTED_CMD='["echo hello"]'

TMPDIR_BUILD=$(mktemp -d)
cat > "${TMPDIR_BUILD}/Dockerfile" <<EOF
FROM busybox:latest
ENV ${EXPECTED_ENV_VAR}
WORKDIR ${EXPECTED_WORKDIR}
LABEL ${EXPECTED_LABEL_KEY}="${EXPECTED_LABEL_VALUE}"
USER ${EXPECTED_USER}
ENTRYPOINT ["/bin/sh","-c"]
CMD ["echo hello"]
EOF

docker build --no-cache -t "${TEST_IMAGE}" "${TMPDIR_BUILD}" || {
  rm -rf "${TMPDIR_BUILD}"
  n2st::print_msg_error "Failed to build test image"
  exit 1
}
rm -rf "${TMPDIR_BUILD}"
n2st::print_msg_done "Test image built: ${TEST_IMAGE}"

# ....Step 2: Record metadata before squash....................................................
n2st::draw_horizontal_line_across_the_terminal_window "/"
n2st::print_msg "Step 2: Recording metadata before squash"

BEFORE_ENV=$(docker inspect --format='{{json .Config.Env}}' "${TEST_IMAGE}")
BEFORE_ENTRYPOINT=$(docker inspect --format='{{json .Config.Entrypoint}}' "${TEST_IMAGE}")
BEFORE_CMD=$(docker inspect --format='{{json .Config.Cmd}}' "${TEST_IMAGE}")
BEFORE_WORKDIR=$(docker inspect --format='{{.Config.WorkingDir}}' "${TEST_IMAGE}")
BEFORE_USER=$(docker inspect --format='{{.Config.User}}' "${TEST_IMAGE}")
BEFORE_LABELS=$(docker inspect --format='{{json .Config.Labels}}' "${TEST_IMAGE}")

echo "  ENV:        ${BEFORE_ENV}"
echo "  ENTRYPOINT: ${BEFORE_ENTRYPOINT}"
echo "  CMD:        ${BEFORE_CMD}"
echo "  WORKDIR:    ${BEFORE_WORKDIR}"
echo "  USER:       ${BEFORE_USER}"
echo "  LABELS:     ${BEFORE_LABELS}"

# ....Step 3: Run squash.......................................................................
n2st::draw_horizontal_line_across_the_terminal_window "/"
n2st::print_msg "Step 3: Squashing image via dna::squash_docker_image"

source "${DNA_ROOT:?err}/src/lib/core/utils/import_dna_lib.bash" || exit 1
source "${DNA_ROOT:?err}/src/lib/core/utils/apptainer_tools.bash" || exit 1
dna::squash_docker_image "${TEST_IMAGE}" || {
  n2st::print_msg_error "dna::squash_docker_image failed"
  exit 1
}

# ....Step 4: Record metadata after squash.....................................................
n2st::draw_horizontal_line_across_the_terminal_window "/"
n2st::print_msg "Step 4: Recording metadata after squash and verifying preservation"

AFTER_ENV=$(docker inspect --format='{{json .Config.Env}}' "${TEST_IMAGE}")
AFTER_ENTRYPOINT=$(docker inspect --format='{{json .Config.Entrypoint}}' "${TEST_IMAGE}")
AFTER_CMD=$(docker inspect --format='{{json .Config.Cmd}}' "${TEST_IMAGE}")
AFTER_WORKDIR=$(docker inspect --format='{{.Config.WorkingDir}}' "${TEST_IMAGE}")
AFTER_USER=$(docker inspect --format='{{.Config.User}}' "${TEST_IMAGE}")
AFTER_LABELS=$(docker inspect --format='{{json .Config.Labels}}' "${TEST_IMAGE}")

echo "  ENV:        ${AFTER_ENV}"
echo "  ENTRYPOINT: ${AFTER_ENTRYPOINT}"
echo "  CMD:        ${AFTER_CMD}"
echo "  WORKDIR:    ${AFTER_WORKDIR}"
echo "  USER:       ${AFTER_USER}"
echo "  LABELS:     ${AFTER_LABELS}"

# ....Step 5: Assert metadata equality.........................................................
n2st::draw_horizontal_line_across_the_terminal_window "/"
n2st::print_msg "Step 5: Asserting metadata is preserved"

_test_pass=true

_assert_equal() {
  local label="$1"
  local before="$2"
  local after="$3"
  if [[ "${before}" == "${after}" ]]; then
    n2st::print_msg_done "  ${label}: preserved ✓"
  else
    n2st::print_msg_error "  ${label}: MISMATCH\n    Before: ${before}\n    After:  ${after}"
    _test_pass=false
  fi
}

# ENV: check expected var is present in after
if echo "${AFTER_ENV}" | python3 -c "
import json, sys
envs = json.load(sys.stdin)
assert any(e.startswith('DNA_TEST_VAR=hello_squash') for e in envs), 'DNA_TEST_VAR not found'
" 2>/dev/null; then
  n2st::print_msg_done "  ENV (DNA_TEST_VAR): preserved ✓"
else
  n2st::print_msg_error "  ENV (DNA_TEST_VAR): NOT preserved"
  _test_pass=false
fi

_assert_equal "ENTRYPOINT" "${BEFORE_ENTRYPOINT}" "${AFTER_ENTRYPOINT}"
_assert_equal "CMD"        "${BEFORE_CMD}"        "${AFTER_CMD}"
_assert_equal "WORKDIR"    "${BEFORE_WORKDIR}"    "${AFTER_WORKDIR}"
_assert_equal "USER"       "${BEFORE_USER}"       "${AFTER_USER}"

# LABELS: check expected label is present
if echo "${AFTER_LABELS}" | python3 -c "
import json, sys
labels = json.load(sys.stdin)
assert labels.get('${EXPECTED_LABEL_KEY}') == '${EXPECTED_LABEL_VALUE}', 'Label not found'
" 2>/dev/null; then
  n2st::print_msg_done "  LABEL (${EXPECTED_LABEL_KEY}): preserved ✓"
else
  n2st::print_msg_error "  LABEL (${EXPECTED_LABEL_KEY}): NOT preserved"
  _test_pass=false
fi

# ....Step 6: Verify single-layer (squashed)...................................................
n2st::draw_horizontal_line_across_the_terminal_window "/"
n2st::print_msg "Step 6: Verifying image is squashed to a single layer"

LAYER_COUNT=$(docker inspect --format='{{len .RootFS.Layers}}' "${TEST_IMAGE}")
echo "  Layer count after squash: ${LAYER_COUNT}"
if [[ "${LAYER_COUNT}" -eq 1 ]]; then
  n2st::print_msg_done "  Single layer confirmed ✓"
else
  n2st::print_msg_error "  Expected 1 layer, found: ${LAYER_COUNT}"
  _test_pass=false
fi

# ....Final result.............................................................................
n2st::draw_horizontal_line_across_the_terminal_window "="
if [[ "${_test_pass}" == true ]]; then
  n2st::print_msg_done "All metadata preservation assertions passed."
  exit 0
else
  n2st::print_msg_error "One or more metadata preservation assertions FAILED."
  exit 1
fi
