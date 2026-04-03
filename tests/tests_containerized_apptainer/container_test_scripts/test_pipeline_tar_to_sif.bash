#!/bin/bash
# =================================================================================================
# Test: Full pipeline — Docker tar archive → Apptainer SIF conversion.
#
# Validates the DNA build_sif.sh helper script by:
#   1. Using the DNA function to generate build_sif.sh
#   2. Running build_sif.sh to convert the mock slurm tar to SIF
#   3. Verifying the SIF file is valid via 'apptainer inspect'
#
# Environment (inherited from run_all_container_tests.bash):
#   MOCK_SLURM_TAR     - Filename of the mock slurm Docker tar archive
#   MOCK_PROJECT_ROOT  - Path to the mock super project
#   SIF_PATH           - Where the SIF file should be created
#   DNA_SRC_LIB        - Path to DNA src/lib
#
# =================================================================================================
set -e

echo "--- Test: Pipeline tar → SIF conversion ---"

TAR_PATH="${MOCK_PROJECT_ROOT}/artifact/apptainer/${MOCK_SLURM_TAR}"
SIF_DIR="$(dirname "${SIF_PATH}")"
SIF_NAME="$(basename "${SIF_PATH}")"

# ====Verify tar exists============================================================================
if [[ ! -f "${TAR_PATH}" ]]; then
  echo "[FAIL] Mock slurm tar not found: ${TAR_PATH}" >&2
  exit 1
fi
echo "[info] Mock slurm tar found: ${TAR_PATH} ($(du -h "${TAR_PATH}" | cut -f1))"

# ====Test A: Generate build_sif.sh using DNA function============================================
echo ""
echo ">>> Test A: Generate build_sif.sh via dna::generate_apptainer_build_sif_script"

# Source mock n2st functions (required by apptainer_tools.bash)
function n2st::print_msg() { echo "[MSG] $*"; }
function n2st::print_msg_error() { echo "[ERROR] $*" >&2; }
function n2st::print_msg_done() { echo "[DONE] $*"; }
function n2st::print_msg_warning() { echo "[WARNING] $*"; }
export -f n2st::print_msg n2st::print_msg_error n2st::print_msg_done n2st::print_msg_warning

export SUPER_PROJECT_ROOT="${MOCK_PROJECT_ROOT}"

# Source apptainer_tools
source "${DNA_SRC_LIB}/core/utils/apptainer_tools.bash"

# Generate build_sif.sh
dna::generate_apptainer_build_sif_script \
  "${MOCK_SLURM_TAR}" \
  "${SIF_NAME}" \
  "${SIF_DIR}"

if [[ ! -f "${SIF_DIR}/build_sif.sh" ]]; then
  echo "[FAIL] build_sif.sh was not created" >&2
  exit 1
fi
echo "    PASS: build_sif.sh generated at ${SIF_DIR}/build_sif.sh"

# Verify build_sif.sh content
if ! grep -q "apptainer build" "${SIF_DIR}/build_sif.sh"; then
  echo "[FAIL] build_sif.sh missing 'apptainer build' command" >&2
  exit 1
fi
if ! grep -q "docker-archive:" "${SIF_DIR}/build_sif.sh"; then
  echo "[FAIL] build_sif.sh missing 'docker-archive:' reference" >&2
  exit 1
fi
if ! grep -q "Apptainer >= 1.1.0" "${SIF_DIR}/build_sif.sh"; then
  echo "[FAIL] build_sif.sh missing version warning" >&2
  exit 1
fi
echo "    PASS: build_sif.sh content verified"

# ====Test B: Execute build_sif.sh to convert tar → SIF==========================================
echo ""
echo ">>> Test B: Execute build_sif.sh (tar → SIF conversion)"

# Remove any existing SIF from previous runs
rm -f "${SIF_PATH}"

cd "${SIF_DIR}"
bash build_sif.sh

if [[ ! -f "${SIF_PATH}" ]]; then
  echo "[FAIL] SIF file not created: ${SIF_PATH}" >&2
  exit 1
fi
echo "    PASS: SIF file created: ${SIF_PATH} ($(du -h "${SIF_PATH}" | cut -f1))"

# ====Test C: Verify SIF is valid via apptainer inspect==========================================
echo ""
echo ">>> Test C: Validate SIF via 'apptainer inspect'"

INSPECT_OUTPUT=$(apptainer inspect "${SIF_PATH}" 2>&1)
echo "${INSPECT_OUTPUT}"

if ! echo "${INSPECT_OUTPUT}" | grep -qi "dna.apptainer.compatible: true"; then
  echo "[FAIL] SIF missing dna.apptainer.compatible label" >&2
  exit 1
fi
echo "    PASS: SIF labels verified"

# ====Test D: Basic apptainer exec smoke test=====================================================
echo ""
echo ">>> Test D: Basic apptainer exec smoke test"

EXEC_OUTPUT=$(apptainer exec "${SIF_PATH}" /bin/bash -c 'echo "SIF_EXEC_OK"' 2>&1)
if ! echo "${EXEC_OUTPUT}" | grep -q "SIF_EXEC_OK"; then
  echo "[FAIL] Basic apptainer exec failed" >&2
  echo "Output: ${EXEC_OUTPUT}" >&2
  exit 1
fi
echo "    PASS: apptainer exec returns expected output"

echo ""
echo "--- Pipeline tar → SIF: ALL CHECKS PASSED ---"
exit 0
