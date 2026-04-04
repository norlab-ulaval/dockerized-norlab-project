#!/bin/bash
# =================================================================================================
# Test: DNA-generated run script execution with real Apptainer.
#
# Validates the output of 'dna run slurm <sjob-id> --generate-apptainer <profile>'
# by generating a run script using dna::generate_apptainer_run_script, then executing
# it with real Apptainer against the mock SIF.
#
# Tests:
#   A. Generate run script via dna::generate_apptainer_run_script
#   B. Verify run script structure (apptainer exec, flags, entrypoint, version warning)
#   C. Execute the generated run script against the mock SIF
#   D. Verify python args are passed through correctly
#   E. Test --print-only equivalent (dna::print_apptainer_exec_command output)
#
# Environment (inherited from run_all_container_tests.bash):
#   SIF_PATH           - Path to the SIF file
#   MOCK_PROJECT_ROOT  - Path to the mock super project
#   SUPER_PROJECT_ROOT - Same as MOCK_PROJECT_ROOT
#   DNA_SRC_LIB        - Path to DNA src/lib
#
# =================================================================================================
set -e

echo "--- Test: Generated run script execution ---"

PROFILE="valeria"
DNA_SJOB_NAME="TEST-001"
OUTPUT_DIR="${MOCK_PROJECT_ROOT}/artifact/apptainer"

# ====Setup========================================================================================
# Source mock n2st functions
function n2st::print_msg() { echo "[MSG] $*"; }
function n2st::print_msg_error() { echo "[ERROR] $*" >&2; }
function n2st::print_msg_done() { echo "[DONE] $*"; }
function n2st::print_msg_warning() { echo "[WARNING] $*"; }
export -f n2st::print_msg n2st::print_msg_error n2st::print_msg_done n2st::print_msg_warning

export SUPER_PROJECT_ROOT="${MOCK_PROJECT_ROOT}"
export DN_PROJECT_IMAGE_NAME="mock-project"
export PROJECT_TAG="test"

# Source apptainer_tools
source "${DNA_SRC_LIB}/core/utils/apptainer_tools.bash"

# ====Verify prerequisites=========================================================================
if [[ ! -f "${SIF_PATH}" ]]; then
  echo "[FAIL] SIF file not found: ${SIF_PATH}" >&2
  exit 1
fi

# ====Test A: Generate run script==================================================================
echo ""
echo ">>> Test A: Generate run script via dna::generate_apptainer_run_script"

SCRIPT_OUTPUT=$(dna::generate_apptainer_run_script \
  "${DNA_SJOB_NAME}" \
  "${PROFILE}" \
  "${SIF_PATH}" \
  "${OUTPUT_DIR}" \
  "test_apptainer_env.py")

RUN_SCRIPT="${OUTPUT_DIR}/run_apptainer_${DNA_SJOB_NAME}.sh"

if [[ ! -f "${RUN_SCRIPT}" ]]; then
  echo "[FAIL] Run script not created: ${RUN_SCRIPT}" >&2
  exit 1
fi
echo "    PASS: Run script generated at ${RUN_SCRIPT}"

# ====Test B: Verify run script structure==========================================================
echo ""
echo ">>> Test B: Verify run script structure"

# Check for essential components
if ! grep -q "apptainer exec" "${RUN_SCRIPT}"; then
  echo "[FAIL] Missing 'apptainer exec' in run script" >&2
  exit 1
fi

if ! grep -q -- "--no-eval" "${RUN_SCRIPT}"; then
  echo "[FAIL] Missing --no-eval flag" >&2
  exit 1
fi

if ! grep -q -- "--cleanenv" "${RUN_SCRIPT}"; then
  echo "[FAIL] Missing --cleanenv flag" >&2
  exit 1
fi

if ! grep -q -- "--no-home" "${RUN_SCRIPT}"; then
  echo "[FAIL] Missing --no-home flag" >&2
  exit 1
fi

if ! grep -q -- "--writable-tmpfs" "${RUN_SCRIPT}"; then
  echo "[FAIL] Missing --writable-tmpfs flag" >&2
  exit 1
fi

if ! grep -q -- "--env-file" "${RUN_SCRIPT}"; then
  echo "[FAIL] Missing --env-file flag" >&2
  exit 1
fi

if ! grep -q "dn_entrypoint.init.bash" "${RUN_SCRIPT}"; then
  echo "[FAIL] Missing entrypoint reference" >&2
  exit 1
fi

if ! grep -q "Apptainer >= 1.1.0" "${RUN_SCRIPT}"; then
  echo "[FAIL] Missing version warning" >&2
  exit 1
fi

if ! grep -q "test_apptainer_env.py" "${RUN_SCRIPT}"; then
  echo "[FAIL] Missing python args in run script" >&2
  exit 1
fi

echo "    PASS: Run script structure verified (all required flags, entrypoint, version warning, python args)"

# ====Test C: Execute the generated run script=====================================================
echo ""
echo ">>> Test C: Execute generated run script against mock SIF"

# The run script sources the profile env file and runs apptainer exec.
# We need to set SUPER_PROJECT_ROOT and SIF_PATH for the script.
cd "${MOCK_PROJECT_ROOT}"
export SUPER_PROJECT_ROOT="${MOCK_PROJECT_ROOT}"
export SIF_PATH="${SIF_PATH}"

# Simulate SLURM environment variables that the script passes via --env
export SLURM_JOB_ID="12345"
export SLURM_TMPDIR="/tmp/slurm_test"
export SLURM_JOB_NAME="test_job"
export SLURM_NODELIST="node01"
export CUDA_VISIBLE_DEVICES="0"

EXEC_OUTPUT=$(bash "${RUN_SCRIPT}" 2>&1) || {
  echo "[FAIL] Run script execution failed (exit code: $?)" >&2
  echo "Output:" >&2
  echo "${EXEC_OUTPUT}" >&2
  exit 1
}

if ! echo "${EXEC_OUTPUT}" | grep -q "APPTAINER_TEST_OK"; then
  echo "[FAIL] Run script did not produce expected output 'APPTAINER_TEST_OK'" >&2
  echo "Output: ${EXEC_OUTPUT}" >&2
  exit 1
fi
echo "    PASS: Generated run script executed successfully"

# ====Test D: Verify environment variables in output===============================================
echo ""
echo ">>> Test D: Verify environment variables passed through"

if ! echo "${EXEC_OUTPUT}" | grep -q "IS_SLURM_RUN=true"; then
  echo "[FAIL] IS_SLURM_RUN not passed through" >&2
  echo "Output: ${EXEC_OUTPUT}" >&2
  exit 1
fi

if ! echo "${EXEC_OUTPUT}" | grep -q "DN_PROJECT_USER=testuser"; then
  echo "[FAIL] DN_PROJECT_USER not passed through" >&2
  echo "Output: ${EXEC_OUTPUT}" >&2
  exit 1
fi

echo "    PASS: Environment variables correctly passed to container"

# ====Test E: print_apptainer_exec_command output==================================================
echo ""
echo ">>> Test E: dna::print_apptainer_exec_command output"

CMD_OUTPUT=$(dna::print_apptainer_exec_command \
  "${PROFILE}" \
  "${SIF_PATH}" \
  "test_apptainer_env.py")

if ! echo "${CMD_OUTPUT}" | grep -q "apptainer exec"; then
  echo "[FAIL] print_apptainer_exec_command missing 'apptainer exec'" >&2
  exit 1
fi

if ! echo "${CMD_OUTPUT}" | grep -q -- "--no-eval"; then
  echo "[FAIL] print_apptainer_exec_command missing --no-eval" >&2
  exit 1
fi

if ! echo "${CMD_OUTPUT}" | grep -q -- "--cleanenv"; then
  echo "[FAIL] print_apptainer_exec_command missing --cleanenv" >&2
  exit 1
fi

if ! echo "${CMD_OUTPUT}" | grep -q "dn_entrypoint.init.bash"; then
  echo "[FAIL] print_apptainer_exec_command missing entrypoint" >&2
  exit 1
fi

echo "    PASS: print_apptainer_exec_command output correct"

echo ""
echo "--- Generated run script: ALL CHECKS PASSED ---"
exit 0
