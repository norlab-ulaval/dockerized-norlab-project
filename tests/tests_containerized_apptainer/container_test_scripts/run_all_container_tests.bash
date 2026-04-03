#!/bin/bash
# =================================================================================================
# Main test runner for containerized Apptainer tests.
# Runs inside the dna-apptainer-test-env Docker container.
#
# Environment (set by orchestrator):
#   MOCK_SLURM_TAR    - Filename of the mock slurm Docker tar archive
#   MOCK_PROJECT_ROOT - Path to the mock super project inside the container
#   DNA_SRC_LIB       - Path to DNA src/lib (mounted read-only)
#
# =================================================================================================
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ====Validate environment=========================================================================
echo "========================================================"
echo " Containerized Apptainer Test Suite"
echo " Apptainer version: $(apptainer --version)"
echo " Mock project root: ${MOCK_PROJECT_ROOT:?err}"
echo " Mock slurm tar:    ${MOCK_SLURM_TAR:?err}"
echo " DNA src lib:       ${DNA_SRC_LIB:?err}"
echo "========================================================"

# ====Shared state=================================================================================
# SIF file path — built by the first test and used by subsequent tests
export SIF_PATH="${MOCK_PROJECT_ROOT}/artifact/apptainer/mock-slurm.sif"
export SUPER_PROJECT_ROOT="${MOCK_PROJECT_ROOT}"

# ====Test execution===============================================================================
declare -i test_count=0
declare -i pass_count=0
declare -i fail_count=0
declare -a test_results=()

function run_test() {
  local test_script="${1:?err}"
  local test_name
  test_name="$(basename "${test_script}" .bash)"

  test_count+=1
  echo ""
  echo "========================================================"
  echo " Test ${test_count}: ${test_name}"
  echo "========================================================"

  if bash "${test_script}"; then
    pass_count+=1
    test_results+=("  ✅ ${test_name}")
    echo "    RESULT: PASS"
  else
    fail_count+=1
    test_results+=("  ❌ ${test_name}")
    echo "    RESULT: FAIL"
  fi
}

# ====Run tests in order===========================================================================
# Test 1: Build SIF from tar (must run first — creates the SIF used by later tests)
run_test "${SCRIPT_DIR}/test_pipeline_tar_to_sif.bash"

# Test 2: Basic apptainer exec with DNA flags
run_test "${SCRIPT_DIR}/test_apptainer_exec_flags.bash"

# Test 3: Generated run script execution
run_test "${SCRIPT_DIR}/test_generated_run_script.bash"

# Test 4: Slurm job template execution (simulated — no actual SLURM)
run_test "${SCRIPT_DIR}/test_slurm_job_template.bash"

# Test 5: Entrypoint runtime detection
run_test "${SCRIPT_DIR}/test_entrypoint_detection.bash"

# ====Summary======================================================================================
echo ""
echo "========================================================"
echo " Test Results: ${pass_count}/${test_count} passed, ${fail_count} failed"
echo "========================================================"
for result in "${test_results[@]}"; do
  echo "${result}"
done
echo ""

if [[ ${fail_count} -gt 0 ]]; then
  echo "FAILED"
  exit 1
fi

echo "ALL TESTS PASSED"
exit 0
