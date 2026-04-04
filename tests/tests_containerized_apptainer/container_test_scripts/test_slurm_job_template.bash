#!/bin/bash
# =================================================================================================
# Test: Slurm job template execution with real Apptainer.
#
# Validates that slurm_job.apptainer.*.template.bash scripts produce correct
# apptainer exec commands by creating an adapted test version (stripping SBATCH
# directives) and executing against the mock SIF.
#
# Tests each profile template (valeria, compute_canada, mamba):
#   A. Template structure validation (SBATCH, apptainer exec, flags, profile source)
#   B. Adapted template execution against mock SIF
#   C. Environment variable pass-through verification
#
# Environment (inherited from run_all_container_tests.bash):
#   SIF_PATH           - Path to the SIF file
#   MOCK_PROJECT_ROOT  - Path to the mock super project
#   DNA_SRC_LIB        - Path to DNA src/lib
#
# =================================================================================================
set -e

echo "--- Test: Slurm job template execution ---"

TEMPLATE_DIR="${DNA_SRC_LIB}/template/slurm_jobs/template"

# ====Verify prerequisites=========================================================================
if [[ ! -f "${SIF_PATH}" ]]; then
  echo "[FAIL] SIF file not found: ${SIF_PATH}" >&2
  exit 1
fi

# ====Helper: adapt template for direct execution=================================================
# Strips SBATCH directives and patches paths/variables for test execution
function adapt_template_for_test() {
  local template_file="${1:?err}"
  local profile="${2:?err}"
  local output_file="${3:?err}"
  local profile_env_file="${MOCK_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.${profile}"

  # Read template, strip SBATCH lines, patch for test execution
  {
    echo "#!/bin/bash"
    echo "# Adapted from $(basename "${template_file}") for containerized testing"
    echo "set -e"
    echo ""
    echo "# Override paths for test environment"
    echo "export SUPER_PROJECT_ROOT=\"${MOCK_PROJECT_ROOT}\""
    echo "export SIF_PATH=\"${SIF_PATH}\""
    echo "PROFILE_ENV_FILE=\"${profile_env_file}\""
    echo ""
    echo "# Simulate SLURM environment"
    echo "export SLURM_JOB_ID=\"99999\""
    echo "export SLURM_TMPDIR=\"/tmp/slurm_test\""
    echo "export SLURM_JOB_NAME=\"test_${profile}\""
    echo "export SLURM_NODELIST=\"testnode01\""
    echo "export CUDA_VISIBLE_DEVICES=\"0\""
    echo ""
    echo "# Stub callbacks (defined in template header, stripped for test)"
    echo "function job_setup_callback() { echo '[test] job_setup_callback'; }"
    echo "function job_teardown_callback() { echo '[test] job_teardown_callback'; }"
    echo ""
    echo "# Override python_arguments for test"
    echo "python_arguments=(\"test_apptainer_env.py\")"
    echo "DNA_SJOB_NAME=\"99999\""
    echo ""
    # Extract the core logic from the template:
    # - Skip SBATCH lines and the original path/SIF_PATH definitions
    # - Keep everything from the source of profile env file onwards
    sed -n '/^# ====DNA internal/,$ p' "${template_file}" | \
      sed 's|PLACEHOLDER_DN_PROJECT_IMAGE_NAME|mock-project|g'
  } > "${output_file}"

  chmod +x "${output_file}"
}

# ====Test profiles================================================================================
PROFILES=("valeria" "compute_canada" "mamba")
TEMPLATE_PASS_COUNT=0

for profile in "${PROFILES[@]}"; do
  TEMPLATE_FILE="${TEMPLATE_DIR}/slurm_job.DNA_SJOB_NAME.apptainer.${profile}.bash"

  if [[ ! -f "${TEMPLATE_FILE}" ]]; then
    echo "[FAIL] Template not found: ${TEMPLATE_FILE}" >&2
    exit 1
  fi

  echo ""
  echo "========================================"
  echo " Profile: ${profile}"
  echo "========================================"

  # ====Test A: Template structure validation====================================================
  echo ""
  echo ">>> Test A (${profile}): Template structure validation"

  if ! grep -q "^#SBATCH" "${TEMPLATE_FILE}"; then
    echo "[FAIL] Template missing SBATCH directives" >&2
    exit 1
  fi

  if ! grep -q "apptainer exec" "${TEMPLATE_FILE}"; then
    echo "[FAIL] Template missing 'apptainer exec'" >&2
    exit 1
  fi

  if ! grep -q -- "--no-eval" "${TEMPLATE_FILE}"; then
    echo "[FAIL] Template missing --no-eval" >&2
    exit 1
  fi

  if ! grep -q -- "--cleanenv" "${TEMPLATE_FILE}"; then
    echo "[FAIL] Template missing --cleanenv" >&2
    exit 1
  fi

  if ! grep -q -- "--no-home" "${TEMPLATE_FILE}"; then
    echo "[FAIL] Template missing --no-home" >&2
    exit 1
  fi

  if ! grep -q -- "--writable-tmpfs" "${TEMPLATE_FILE}"; then
    echo "[FAIL] Template missing --writable-tmpfs" >&2
    exit 1
  fi

  if ! grep -q -- "--env-file" "${TEMPLATE_FILE}"; then
    echo "[FAIL] Template missing --env-file" >&2
    exit 1
  fi

  if ! grep -q -- "--env SLURM_JOB_ID" "${TEMPLATE_FILE}"; then
    echo "[FAIL] Template missing --env SLURM_JOB_ID" >&2
    exit 1
  fi

  if ! grep -q "dn_entrypoint.init.bash" "${TEMPLATE_FILE}"; then
    echo "[FAIL] Template missing entrypoint" >&2
    exit 1
  fi

  if ! grep -q "Apptainer >= 1.1.0" "${TEMPLATE_FILE}"; then
    echo "[FAIL] Template missing version warning" >&2
    exit 1
  fi

  if ! grep -q -- "--nv" "${TEMPLATE_FILE}"; then
    echo "[FAIL] Template missing --nv GPU flag" >&2
    exit 1
  fi

  if ! grep -q "job_setup_callback\|job_teardown_callback" "${TEMPLATE_FILE}"; then
    echo "[FAIL] Template missing setup/teardown callbacks" >&2
    exit 1
  fi

  echo "    PASS: Template structure correct"

  # ====Test B: Adapted template execution========================================================
  echo ""
  echo ">>> Test B (${profile}): Execute adapted template"

  ADAPTED_SCRIPT="/tmp/test_template_${profile}.bash"
  adapt_template_for_test "${TEMPLATE_FILE}" "${profile}" "${ADAPTED_SCRIPT}"

  EXEC_OUTPUT=$(bash "${ADAPTED_SCRIPT}" 2>&1) || {
    echo "[FAIL] Adapted template execution failed for ${profile} (exit code: $?)" >&2
    echo "Output:" >&2
    echo "${EXEC_OUTPUT}" >&2
    echo "" >&2
    echo "--- Adapted script content ---" >&2
    cat "${ADAPTED_SCRIPT}" >&2
    exit 1
  }

  if ! echo "${EXEC_OUTPUT}" | grep -q "APPTAINER_TEST_OK"; then
    echo "[FAIL] Template did not produce 'APPTAINER_TEST_OK'" >&2
    echo "Output: ${EXEC_OUTPUT}" >&2
    exit 1
  fi
  echo "    PASS: Adapted ${profile} template executed successfully"

  # ====Test C: Environment variable verification=================================================
  echo ""
  echo ">>> Test C (${profile}): Verify environment variables"

  if ! echo "${EXEC_OUTPUT}" | grep -q "IS_SLURM_RUN=true"; then
    echo "[FAIL] IS_SLURM_RUN not passed through for ${profile}" >&2
    echo "Output: ${EXEC_OUTPUT}" >&2
    exit 1
  fi

  if ! echo "${EXEC_OUTPUT}" | grep -q "DN_PROJECT_USER=testuser"; then
    echo "[FAIL] DN_PROJECT_USER not passed through for ${profile}" >&2
    echo "Output: ${EXEC_OUTPUT}" >&2
    exit 1
  fi

  echo "    PASS: Environment variables correct for ${profile}"

  # Cleanup
  rm -f "${ADAPTED_SCRIPT}"
  TEMPLATE_PASS_COUNT=$((TEMPLATE_PASS_COUNT + 1))
done

echo ""
echo "--- Slurm job templates: ALL ${TEMPLATE_PASS_COUNT}/${#PROFILES[@]} PROFILES PASSED ---"
exit 0
