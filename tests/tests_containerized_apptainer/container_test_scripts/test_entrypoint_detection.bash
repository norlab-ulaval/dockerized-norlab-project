#!/bin/bash
# =================================================================================================
# Test: Entrypoint Apptainer runtime detection.
#
# Validates that the mock entrypoint (mirroring dn_entrypoint.init.bash behavior)
# correctly detects it is running inside Apptainer via the APPTAINER_CONTAINER
# environment variable that Apptainer automatically sets.
#
# Tests:
#   A. DNA_RUNTIME is set to "apptainer" when running via apptainer exec
#   B. APPTAINER_CONTAINER env var is automatically set by Apptainer
#   C. Entrypoint runs python3 with correct arguments
#   D. DN_ENTRYPOINT_TRACE_EXECUTION produces trace output
#   E. Multiple python arguments are passed correctly
#
# Environment (inherited from run_all_container_tests.bash):
#   SIF_PATH           - Path to the SIF file
#   MOCK_PROJECT_ROOT  - Path to the mock super project
#
# =================================================================================================
set -e

echo "--- Test: Entrypoint Apptainer runtime detection ---"

PROFILE_ENV_FILE="${MOCK_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.valeria"

# ====Verify prerequisites=========================================================================
if [[ ! -f "${SIF_PATH}" ]]; then
  echo "[FAIL] SIF file not found: ${SIF_PATH}" >&2
  exit 1
fi

# ====Test A: DNA_RUNTIME detection================================================================
echo ""
echo ">>> Test A: DNA_RUNTIME is set to 'apptainer' inside Apptainer"

EXEC_OUTPUT=$(apptainer exec \
  --cleanenv \
  --no-eval \
  --no-home \
  --writable-tmpfs \
  --env-file "${PROFILE_ENV_FILE}" \
  "${SIF_PATH}" \
  /dockerized-norlab/project/project-slurm/dn_entrypoint.init.bash \
  -c "import os; print(f'DNA_RUNTIME={os.environ.get(\"DNA_RUNTIME\", \"UNSET\")}')" 2>&1)

if ! echo "${EXEC_OUTPUT}" | grep -q "DNA_RUNTIME=apptainer"; then
  echo "[FAIL] DNA_RUNTIME not set to 'apptainer'" >&2
  echo "Output: ${EXEC_OUTPUT}" >&2
  exit 1
fi
echo "    PASS: DNA_RUNTIME=apptainer detected correctly"

# ====Test B: APPTAINER_CONTAINER is set automatically=============================================
echo ""
echo ">>> Test B: APPTAINER_CONTAINER is automatically set by Apptainer"

# Note: --cleanenv blocks most vars but Apptainer always injects its own vars
EXEC_OUTPUT=$(apptainer exec \
  --no-eval \
  --no-home \
  --writable-tmpfs \
  "${SIF_PATH}" \
  /bin/bash -c 'echo "APPTAINER_CONTAINER=${APPTAINER_CONTAINER:-UNSET}"' 2>&1)

if echo "${EXEC_OUTPUT}" | grep -q "APPTAINER_CONTAINER=UNSET"; then
  echo "[FAIL] APPTAINER_CONTAINER not set by Apptainer" >&2
  echo "Output: ${EXEC_OUTPUT}" >&2
  exit 1
fi
echo "    PASS: APPTAINER_CONTAINER is automatically set"

# ====Test C: Python args pass-through=============================================================
echo ""
echo ">>> Test C: Python arguments are passed through entrypoint"

EXEC_OUTPUT=$(apptainer exec \
  --cleanenv \
  --no-eval \
  --no-home \
  --writable-tmpfs \
  --env-file "${PROFILE_ENV_FILE}" \
  "${SIF_PATH}" \
  /dockerized-norlab/project/project-slurm/dn_entrypoint.init.bash \
  "test_apptainer_env.py" 2>&1)

if ! echo "${EXEC_OUTPUT}" | grep -q "APPTAINER_TEST_OK"; then
  echo "[FAIL] test_apptainer_env.py did not produce APPTAINER_TEST_OK" >&2
  echo "Output: ${EXEC_OUTPUT}" >&2
  exit 1
fi

if ! echo "${EXEC_OUTPUT}" | grep -q "DN_PROJECT_PATH=/ros2_ws/src/mock-project"; then
  echo "[FAIL] DN_PROJECT_PATH not set correctly" >&2
  echo "Output: ${EXEC_OUTPUT}" >&2
  exit 1
fi

echo "    PASS: Python script executed through entrypoint with correct env"

# ====Test D: Trace execution output===============================================================
echo ""
echo ">>> Test D: DN_ENTRYPOINT_TRACE_EXECUTION produces trace output"

# The valeria profile has DN_ENTRYPOINT_TRACE_EXECUTION=true
EXEC_OUTPUT=$(apptainer exec \
  --cleanenv \
  --no-eval \
  --no-home \
  --writable-tmpfs \
  --env-file "${PROFILE_ENV_FILE}" \
  "${SIF_PATH}" \
  /dockerized-norlab/project/project-slurm/dn_entrypoint.init.bash \
  -c "print('trace_test')" 2>&1)

if ! echo "${EXEC_OUTPUT}" | grep -q "\[DN trace\]"; then
  echo "[FAIL] Trace execution output not found" >&2
  echo "Output: ${EXEC_OUTPUT}" >&2
  exit 1
fi
echo "    PASS: Trace execution output present"

# ====Test E: Multiple python arguments============================================================
echo ""
echo ">>> Test E: Multiple python arguments pass-through"

EXEC_OUTPUT=$(apptainer exec \
  --cleanenv \
  --no-eval \
  --no-home \
  --writable-tmpfs \
  --env-file "${PROFILE_ENV_FILE}" \
  "${SIF_PATH}" \
  /dockerized-norlab/project/project-slurm/dn_entrypoint.init.bash \
  -c "import sys; print(f'ARGS={sys.argv[1:]}')" "--arg1" "--arg2=value" 2>&1)

if ! echo "${EXEC_OUTPUT}" | grep -q "ARGS=\['--arg1', '--arg2=value'\]"; then
  echo "[FAIL] Multiple arguments not passed correctly" >&2
  echo "Output: ${EXEC_OUTPUT}" >&2
  exit 1
fi
echo "    PASS: Multiple python arguments passed correctly"

echo ""
echo "--- Entrypoint detection: ALL CHECKS PASSED ---"
exit 0
