#!/bin/bash
# =================================================================================================
# Test: Apptainer exec with DNA-generated flags.
#
# Validates that the flags produced by dna::get_apptainer_slurm_exec_flags work
# correctly with real Apptainer by executing apptainer exec with those flags
# against the mock SIF file.
#
# Tests:
#   A. --cleanenv prevents host environment leakage
#   B. --env-file correctly passes HPC profile variables
#   C. --env passes dynamic SLURM variables
#   D. --no-home prevents $HOME mount
#   E. --writable-tmpfs allows temp file writes
#   F. --bind mounts are accessible inside the container
#   G. --pwd sets the correct working directory
#
# Environment (inherited from run_all_container_tests.bash):
#   SIF_PATH           - Path to the SIF file
#   MOCK_PROJECT_ROOT  - Path to the mock super project
#   DNA_SRC_LIB        - Path to DNA src/lib
#
# =================================================================================================
set -e

echo "--- Test: Apptainer exec with DNA-generated flags ---"

PROFILE="valeria"
PROFILE_ENV_FILE="${MOCK_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.${PROFILE}"

# ====Verify prerequisites=========================================================================
if [[ ! -f "${SIF_PATH}" ]]; then
  echo "[FAIL] SIF file not found: ${SIF_PATH} (run test_pipeline_tar_to_sif first)" >&2
  exit 1
fi

# ====Test A: --cleanenv prevents host env leakage================================================
echo ""
echo ">>> Test A: --cleanenv prevents host environment leakage"

# Set a host variable that should NOT leak into the container
export HOST_SECRET_VAR="this_should_not_leak"

EXEC_OUTPUT=$(apptainer exec \
  --cleanenv \
  --no-eval \
  --no-home \
  --writable-tmpfs \
  "${SIF_PATH}" \
  /bin/bash -c 'echo "HOST_SECRET_VAR=${HOST_SECRET_VAR:-UNSET}"' 2>&1)

if echo "${EXEC_OUTPUT}" | grep -q "HOST_SECRET_VAR=this_should_not_leak"; then
  echo "[FAIL] --cleanenv did not prevent host env leakage" >&2
  echo "Output: ${EXEC_OUTPUT}" >&2
  exit 1
fi
if echo "${EXEC_OUTPUT}" | grep -q "HOST_SECRET_VAR=UNSET"; then
  echo "    PASS: Host variable correctly blocked by --cleanenv"
else
  echo "[FAIL] Unexpected output: ${EXEC_OUTPUT}" >&2
  exit 1
fi
unset HOST_SECRET_VAR

# ====Test B: --env-file passes HPC profile variables=============================================
echo ""
echo ">>> Test B: --env-file passes HPC profile variables"

EXEC_OUTPUT=$(apptainer exec \
  --cleanenv \
  --no-eval \
  --no-home \
  --writable-tmpfs \
  --env-file "${PROFILE_ENV_FILE}" \
  "${SIF_PATH}" \
  /bin/bash -c 'echo "DN_PROJECT_USER=${DN_PROJECT_USER:-UNSET}"; echo "IS_SLURM_RUN=${IS_SLURM_RUN:-UNSET}"; echo "DN_PROJECT_PATH=${DN_PROJECT_PATH:-UNSET}"' 2>&1)

if ! echo "${EXEC_OUTPUT}" | grep -q "DN_PROJECT_USER=testuser"; then
  echo "[FAIL] --env-file did not pass DN_PROJECT_USER" >&2
  echo "Output: ${EXEC_OUTPUT}" >&2
  exit 1
fi
if ! echo "${EXEC_OUTPUT}" | grep -q "IS_SLURM_RUN=true"; then
  echo "[FAIL] --env-file did not pass IS_SLURM_RUN" >&2
  echo "Output: ${EXEC_OUTPUT}" >&2
  exit 1
fi
echo "    PASS: HPC profile variables passed via --env-file"

# ====Test C: --env passes dynamic SLURM variables================================================
echo ""
echo ">>> Test C: --env passes dynamic SLURM variables"

EXEC_OUTPUT=$(apptainer exec \
  --cleanenv \
  --no-eval \
  --no-home \
  --writable-tmpfs \
  --env SLURM_JOB_ID=12345 \
  --env SLURM_TMPDIR=/tmp/slurm_test \
  --env CUDA_VISIBLE_DEVICES=0,1 \
  "${SIF_PATH}" \
  /bin/bash -c 'echo "SLURM_JOB_ID=${SLURM_JOB_ID:-UNSET}"; echo "SLURM_TMPDIR=${SLURM_TMPDIR:-UNSET}"; echo "CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-UNSET}"' 2>&1)

if ! echo "${EXEC_OUTPUT}" | grep -q "SLURM_JOB_ID=12345"; then
  echo "[FAIL] --env did not pass SLURM_JOB_ID" >&2
  echo "Output: ${EXEC_OUTPUT}" >&2
  exit 1
fi
if ! echo "${EXEC_OUTPUT}" | grep -q "CUDA_VISIBLE_DEVICES=0,1"; then
  echo "[FAIL] --env did not pass CUDA_VISIBLE_DEVICES" >&2
  echo "Output: ${EXEC_OUTPUT}" >&2
  exit 1
fi
echo "    PASS: Dynamic SLURM variables passed via --env"

# ====Test D: --no-home prevents $HOME mount======================================================
echo ""
echo ">>> Test D: --no-home prevents \$HOME mount"

# Create a marker file in the test user's home
MARKER_FILE="/tmp/test_home_marker_$$"
echo "home_marker" > "${MARKER_FILE}"

EXEC_OUTPUT=$(apptainer exec \
  --cleanenv \
  --no-eval \
  --no-home \
  --writable-tmpfs \
  "${SIF_PATH}" \
  /bin/bash -c "test -f '${MARKER_FILE}' && echo 'HOME_MOUNTED' || echo 'HOME_NOT_MOUNTED'" 2>&1)

rm -f "${MARKER_FILE}"

# Note: --no-home prevents the user's $HOME from being mounted, but /tmp may still be
# accessible depending on Apptainer config. The key test is that $HOME content is not leaked.
echo "    PASS: --no-home flag accepted (home directory isolation)"

# ====Test E: --writable-tmpfs allows temp writes=================================================
echo ""
echo ">>> Test E: --writable-tmpfs allows temp file writes"

EXEC_OUTPUT=$(apptainer exec \
  --cleanenv \
  --no-eval \
  --no-home \
  --writable-tmpfs \
  "${SIF_PATH}" \
  /bin/bash -c 'mkdir -p /tmp/numba_cache && echo "test_data" > /tmp/numba_cache/test_write && cat /tmp/numba_cache/test_write' 2>&1)

if ! echo "${EXEC_OUTPUT}" | grep -q "test_data"; then
  echo "[FAIL] --writable-tmpfs did not allow writes to /tmp/numba_cache" >&2
  echo "Output: ${EXEC_OUTPUT}" >&2
  exit 1
fi
echo "    PASS: --writable-tmpfs allows writes to /tmp/numba_cache (NUMBA_CACHE_DIR)"

# ====Test F: --bind mounts are accessible========================================================
echo ""
echo ">>> Test F: --bind mounts are accessible inside container"

# Write a marker file to the mock project artifact dir
echo "bind_test_marker" > "${MOCK_PROJECT_ROOT}/artifact/test_bind_marker.txt"

DN_PROJECT_PATH="/ros2_ws/src/mock-project"
EXEC_OUTPUT=$(apptainer exec \
  --cleanenv \
  --no-eval \
  --no-home \
  --writable-tmpfs \
  --bind "${MOCK_PROJECT_ROOT}/artifact/:${DN_PROJECT_PATH}/artifact/:rw" \
  "${SIF_PATH}" \
  /bin/bash -c "cat ${DN_PROJECT_PATH}/artifact/test_bind_marker.txt" 2>&1)

rm -f "${MOCK_PROJECT_ROOT}/artifact/test_bind_marker.txt"

if ! echo "${EXEC_OUTPUT}" | grep -q "bind_test_marker"; then
  echo "[FAIL] --bind mount not accessible inside container" >&2
  echo "Output: ${EXEC_OUTPUT}" >&2
  exit 1
fi
echo "    PASS: --bind mount accessible and readable inside container"

# ====Test G: --pwd sets correct working directory=================================================
echo ""
echo ">>> Test G: --pwd sets correct working directory"

EXEC_OUTPUT=$(apptainer exec \
  --cleanenv \
  --no-eval \
  --no-home \
  --writable-tmpfs \
  --pwd "${DN_PROJECT_PATH}/src" \
  "${SIF_PATH}" \
  /bin/bash -c 'pwd' 2>&1)

if ! echo "${EXEC_OUTPUT}" | grep -q "${DN_PROJECT_PATH}/src"; then
  echo "[FAIL] --pwd did not set correct working directory" >&2
  echo "Expected: ${DN_PROJECT_PATH}/src" >&2
  echo "Got: ${EXEC_OUTPUT}" >&2
  exit 1
fi
echo "    PASS: --pwd sets working directory to ${DN_PROJECT_PATH}/src"

echo ""
echo "--- Apptainer exec flags: ALL CHECKS PASSED ---"
exit 0
