#!/bin/bash
# =================================================================================================
# Dry-run integration test for Apptainer command generation workflow.
#
# Validates the full apptainer command generation pipeline without executing 'apptainer' locally.
# Tests are macOS-compatible — no Apptainer binary required.
#
# Usage:
#   $ bash tests/tests_dryrun_and_tests_scripts/dryrun_apptainer_command_generation.bash
#
# =================================================================================================
set -e

SCRIPT_PATH="$(realpath -q "${BASH_SOURCE[0]:-.}")"
SCRIPT_PATH_PARENT="$(dirname "${SCRIPT_PATH}")"
PROJECT_ROOT="$(cd "${SCRIPT_PATH_PARENT}/../.." && pwd)"

echo "========================================================"
echo " Dry-run: Apptainer command generation"
echo " Project root: ${PROJECT_ROOT}"
echo "========================================================"

# ....Setup temp environment......................................................................
TEMP_DIR=$(mktemp -d)
MOCK_PROJECT_ROOT="${TEMP_DIR}/mock_super_project"

cleanup() {
  rm -rf "${TEMP_DIR}"
}
trap cleanup EXIT

mkdir -p "${MOCK_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile"
mkdir -p "${MOCK_PROJECT_ROOT}/artifact/apptainer"
mkdir -p "${MOCK_PROJECT_ROOT}/data/external_data"
mkdir -p "${MOCK_PROJECT_ROOT}/data/shared_data"

# Create a mock valeria profile env file
cat > "${MOCK_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.valeria" << 'EOF'
DN_HOST=linux/x86
DN_PROJECT_PATH=/ros2_ws/src/my-project
APPTAINER_CACHEDIR="${HOME}/.apptainer/cache"
APPTAINER_TMPDIR=/tmp
DN_ENTRYPOINT_TRACE_EXECUTION=false
IS_SLURM_RUN=true
EOF

# Create mock n2st functions
function n2st::print_msg() { echo "[MSG] $*"; }
function n2st::print_msg_error() { echo "[ERROR] $*" >&2; }
function n2st::print_msg_done() { echo "[DONE] $*"; }
function n2st::print_msg_warning() { echo "[WARNING] $*"; }
export -f n2st::print_msg n2st::print_msg_error n2st::print_msg_done n2st::print_msg_warning

export SUPER_PROJECT_ROOT="${MOCK_PROJECT_ROOT}"
export DN_PROJECT_IMAGE_NAME="my-project"
export PROJECT_TAG="l4t-r36.4.0"
export DNA_LIB_PATH="${PROJECT_ROOT}/src/lib"

# ....Source apptainer_tools......................................................................
echo ""
echo ">>> Test 1: Source apptainer_tools.bash"
source "${PROJECT_ROOT}/src/lib/core/utils/apptainer_tools.bash"
echo "    PASS: apptainer_tools.bash sourced successfully"

# ....Test 1: check_apptainer_profile_env_file............................................
echo ""
echo ">>> Test 2: dna::check_apptainer_profile_env_file with valid profile"
dna::check_apptainer_profile_env_file "valeria"
echo "    PASS: valeria profile env file found"

echo ""
echo ">>> Test 3: dna::check_apptainer_profile_env_file with invalid profile"
if dna::check_apptainer_profile_env_file "nonexistent" 2>/dev/null; then
  echo "    FAIL: Should have failed for nonexistent profile" >&2
  exit 1
else
  echo "    PASS: Correctly failed for nonexistent profile"
fi

# ....Test 2: generate_apptainer_build_sif_script.........................................
echo ""
echo ">>> Test 4: dna::generate_apptainer_build_sif_script"
dna::generate_apptainer_build_sif_script \
  "my-project-slurm.l4t-r36.4.0.tar" \
  "my-project-slurm.sif" \
  "${TEMP_DIR}"

if [[ ! -f "${TEMP_DIR}/build_sif.sh" ]]; then
  echo "    FAIL: build_sif.sh not created" >&2
  exit 1
fi
if ! grep -q "apptainer build" "${TEMP_DIR}/build_sif.sh"; then
  echo "    FAIL: build_sif.sh missing 'apptainer build'" >&2
  exit 1
fi
if ! grep -q "docker-archive:" "${TEMP_DIR}/build_sif.sh"; then
  echo "    FAIL: build_sif.sh missing 'docker-archive:'" >&2
  exit 1
fi
echo "    PASS: build_sif.sh created with correct content"

# ....Test 3: get_apptainer_slurm_exec_flags..............................................
echo ""
echo ">>> Test 5: dna::get_apptainer_slurm_exec_flags"
FLAGS=$(dna::get_apptainer_slurm_exec_flags "valeria" "artifact/apptainer/my-project-slurm.sif")

if ! echo "${FLAGS}" | grep -q -- "--no-eval"; then
  echo "    FAIL: Missing --no-eval flag" >&2; exit 1
fi
if ! echo "${FLAGS}" | grep -q -- "--cleanenv"; then
  echo "    FAIL: Missing --cleanenv flag" >&2; exit 1
fi
if ! echo "${FLAGS}" | grep -q -- "--no-home"; then
  echo "    FAIL: Missing --no-home flag" >&2; exit 1
fi
if ! echo "${FLAGS}" | grep -q -- "--nv"; then
  echo "    FAIL: Missing --nv flag" >&2; exit 1
fi
if ! echo "${FLAGS}" | grep -q -- "--bind"; then
  echo "    FAIL: Missing --bind flags" >&2; exit 1
fi
if ! echo "${FLAGS}" | grep -q -- "--env-file"; then
  echo "    FAIL: Missing --env-file flag" >&2; exit 1
fi
if ! echo "${FLAGS}" | grep -q -- "--env CUDA_VISIBLE_DEVICES="; then
  echo "    FAIL: Missing --env CUDA_VISIBLE_DEVICES" >&2; exit 1
fi
if ! echo "${FLAGS}" | grep -q -- "--env SLURM_JOB_ID="; then
  echo "    FAIL: Missing --env SLURM_JOB_ID" >&2; exit 1
fi
if ! echo "${FLAGS}" | grep -q -- "--writable-tmpfs"; then
  echo "    FAIL: Missing --writable-tmpfs flag" >&2; exit 1
fi
if echo "${FLAGS}" | grep -q ".X11-unix"; then
  echo "    FAIL: X11 bind mount should NOT be present for HPC" >&2; exit 1
fi
echo "    PASS: exec flags contain expected Apptainer flags (--no-eval, --cleanenv, --no-home, --nv, --env, --env-file) and no X11 mounts"

# ....Test 4: generate_apptainer_run_script..............................................
echo ""
echo ">>> Test 6: dna::generate_apptainer_run_script"
SCRIPT_PATH_OUT=$(dna::generate_apptainer_run_script \
  "NMO-001" \
  "valeria" \
  "artifact/apptainer/my-project-slurm.sif" \
  "${TEMP_DIR}" \
  "launcher/train.py" "--epochs=10")

if [[ ! -f "${TEMP_DIR}/run_apptainer_NMO-001.sh" ]]; then
  echo "    FAIL: run_apptainer_NMO-001.sh not created" >&2; exit 1
fi
if ! grep -q "apptainer exec" "${TEMP_DIR}/run_apptainer_NMO-001.sh"; then
  echo "    FAIL: Missing 'apptainer exec' in run script" >&2; exit 1
fi
if ! grep -q "dn_entrypoint.init.bash" "${TEMP_DIR}/run_apptainer_NMO-001.sh"; then
  echo "    FAIL: Missing entrypoint in run script" >&2; exit 1
fi
if ! grep -q "launcher/train.py" "${TEMP_DIR}/run_apptainer_NMO-001.sh"; then
  echo "    FAIL: Missing python args in run script" >&2; exit 1
fi
if ! grep -q "Apptainer >= 1.1.0" "${TEMP_DIR}/run_apptainer_NMO-001.sh"; then
  echo "    FAIL: Missing Apptainer version warning in run script" >&2; exit 1
fi
echo "    PASS: run_apptainer_NMO-001.sh created with correct content and version warning"

# ....Test 5: print_apptainer_exec_command output.........................................
echo ""
echo ">>> Test 7: dna::print_apptainer_exec_command"
CMD_OUTPUT=$(dna::print_apptainer_exec_command \
  "valeria" \
  "artifact/apptainer/my-project-slurm.sif" \
  "launcher/train.py")

if ! echo "${CMD_OUTPUT}" | grep -q "apptainer exec"; then
  echo "    FAIL: Missing 'apptainer exec' in command output" >&2; exit 1
fi
if ! echo "${CMD_OUTPUT}" | grep -q "dn_entrypoint.init.bash"; then
  echo "    FAIL: Missing entrypoint in command output" >&2; exit 1
fi
echo "    PASS: print_apptainer_exec_command output is correct"

# ....Test 6: compute_canada profile.......................................................
echo ""
echo ">>> Test 8: compute_canada profile flags"
CCFLAGS=$(dna::get_apptainer_slurm_exec_flags "compute_canada" "artifact/apptainer/my-project-slurm.sif")
if ! echo "${CCFLAGS}" | grep -q ".env.compute_canada"; then
  echo "    FAIL: Missing .env.compute_canada in flags" >&2; exit 1
fi
echo "    PASS: compute_canada profile uses correct env file"

# ....Test 7: APPTAINER_ENABLE_GPU=false disables --nv......................................
echo ""
echo ">>> Test 9: APPTAINER_ENABLE_GPU=false disables --nv flag"
export APPTAINER_ENABLE_GPU=false
GPU_OFF_FLAGS=$(dna::get_apptainer_slurm_exec_flags "valeria" "artifact/apptainer/my-project-slurm.sif")
if echo "${GPU_OFF_FLAGS}" | grep -q -- "--nv"; then
  echo "    FAIL: --nv flag should NOT be present when APPTAINER_ENABLE_GPU=false" >&2; exit 1
fi
echo "    PASS: --nv flag correctly omitted when APPTAINER_ENABLE_GPU=false"
unset APPTAINER_ENABLE_GPU

# ....Test 8: build_sif.sh contains version warning........................................
echo ""
echo ">>> Test 10: build_sif.sh contains Apptainer version warning"
if ! grep -q "Apptainer >= 1.1.0" "${TEMP_DIR}/build_sif.sh"; then
  echo "    FAIL: Missing Apptainer version warning in build_sif.sh" >&2; exit 1
fi
echo "    PASS: build_sif.sh contains Apptainer version warning"

echo ""
echo "========================================================"
echo " ALL DRY-RUN TESTS PASSED"
echo "========================================================"
exit 0
