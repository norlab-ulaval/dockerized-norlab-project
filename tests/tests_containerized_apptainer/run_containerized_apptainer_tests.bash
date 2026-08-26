#!/bin/bash
# =================================================================================================
# Host-side orchestrator for containerized Apptainer integration tests.
#
# Builds a Docker test environment with Apptainer installed, creates a mock slurm image
# tar archive, and runs the full Apptainer pipeline test suite inside the container.
#
# This script is designed to run on macOS (or any Docker host) — Apptainer is never
# executed locally. All Apptainer operations happen inside the test container.
#
# Prerequisites:
#   - Docker Desktop running
#   - Internet access (for pulling ubuntu:22.04 and installing Apptainer)
#
# Usage:
#   $ bash tests/tests_containerized_apptainer/run_containerized_apptainer_tests.bash
#
# Options:
#   --no-cache    Force rebuild of test Docker images (no cache)
#   --keep        Keep test artifacts after completion (for debugging)
#   --skip-build  Skip Docker image build (use existing images)
#
# Environment:
#   DNA_APPTAINER_TEST_FORCE=true   Run the suite even on a non-amd64 host. By default the suite is
#                                   skipped (exit 0) on non-amd64 hosts because the amd64-only
#                                   Apptainer binary crashes under QEMU emulation (e.g. Apple Silicon).
#
# =================================================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# ====Configuration================================================================================
APPTAINER_TEST_ENV_IMAGE="dna-apptainer-test-env"
MOCK_SLURM_IMAGE="dna-mock-slurm-image"
MOCK_SLURM_TAR="mock-slurm-image.tar"
TEST_CONTAINER_NAME="dna-apptainer-test-run"

DOCKER_BUILD_OPTS=()
KEEP_ARTIFACTS=false
SKIP_BUILD=false

# ====Parse arguments==============================================================================
for arg in "$@"; do
  case "${arg}" in
    --no-cache)
      DOCKER_BUILD_OPTS+=("--no-cache")
      ;;
    --keep)
      KEEP_ARTIFACTS=true
      ;;
    --skip-build)
      SKIP_BUILD=true
      ;;
    *)
      echo "[error] Unknown option: ${arg}" >&2
      exit 1
      ;;
  esac
done

# ====Cleanup======================================================================================
ARTIFACTS_DIR="${SCRIPT_DIR}/.test_artifacts"

# Robustly remove the test artifacts directory.
# The suite runs the container --privileged (needed for Apptainer), so files it creates under the
# bind-mounted mock project (e.g. scratch/sif/mock-slurm.sif and scratch/tmp/*) are owned by root
# on the host. A plain host-side 'rm -rf' then fails with "Permission denied" — and because this
# script runs under 'set -e', that failure would abort cleanup() before it resets the exit code,
# turning an otherwise-green run into a spurious non-zero exit ([NBS error]). Remove any root-owned
# leftovers from inside a throwaway root container, and never let cleanup fail the suite over them.
function remove_artifacts_dir() {
  [[ -d "${ARTIFACTS_DIR}" ]] || return 0
  # Fast path: plain host removal (works when nothing is root-owned).
  if rm -rf "${ARTIFACTS_DIR}" 2>/dev/null; then
    return 0
  fi
  echo "[info] Some test artifacts are root-owned (created inside the privileged test container);" >&2
  echo "[info]   removing them via a throwaway root container." >&2
  local parent base
  parent="$(dirname "${ARTIFACTS_DIR}")"
  base="$(basename "${ARTIFACTS_DIR}")"
  docker run --rm --platform linux/amd64 \
    -v "${parent}:/host_parent:rw" \
    "${APPTAINER_TEST_ENV_IMAGE}" \
    /bin/sh -c "rm -rf '/host_parent/${base}'" 2>/dev/null || true
  # Best-effort final host cleanup; never fail the suite over leftover temp files.
  rm -rf "${ARTIFACTS_DIR}" 2>/dev/null || true
  return 0
}

function cleanup() {
  local exit_code=$?

  echo ""
  echo "========================================================"
  echo " Cleanup"
  echo "========================================================"

  # Remove test container if still running
  docker rm -f "${TEST_CONTAINER_NAME}" 2>/dev/null || true

  if [[ "${KEEP_ARTIFACTS}" == "false" ]]; then
    # Remove test artifacts
    remove_artifacts_dir
    # Remove Docker images
    docker rmi -f "${MOCK_SLURM_IMAGE}" 2>/dev/null || true
    echo "[info] Test artifacts cleaned up (use --keep to preserve)"
  else
    echo "[info] Test artifacts preserved at: ${ARTIFACTS_DIR}"
    echo "[info] Docker images preserved: ${APPTAINER_TEST_ENV_IMAGE}, ${MOCK_SLURM_IMAGE}"
  fi

  if [[ ${exit_code} -eq 0 ]]; then
    echo ""
    echo "========================================================"
    echo " ALL CONTAINERIZED APPTAINER TESTS PASSED"
    echo "========================================================"
  else
    echo ""
    echo "========================================================"
    echo " CONTAINERIZED APPTAINER TESTS FAILED (exit code: ${exit_code})"
    echo "========================================================"
  fi

  exit ${exit_code}
}
trap cleanup EXIT

echo "========================================================"
echo " Containerized Apptainer Integration Tests"
echo " Project root: ${PROJECT_ROOT}"
echo " Script dir:   ${SCRIPT_DIR}"
echo "========================================================"

# ====Host architecture guard======================================================================
# Apptainer publishes amd64-only .deb packages, so the test environment image is built for
# linux/amd64. On a non-amd64 host (e.g. Apple Silicon / arm64), Docker runs that image under
# QEMU user-mode emulation, where the Apptainer Go binary crashes with a fatal runtime error
# (`lfstack.push invalid packing` — QEMU does not preserve the upper pointer bits Go relies on).
# This is a host/QEMU emulation limitation, not a DNA defect, so skip the suite gracefully (exit 0)
# unless the host is natively amd64 or the user explicitly forces the run.
HOST_ARCH="$(uname -m)"
case "${HOST_ARCH}" in
  x86_64 | amd64) HOST_IS_AMD64=true ;;
  *) HOST_IS_AMD64=false ;;
esac

if [[ "${HOST_IS_AMD64}" == "false" && "${DNA_APPTAINER_TEST_FORCE:-false}" != "true" ]]; then
  # Disable the EXIT cleanup trap's failure banner by exiting 0 explicitly.
  trap - EXIT
  echo ""
  echo "========================================================"
  echo " SKIPPED: Containerized Apptainer tests"
  echo "--------------------------------------------------------"
  echo " Host architecture is '${HOST_ARCH}' (non-amd64)."
  echo " The Apptainer .deb is amd64-only; running it under QEMU"
  echo " emulation crashes the Apptainer Go binary."
  echo " Set DNA_APPTAINER_TEST_FORCE=true to run anyway (expected"
  echo " to fail under emulation), or run on a native amd64 host."
  echo "========================================================"
  # Best-effort cleanup of any leftover artifacts/container.
  docker rm -f "${TEST_CONTAINER_NAME}" 2>/dev/null || true
  if [[ "${KEEP_ARTIFACTS}" == "false" ]]; then
    remove_artifacts_dir
  fi
  exit 0
fi

# ====Step 1: Build test environment image=========================================================
if [[ "${SKIP_BUILD}" == "false" ]]; then
  echo ""
  echo ">>> Step 1/4: Building Apptainer test environment image (linux/amd64)..."
  docker build \
    --platform linux/amd64 \
    "${DOCKER_BUILD_OPTS[@]}" \
    -f "${SCRIPT_DIR}/Dockerfile.apptainer-test-env" \
    -t "${APPTAINER_TEST_ENV_IMAGE}" \
    --load \
    "${SCRIPT_DIR}"

  echo "    DONE: ${APPTAINER_TEST_ENV_IMAGE} built successfully"
else
  echo ""
  echo ">>> Step 1/4: Skipping image build (--skip-build)"
fi

# ====Step 2: Build mock slurm image and save as tar===============================================
if [[ "${SKIP_BUILD}" == "false" ]]; then
  echo ""
  echo ">>> Step 2/4: Building mock slurm image and saving as tar (linux/amd64)..."
  docker build \
    --platform linux/amd64 \
    "${DOCKER_BUILD_OPTS[@]}" \
    -f "${SCRIPT_DIR}/Dockerfile.mock-slurm-image" \
    -t "${MOCK_SLURM_IMAGE}" \
    --load \
    "${SCRIPT_DIR}"

  mkdir -p "${ARTIFACTS_DIR}"
  docker save "${MOCK_SLURM_IMAGE}" -o "${ARTIFACTS_DIR}/${MOCK_SLURM_TAR}"

  echo "    DONE: ${ARTIFACTS_DIR}/${MOCK_SLURM_TAR} created ($(du -h "${ARTIFACTS_DIR}/${MOCK_SLURM_TAR}" | cut -f1))"
else
  echo ""
  echo ">>> Step 2/4: Skipping mock image build (--skip-build)"
  if [[ ! -f "${ARTIFACTS_DIR}/${MOCK_SLURM_TAR}" ]]; then
    echo "[error] Mock slurm tar not found at ${ARTIFACTS_DIR}/${MOCK_SLURM_TAR}" >&2
    echo "[hint] Run without --skip-build first" >&2
    exit 1
  fi
fi

# ====Step 3: Prepare mock super project structure=================================================
echo ""
echo ">>> Step 3/4: Preparing mock super project structure..."

MOCK_SUPER_PROJECT="${ARTIFACTS_DIR}/mock_super_project"
mkdir -p "${MOCK_SUPER_PROJECT}/.dockerized_norlab/configuration/hpc_server_profile"
mkdir -p "${MOCK_SUPER_PROJECT}/.dockerized_norlab/configuration/entrypoints/project-slurm"
mkdir -p "${MOCK_SUPER_PROJECT}/.dockerized_norlab/dn_container_env_variable"
mkdir -p "${MOCK_SUPER_PROJECT}/artifact/apptainer"
mkdir -p "${MOCK_SUPER_PROJECT}/data/external_data"
mkdir -p "${MOCK_SUPER_PROJECT}/data/shared_data"
mkdir -p "${MOCK_SUPER_PROJECT}/src"
mkdir -p "${MOCK_SUPER_PROJECT}/utilities"

# Copy test python script into mock src/ so it is accessible when src/ is bind-mounted
# at runtime (--bind "${SUPER_PROJECT_ROOT}/src/:${DN_PROJECT_PATH}/src/:ro" would otherwise
# shadow the script that was baked into the mock slurm image).
cp "${SCRIPT_DIR}/test_apptainer_env.py" "${MOCK_SUPER_PROJECT}/src/test_apptainer_env.py"

# Create mock HPC profile env files
cat > "${MOCK_SUPER_PROJECT}/.dockerized_norlab/configuration/hpc_server_profile/.env.valeria" << 'ENVEOF'
# HPC Server Profile: Valeria (test mock)
DN_HOST=linux/x86
DN_PROJECT_USER=testuser
DN_PROJECT_PATH=/ros2_ws/src/mock-project
DN_PROJECT_GIT_NAME=mock-project
APPTAINER_CACHEDIR="${HOME}/.apptainer/cache"
APPTAINER_TMPDIR=/tmp
DN_ENTRYPOINT_TRACE_EXECUTION=true
IS_SLURM_RUN=true
APPTAINER_TARGET_PLATFORM=linux/amd64
APPTAINER_ENABLE_GPU=false
ENVEOF

cat > "${MOCK_SUPER_PROJECT}/.dockerized_norlab/configuration/hpc_server_profile/.env.compute_canada" << 'ENVEOF'
# HPC Server Profile: Compute Canada (test mock)
DN_HOST=linux/x86
DN_PROJECT_USER=testuser
DN_PROJECT_PATH=/ros2_ws/src/mock-project
DN_PROJECT_GIT_NAME=mock-project
APPTAINER_CACHEDIR="${SLURM_TMPDIR}/apptainer_cache"
APPTAINER_TMPDIR="${SLURM_TMPDIR}/apptainer_tmp"
DN_ENTRYPOINT_TRACE_EXECUTION=true
IS_SLURM_RUN=true
APPTAINER_TARGET_PLATFORM=linux/amd64
APPTAINER_ENABLE_GPU=false
ENVEOF

cat > "${MOCK_SUPER_PROJECT}/.dockerized_norlab/configuration/hpc_server_profile/.env.mamba" << 'ENVEOF'
# HPC Server Profile: Mamba (test mock)
DN_HOST=linux/x86
DN_PROJECT_USER=testuser
DN_PROJECT_PATH=/ros2_ws/src/mock-project
DN_PROJECT_GIT_NAME=mock-project
APPTAINER_CACHEDIR="${HOME}/.apptainer/cache"
APPTAINER_TMPDIR=/tmp
DN_ENTRYPOINT_TRACE_EXECUTION=true
IS_SLURM_RUN=true
APPTAINER_TARGET_PLATFORM=linux/amd64
APPTAINER_ENABLE_GPU=false
ENVEOF

# Create empty entrypoint callbacks (required by real entrypoint)
touch "${MOCK_SUPER_PROJECT}/.dockerized_norlab/configuration/entrypoints/project-slurm/dn_entrypoint.init.callback.bash"
touch "${MOCK_SUPER_PROJECT}/.dockerized_norlab/configuration/entrypoints/dn_entrypoint.global.init.callback.bash"

# Copy mock slurm tar to artifact dir
cp "${ARTIFACTS_DIR}/${MOCK_SLURM_TAR}" "${MOCK_SUPER_PROJECT}/artifact/apptainer/${MOCK_SLURM_TAR}"

echo "    DONE: Mock super project prepared at ${MOCK_SUPER_PROJECT}"

# ====Step 4: Run containerized tests=============================================================
echo ""
echo ">>> Step 4/4: Running containerized Apptainer tests..."
echo ""

docker run \
  --rm \
  --privileged \
  --platform linux/amd64 \
  --name "${TEST_CONTAINER_NAME}" \
  -v "${SCRIPT_DIR}/container_test_scripts:/workspace/container_test_scripts:ro" \
  -v "${MOCK_SUPER_PROJECT}:/mock_project:rw" \
  -v "${PROJECT_ROOT}/src/lib:/workspace/dna_src_lib:ro" \
  -e MOCK_SLURM_TAR="${MOCK_SLURM_TAR}" \
  -e MOCK_PROJECT_ROOT="/mock_project" \
  -e DNA_SRC_LIB="/workspace/dna_src_lib" \
  "${APPTAINER_TEST_ENV_IMAGE}" \
  /bin/bash /workspace/container_test_scripts/run_all_container_tests.bash
