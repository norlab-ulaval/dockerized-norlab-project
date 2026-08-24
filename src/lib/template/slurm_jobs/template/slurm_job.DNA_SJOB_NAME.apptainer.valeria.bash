#!/bin/bash
#
#SBATCH --partition=gpu
#SBATCH --gres=gpu:1
#SBATCH --mem=32g
#SBATCH --cpus-per-task=12
#SBATCH --time=0-24:00
#SBATCH --output=artifact/slurm_jobs_logs/%x-%j.out
#
# Note:
# - Flag time format --time=D-HH:MM ->  D=day, HH=hours, MM=minutes
# =================================================================================================
# Execute Apptainer slurm job on Valeria HPC server.
#
# Standalone script — does NOT require DNA to be installed on the Valeria server.
# The apptainer exec command is executed directly using the pre-built SIF file.
#
# Workflow:
#   Two pipelines are available. Choose the one that fits your setup:
#
#   Pipeline A — tar archive (--save): build image locally, transfer tar, convert to SIF on HPC.
#     Local (macOS):
#       1. Build:    dna build slurm --apptainer valeria --save
#                    → builds Docker image, saves tar archive, generates dna_tar_to_apptainer_sif_converter.sh
#       2. Edit:     Set DNA_SJOB_NAME and python_arguments in this script
#       3. Transfer (use your preferred method, e.g., rsync, scp, sftp):
#                    artifact/apptainer/valeria/, slurm_jobs/slurm_job.<DNA_SJOB_NAME>.apptainer.valeria.bash,
#                    .dockerized_norlab/,
#                    data/external_data/, data/repository_data/
#                    (data/shared_data/ is optional — replaced by a local data volume on the HPC server)
#     On Valeria:
#       4. Build SIF: bash artifact/apptainer/valeria/dna_tar_to_apptainer_sif_converter.sh
#       5. Submit:    from super-project root dir execute $ sbatch slurm_jobs/slurm_job.<DNA_SJOB_NAME>.apptainer.valeria.bash
#
#   Pipeline B — registry push (--push): build and push image to a Docker registry, pull on HPC via Apptainer.
#     Local (macOS):
#       1. Build:    dna build slurm --apptainer valeria --push
#                    → builds Docker image, pushes to registry, generates dna_registry_to_apptainer_sif_converter.sh
#       2. Edit:     Set DNA_SJOB_NAME and python_arguments in this script
#       3. Transfer (use your preferred method, e.g., rsync, scp, sftp):
#                    artifact/apptainer/valeria/, slurm_jobs/slurm_job.<DNA_SJOB_NAME>.apptainer.valeria.bash,
#                    .dockerized_norlab/,
#                    data/external_data/, data/repository_data/
#                    (data/shared_data/ is optional — replaced by a local data volume on the HPC server)
#     On Valeria:
#       4. Build SIF: bash artifact/apptainer/valeria/dna_registry_to_apptainer_sif_converter.sh
#                    (optionally add --docker-login to authenticate to a private registry)
#       5. Submit:    from super-project root dir execute $ sbatch slurm_jobs/slurm_job.<DNA_SJOB_NAME>.apptainer.valeria.bash
#
# Usage:
#   $ sbatch slurm_job.<DNA_SJOB_NAME>.apptainer.valeria.bash
#
# =================================================================================================
declare -x DNA_SJOB_NAME
declare -a python_arguments=()

# ====Setup========================================================================================
# ....Custom setup (optional)......................................................................
function job_setup_callback() {
  # TODO: Add any instruction that should be executed before the apptainer exec command

  # Required for wandb.ai
  module load httpproxy
}

# ....Custom teardown (optional)...................................................................
function job_teardown_callback() {
  local exit_code=$?
  # TODO: Add any instruction that should be executed after apptainer exec exits.
  exit "${exit_code:-1}"
}

# ....Python module................................................................................
# TODO: Set python module to launch
python_arguments+=("launcher/example.py")
# Note: container workdir is <DN_PROJECT_PATH>/src/ (set in .env.valeria: DN_PROJECT_PATH)

# ....Optional hydra flags.........................................................................
# --config-path,-cp : Overrides the config_path specified in hydra.main(). (absolute or relative)
# --config-name,-cn : Overrides the config_name specified in hydra.main()
# --config-dir,-cd : Adds an additional config dir to the config search path
#python_arguments+=("--config-path=")
#python_arguments+=("--config-dir=")
#python_arguments+=("--config-name=")

# ====DNA internal=================================================================================
# ....Set job name.................................................................................
# Recommend opening an issue tracker task (e.g., YouTrack, GitHub issue, Trello)
#  and use its issue ID as the DNA_SJOB_NAME.

# Auto-set DNA_SJOB_NAME from the script filename (slurm_job.<name>.apptainer.valeria.bash → <name>)
DNA_SJOB_NAME="$( basename "${BASH_SOURCE[0]}" | sed 's/^slurm_job\.//;s/\.apptainer\.valeria\.bash$//' )"
export DNA_SJOB_NAME

# ....HPC server configuration.....................................................................
SUPER_PROJECT_ROOT="${SUPER_PROJECT_ROOT:-$(pwd)}"
# DNA names each SIF with the full version: <image>-slurm-<PROJECT_TAG>-<target>.sif (e.g.
# ...-slurm-l4t-r36.4.0-valeria.sif). The exact version is only known at build time, so resolve the
# newest matching versioned SIF at runtime. Export SIF_PATH to pin a specific version instead.
SIF_PATH="${SIF_PATH:-$(ls -t ${SCRATCH}/sif/PLACEHOLDER_DN_PROJECT_IMAGE_NAME-slurm-*-valeria.sif 2>/dev/null | head -n1)}"
PROFILE_ENV_FILE="${SUPER_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.valeria"

# Source HPC-specific env (sets DN_PROJECT_PATH, DN_PROJECT_USER, etc.)
# shellcheck source=/dev/null
source "${PROFILE_ENV_FILE}" 2>/dev/null || {
  echo "[warning] Profile env file not found: ${PROFILE_ENV_FILE}" 1>&2
}

# ====Load Apptainer module (HPC module system)====================================================
# Try to load the highest available apptainer version; fallback to default.
if command -v module &>/dev/null; then
  _APPTAINER_LATEST_VERSION="$( module spider apptainer 2>&1 | grep -oE 'apptainer/[0-9]+\.[0-9]+\.[0-9]+' | sed 's|apptainer/||' | sort -V | tail -1 )"
  if [[ -n "${_APPTAINER_LATEST_VERSION}" ]]; then
    echo "[info] Loading Apptainer module version: ${_APPTAINER_LATEST_VERSION}" 1>&2
    module load "apptainer/${_APPTAINER_LATEST_VERSION}"
  else
    echo "[info] Loading default Apptainer module" 1>&2
    module load apptainer
  fi
fi

# Set APPTAINER_CACHEDIR and APPTAINER_TMPDIR to the local node scratch space.
# Using SLURM_TMPDIR (fast local SSD allocated per job) avoids writing to the Lustre
# home filesystem, which has quota limits and does not support atomic rename required
# by Apptainer's cache. Falls back to /tmp if SLURM_TMPDIR is not set.
# Ref: https://apptainer.org/docs/user/latest/build_env.html
# Ref: https://doc.s3.valeria.science/fr/calcul/apptainer.html#bonnes-pratiques
export APPTAINER_CACHEDIR="$( mktemp -d -p "${SLURM_TMPDIR}" 2>/dev/null || mktemp -d )"
export APPTAINER_TMPDIR="$( mktemp -d -p "${SLURM_TMPDIR}" 2>/dev/null || mktemp -d )"

# Sanity checks
if [[ ! -f "${SIF_PATH}" ]]; then
  echo "[error] SIF file not found: ${SIF_PATH}" 1>&2
  echo "[hint] Build it with: bash artifact/apptainer/valeria/dna_tar_to_apptainer_sif_converter.sh" 1>&2
  exit 1
fi

if [[ -z "${DN_PROJECT_PATH}" ]]; then
  echo "[error] DN_PROJECT_PATH is not set. Check ${PROFILE_ENV_FILE}" 1>&2
  exit 1
fi

# ====Content guard: verify the SIF carries a COMPLETE baked-in super-project '.git'==============
# The DN/N2ST entrypoint bootstrap resolves PROJECT_PATH/N2ST_PATH via 'git rev-parse'. A truncated
# SIF conversion can drop the (large) super-project '.git' while smaller sibling repos under
# /ros2_ws/src/ survive, so a mere '.git/HEAD' existence check is NOT enough (it can false-pass).
# Validate that ${DN_PROJECT_PATH}/.git is a COMPLETE repository (HEAD + objects + refs resolvable by
# git) and fail fast with an actionable message.
# IMPORTANT: '--no-mount cwd'. Apptainer auto-binds the current working directory into the
# container. When this job is launched from the host super-project root, that host directory
# (which does NOT carry '.git' on the HPC server — '.git' is baked into the image) gets mounted
# over ${DN_PROJECT_PATH}, MASKING the image's baked-in '.git' and breaking the DN/N2ST bootstrap.
# Disabling the cwd auto-mount keeps the baked-in '.git' visible.
if ! apptainer exec --no-mount cwd "${SIF_PATH}" /bin/sh -c '[ -d "'"${DN_PROJECT_PATH}"'/.git/objects" ] && [ -d "'"${DN_PROJECT_PATH}"'/.git/refs" ] && git -c safe.directory="*" --git-dir="'"${DN_PROJECT_PATH}"'/.git" rev-parse --verify HEAD >/dev/null 2>&1'; then
  echo "[error] The SIF has a missing/incomplete baked-in super-project '.git': ${SIF_PATH}" 1>&2
  echo "[error]   (expected a valid ${DN_PROJECT_PATH}/.git inside the container). The SIF was likely" 1>&2
  echo "[error]   produced by a truncated/OOM-killed SIF conversion. Rebuild it with the tar pipeline" 1>&2
  echo "[error]   (dna build slurm --apptainer <target> --save) or copy a known-good SIF, then re-submit." 1>&2
  exit 1
fi

job_setup_callback
trap job_teardown_callback EXIT

# ====Apptainer compatibility======================================================================
echo "[info] This script requires Apptainer >= 1.1.0 (for --no-eval, --cleanenv, --env-file comment support)." 1>&2

# ====Launch Apptainer slurm job===================================================================
echo "[info] Launching Apptainer slurm job: DNA_SJOB_NAME=${DNA_SJOB_NAME}"
echo "[info] SIF: ${SIF_PATH}"
echo "[info] Python args: ${python_arguments[*]}"

# Note: --nv enables NVIDIA GPU access inside the container (equivalent to Docker's runtime: nvidia).
#       Remove it for CPU-only jobs.
# Note: src/ and utilities/ are bind-mounted read-only to enable fast code iteration.
#       Build, push, pull, and convert to SIF once; then rsync modified code to the HPC server
#       and re-submit the job without rebuilding the Docker image or reconverting the SIF.
apptainer exec \
    --no-eval \
    --cleanenv \
    --no-home \
    --no-mount cwd \
    --nv \
    --bind /etc/localtime:/etc/localtime:ro \
    --bind "${SUPER_PROJECT_ROOT}/.dockerized_norlab/configuration/entrypoints/:/entrypoints/:ro" \
    --bind "${SUPER_PROJECT_ROOT}/.dockerized_norlab/dn_container_env_variable/:/dn_container_env_variable/:rw" \
    --bind "${SUPER_PROJECT_ROOT}/artifact/:${DN_PROJECT_PATH}/artifact/:rw" \
    --bind "${SUPER_PROJECT_ROOT}/data/external_data/:${DN_PROJECT_PATH}/data/external_data/:rw" \
    --bind "${DNA_HOST_SHARED_DATA_PATH:-${SUPER_PROJECT_ROOT}/data/shared_data/}:${DN_PROJECT_PATH}/data/shared_data/:ro" \
    --bind "${SUPER_PROJECT_ROOT}/src/:${DN_PROJECT_PATH}/src/:ro" \
    --bind "${SUPER_PROJECT_ROOT}/utilities/:${DN_PROJECT_PATH}/utilities/:ro" \
    --env GIT_DIR="${DN_PROJECT_PATH}/.git" \
    --env-file "${PROFILE_ENV_FILE}" \
    --env CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES}" \
    --env SLURM_JOB_ID="${SLURM_JOB_ID}" \
    --env SLURM_TMPDIR="${SLURM_TMPDIR}" \
    --env SLURM_JOB_NAME="${SLURM_JOB_NAME}" \
    --env SLURM_NODELIST="${SLURM_NODELIST}" \
    --env DN_CONTAINER_NAME="${DN_CONTAINER_NAME:?err}-${DNA_SJOB_NAME}" \
    --pwd "${DN_PROJECT_PATH}/src" \
    --workdir "${SLURM_TMPDIR:-/tmp}" \
    --writable-tmpfs \
    "${SIF_PATH}" \
    "/dockerized-norlab/project/project-slurm/dn_entrypoint.init.bash" \
    "${python_arguments[@]}"
