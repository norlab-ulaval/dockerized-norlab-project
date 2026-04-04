#!/bin/bash
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=12
#SBATCH --time=7-00:00
#SBATCH --output=out/%x-%j.out
#SBATCH --account=PLACEHOLDER_ACCOUNT
# Note: Flag time format --time=D-HH:MM ->  D=day, HH=hours, MM=minutes
# Note: Replace PLACEHOLDER_ACCOUNT with your Compute Canada allocation account (e.g., def-username)
# =================================================================================================
# Execute Apptainer slurm job on Compute Canada (Digital Research Alliance of Canada) HPC server.
#
# Standalone script — does NOT require DNA to be installed on Compute Canada servers.
# The apptainer exec command is executed directly using the pre-built SIF file.
#
# Workflow:
#   Local (macOS):
#     1. Build:    dna build slurm --apptainer compute_canada
#                  → builds Docker image, saves tar archive, generates dna_tar_to_apptainer_sif_converter.sh
#     2. Edit:     Set DNA_SJOB_NAME and python_arguments in this script
#     3. Transfer (use your preferred method, e.g., rsync, scp, sftp):
#                  artifact/apptainer/, slurm_jobs/slurm_job.<DNA_SJOB_NAME>.apptainer.compute_canada.bash,
#                  .dockerized_norlab/,
#                  data/external_data/, data/repository_data/
#                  (data/shared_data/ is optional — replaced by a local data volume on the HPC server)
#   On Compute Canada:
#     4. Build SIF: bash artifact/apptainer/dna_tar_to_apptainer_sif_converter.sh
#     5. Submit:    from super-project root dir execute $ sbatch slurm_jobs/slurm_job.<DNA_SJOB_NAME>.apptainer.compute_canada.bash
#
# Usage:
#   $ sbatch slurm_job.<DNA_SJOB_NAME>.apptainer.compute_canada.bash
#
# =================================================================================================
declare -x DNA_SJOB_NAME
declare -a python_arguments=()

# ====Setup========================================================================================
# ....Custom setup (optional)......................................................................
function job_setup_callback() {
  # Add any instruction that should be executed before the apptainer exec command
  :
}

# ....Custom teardown (optional)...................................................................
function job_teardown_callback() {
  local exit_code=$?
  # TODO: Add any instruction that should be executed after apptainer exec exits.
  exit "${exit_code:-1}"
}

# ....Set job name.................................................................................
# TODO: Set DNA_SJOB_NAME
DNA_SJOB_NAME="default"
# Note: Recommend opening an issue tracker task (e.g., YouTrack, GitHub issue, Trello)
#  and use its issue ID as an DNA_SJOB_NAME.

# ....Python module................................................................................
# TODO: Set python module to launch
python_arguments+=("launcher/example.py")
# Note: container workdir is <DN_PROJECT_PATH>/src/ (set in .env.compute_canada: DN_PROJECT_PATH)

# ....HPC server configuration.....................................................................
SUPER_PROJECT_ROOT="${SUPER_PROJECT_ROOT:-$(pwd)}"
SIF_PATH="${SIF_PATH:-${SUPER_PROJECT_ROOT}/artifact/apptainer/PLACEHOLDER_DN_PROJECT_IMAGE_NAME-slurm.sif}"
PROFILE_ENV_FILE="${SUPER_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.compute_canada"

# ====DNA internal=================================================================================
export DNA_SJOB_NAME

# Source HPC-specific env (sets DN_PROJECT_PATH, DN_PROJECT_USER, etc.)
# shellcheck source=/dev/null
source "${PROFILE_ENV_FILE}" 2>/dev/null || {
  echo "[warning] Profile env file not found: ${PROFILE_ENV_FILE}" 1>&2
}

# Set APPTAINER_TMPDIR to SLURM_TMPDIR for best performance on Compute Canada
# (SLURM_TMPDIR is high-speed local storage allocated per job)
APPTAINER_TMPDIR="${SLURM_TMPDIR:-/tmp}"
export APPTAINER_TMPDIR

# Sanity checks
if [[ ! -f "${SIF_PATH}" ]]; then
  echo "[error] SIF file not found: ${SIF_PATH}" 1>&2
  echo "[hint] Build it with: bash artifact/apptainer/dna_tar_to_apptainer_sif_converter.sh" 1>&2
  exit 1
fi

if [[ -z "${DN_PROJECT_PATH}" ]]; then
  echo "[error] DN_PROJECT_PATH is not set. Check ${PROFILE_ENV_FILE}" 1>&2
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
apptainer exec \
    --no-eval \
    --cleanenv \
    --no-home \
    --nv \
    --bind /etc/localtime:/etc/localtime:ro \
    --bind "${SUPER_PROJECT_ROOT}/.dockerized_norlab/configuration/entrypoints/:/entrypoints/:ro" \
    --bind "${SUPER_PROJECT_ROOT}/.dockerized_norlab/dn_container_env_variable/:/dn_container_env_variable/:rw" \
    --bind "${SUPER_PROJECT_ROOT}/artifact/:${DN_PROJECT_PATH}/artifact/:rw" \
    --bind "${SUPER_PROJECT_ROOT}/data/external_data/:${DN_PROJECT_PATH}/data/external_data/:rw" \
    --bind "${DNA_HOST_SHARED_DATA_PATH:-${SUPER_PROJECT_ROOT}/data/shared_data/}:${DN_PROJECT_PATH}/data/shared_data/:ro" \
    --env-file "${PROFILE_ENV_FILE}" \
    --env CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES}" \
    --env SLURM_JOB_ID="${SLURM_JOB_ID}" \
    --env SLURM_TMPDIR="${SLURM_TMPDIR}" \
    --env SLURM_JOB_NAME="${SLURM_JOB_NAME}" \
    --env SLURM_NODELIST="${SLURM_NODELIST}" \
    --pwd "${DN_PROJECT_PATH}/src" \
    --writable-tmpfs \
    "${SIF_PATH}" \
    /dockerized-norlab/project/project-slurm/dn_entrypoint.init.bash \
    "${python_arguments[@]}"
