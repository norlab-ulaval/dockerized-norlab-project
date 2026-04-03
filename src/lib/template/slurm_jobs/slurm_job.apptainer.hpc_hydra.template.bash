#!/bin/bash
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=12
#SBATCH --time=7-00:00
#SBATCH --output=out/%x-%j.out
# Note: Flag time format --time=D-HH:MM ->  D=day, HH=hours, MM=minutes
# =================================================================================================
# Execute Hydra-based Apptainer slurm job on an HPC server (Valeria or Compute Canada).
#
# Standalone script — does NOT require DNA to be installed on the HPC server.
# The apptainer exec command is executed directly using the pre-built SIF file.
#
# Workflow:
#   Local (macOS):
#     1. Build:    dna build slurm --apptainer <profile>
#     2. Generate: dna run slurm <sjob-id> --generate-apptainer <profile> <python-args>
#     3. Transfer: rsync -av artifact/apptainer/ user@hpc:/path/to/project/artifact/apptainer/
#   On HPC:
#     4. Build SIF: bash artifact/apptainer/build_sif.sh
#     5. Submit:    sbatch slurm_job.apptainer.hpc_hydra.template.bash
#
# Usage:
#   $ sbatch slurm_job.apptainer.hpc_hydra.template.bash
#
# =================================================================================================
declare -x SJOB_ID
declare -a hydra_flags=()

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
# TODO: Set SJOB_ID
SJOB_ID="default"
# Note: Recommend opening an issue tracker task (e.g., YouTrack, GitHub issue, Trello)
#  and use its issue ID as an SJOB_ID.

# ....Hydra app module.............................................................................
# TODO: Set python module to launch
hydra_flags+=("launcher/example_app.py")
# Note: container workdir is <DN_PROJECT_PATH>/src/ (set in profile env: DN_PROJECT_PATH)

# ....Optional hydra flags.........................................................................
# --config-path,-cp : Overrides the config_path specified in hydra.main(). (absolute or relative)
# --config-name,-cn : Overrides the config_name specified in hydra.main()
# --config-dir,-cd : Adds an additional config dir to the config search path
#hydra_flags+=("--config-path=")
#hydra_flags+=("--config-dir=")
#hydra_flags+=("--config-name=")

# ....HPC server configuration.....................................................................
# TODO: Set profile to 'valeria', 'compute_canada', or 'mamba'
HPC_PROFILE="${HPC_PROFILE:-valeria}"
SUPER_PROJECT_ROOT="${SUPER_PROJECT_ROOT:-$(pwd)}"
SIF_PATH="${SIF_PATH:-${SUPER_PROJECT_ROOT}/artifact/apptainer/PLACEHOLDER_DN_PROJECT_IMAGE_NAME-slurm.sif}"
PROFILE_ENV_FILE="${SUPER_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.${HPC_PROFILE}"

# ====DNA internal=================================================================================
export SJOB_ID

# Source HPC-specific env (sets DN_PROJECT_PATH, DN_PROJECT_USER, etc.)
# shellcheck source=/dev/null
source "${PROFILE_ENV_FILE}" 2>/dev/null || {
  echo "[warning] Profile env file not found: ${PROFILE_ENV_FILE}" 1>&2
}

# Set APPTAINER_TMPDIR to SLURM_TMPDIR for best performance on HPC
APPTAINER_TMPDIR="${SLURM_TMPDIR:-/tmp}"
export APPTAINER_TMPDIR

# Sanity checks
if [[ ! -f "${SIF_PATH}" ]]; then
  echo "[error] SIF file not found: ${SIF_PATH}" 1>&2
  echo "[hint] Build it with: bash artifact/apptainer/build_sif.sh" 1>&2
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

# ====GPU configuration============================================================================
NV_FLAG=""
if [[ "${APPTAINER_ENABLE_GPU:-true}" == "true" ]]; then
    NV_FLAG="--nv"
fi

# ====Launch Apptainer slurm job===================================================================
echo "[info] Launching Apptainer Hydra slurm job: SJOB_ID=${SJOB_ID}"
echo "[info] SIF: ${SIF_PATH}"
echo "[info] Hydra flags: ${hydra_flags[*]}"

apptainer exec \
    --no-eval \
    --cleanenv \
    --no-home \
    ${NV_FLAG} \
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
    "${hydra_flags[@]}"
