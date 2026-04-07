#!/bin/bash
# =================================================================================================
# Apptainer/HPC utilities for generating Apptainer-compatible commands and scripts.
#
# Provides functions to generate Apptainer exec commands, standalone run scripts, and
# SIF build helper scripts from DNA's Docker-based slurm pipeline.
#
# Design constraints:
#   - Apptainer is Linux-only and NOT supported on macOS.
#   - DNA's role is to BUILD and SAVE a linux/amd64 Docker tar archive locally, which is then
#     transferred to the HPC server where Apptainer converts and runs it.
#   - These functions GENERATE apptainer commands/scripts — they do NOT execute 'apptainer' locally.
#   - HPC server profiles are selected via --apptainer <profile> on existing dna commands.
#
# Usage:
#   $ source apptainer_tools.bash
#
# Globals:
#   Read SUPER_PROJECT_ROOT         - Super project root directory (host side)
#   Read DN_PROJECT_GIT_NAME        - Project git repository name
#   Read DN_PROJECT_IMAGE_NAME      - Docker image name
#   Read PROJECT_TAG                - Docker image tag
#   Read DNA_DEBUG                  - Enable debug output when set to true
#
# =================================================================================================


# =================================================================================================
# Verifies that a server profile environment file exists.
#
# Usage:
#   $ dna::check_apptainer_profile_env_file "valeria"
#
# Positional argument:
#   profile - HPC server profile name (e.g., 'valeria', 'compute_canada', 'mamba')
#
# Returns:
#   0 if profile env file exists
#   1 if profile env file is missing
# =================================================================================================
function dna::check_apptainer_profile_env_file() {
  local profile="${1:?err}"
  local profile_env_file="${SUPER_PROJECT_ROOT:?err}/.dockerized_norlab/configuration/hpc_server_profile/.env.${profile}"

  if [[ ! -f "${profile_env_file}" ]]; then
    n2st::print_msg_error "HPC server profile env file not found: ${profile_env_file}
Create it from the template: src/lib/template/.dockerized_norlab/configuration/hpc_server_profile/.env.${profile}"
    return 1
  fi
  return 0
}


# =================================================================================================
# Sources a HPC server profile env file and exports DN_PROJECT_USER for the Docker build.
#
# This function must be called AFTER load_super_project_config.bash and BEFORE any Docker
# build/save command so that DN_PROJECT_USER from the HPC profile overrides the local default.
# It validates that DN_PROJECT_USER is set to a real value (not a placeholder or empty).
#
# Usage:
#   $ dna::load_apptainer_profile_env "valeria"
#
# Positional argument:
#   profile - HPC server profile name (e.g., 'valeria', 'compute_canada', 'mamba')
#
# Globals:
#   Read  SUPER_PROJECT_ROOT
#   Write DN_PROJECT_USER (exported)
#
# Returns:
#   0 on success, 1 if profile is missing or DN_PROJECT_USER is invalid
# =================================================================================================
function dna::load_apptainer_profile_env() {
  local profile="${1:?err}"
  local profile_env_file="${SUPER_PROJECT_ROOT:?err}/.dockerized_norlab/configuration/hpc_server_profile/.env.${profile}"

  dna::check_apptainer_profile_env_file "${profile}" || return 1

  # Source profile env file to pick up DN_PROJECT_USER and other HPC-specific overrides
  n2st::print_msg "Loading HPC server profile: ${profile_env_file}"
  set -o allexport
  # shellcheck disable=SC1090
  source "${profile_env_file}" || {
    n2st::print_msg_error "Failed to source HPC profile env file: ${profile_env_file}"
    set +o allexport
    return 1
  }
  set +o allexport

  # Validate DN_PROJECT_USER is set to a real value
  if [[ -z "${DN_PROJECT_USER:-}" ]] || [[ "${DN_PROJECT_USER}" == "PLACEHOLDER_HPC_USERNAME" ]]; then
    n2st::print_msg_error "DN_PROJECT_USER is not configured in ${profile_env_file}
Set it to your HPC server username so the container user matches the Apptainer host user.
Example: DN_PROJECT_USER=jdoe"
    return 1
  fi

  n2st::print_msg "Using HPC server DN_PROJECT_USER=${DN_PROJECT_USER} (from profile: ${profile})"
  export DN_PROJECT_USER
  return 0
}


# =================================================================================================
# Generates a dna_tar_to_apptainer_sif_converter.sh helper script to be run on the HPC server.
#
# This script:
#   1. Sets up and validates the expected super-project directory structure on the HPC server.
#   2. Converts the Docker tar archive to an Apptainer SIF file.
#
# It is NOT executed locally — copy it to the HPC server alongside the tar archive.
#
# Expected HPC super-project directory structure (validated by the generated script):
#   super-project/
#     ├── .dockerized_norlab/          ← DNA configuration (transferred from local)
#     ├── artifact/apptainer/          ← tar archive + this script + built SIF
#     ├── artifact/optuna_storage/
#     ├── artifact/slurm_jobs_logs/
#     ├── artifact/tensorboard_tmp/
#     ├── data/external_data/
#     ├── data/repository_data/
#     ├── data/shared_data/
#     └── slurm_jobs/                  ← sbatch scripts
#
# Usage:
#   $ dna::generate_apptainer_build_sif_script "myproject-slurm.latest.tar" "myproject-slurm.sif" "/output/dir"
#
# Positional arguments:
#   tar_filename  - Docker tar archive filename (e.g., 'myproject-slurm.latest.tar')
#   sif_name      - Output SIF filename (e.g., 'myproject-slurm.sif')
#   output_dir    - Directory where the dna_tar_to_apptainer_sif_converter.sh script will be written
#
# Outputs:
#   Writes dna_tar_to_apptainer_sif_converter.sh to output_dir
#
# Returns:
#   0 on success, 1 on failure
# =================================================================================================
function dna::generate_apptainer_build_sif_script() {
  local tar_filename="${1:?err}"
  local sif_name="${2:?err}"
  local output_dir="${3:?err}"
  local profile="${4:-}"

  local script_path="${output_dir}/dna_tar_to_apptainer_sif_converter.sh"

  n2st::print_msg "Generating Apptainer SIF build helper script: ${script_path}"

  cat > "${script_path}" << 'SCRIPT_EOF'
#!/bin/bash
# =================================================================================================
# Auto-generated by DNA: dna build slurm --apptainer / dna save --apptainer
# Run this script on the HPC server AFTER transferring the artifact/apptainer/ directory.
#
# This script:
#   1. Sets up and validates the expected super-project directory structure on the HPC server.
#   2. Converts the Docker tar archive to an Apptainer SIF file.
#
# Usage (from the super-project root directory on the HPC server):
#   $ bash artifact/apptainer/dna_tar_to_apptainer_sif_converter.sh [--target-dir <TARGET-DIRECTORY-PATH>]
#
# Options:
#   --target-dir <TARGET-DIRECTORY-PATH>
#       Optional. Path to the HPC super-project root directory.
#       When specified, the HPC super-project directory structure will be created under this path,
#       and the Apptainer SIF file will be output to <TARGET-DIRECTORY-PATH>/artifact/apptainer/.
#       Defaults to two levels above this script's directory (i.e., the super-project root inferred
#       from the standard artifact/apptainer/ placement).
#
# Requires:
#   - apptainer installed on the HPC server
#   - The Docker tar archive (.tar) in the same directory as this script (artifact/apptainer/)
#
# =================================================================================================
set -e

# ====Argument parsing=============================================================================
TARGET_DIR=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --target-dir)
      if [[ -z "${2:-}" ]]; then
        echo "[error] --target-dir requires a path argument." 1>&2
        exit 1
      fi
      TARGET_DIR="${2}"
      shift 2
      ;;
    *)
      echo "[error] Unknown argument: $1" 1>&2
      echo "Usage: $0 [--target-dir <TARGET-DIRECTORY-PATH>]" 1>&2
      exit 1
      ;;
  esac
done

# ====Apptainer compatibility======================================================================
echo "[info] This script requires Apptainer >= 1.1.0 (for --no-eval, --cleanenv, --env-file comment support)." 1>&2

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SUPER_PROJECT_ROOT="${TARGET_DIR:-${SUPER_PROJECT_ROOT:-$(cd "${SCRIPT_DIR}/../.." && pwd)}}"
SCRIPT_EOF

  # Inject the tar_filename and sif_name variables (expand at generation time)
  cat >> "${script_path}" << EOF
TAR_FILENAME="${tar_filename}"
SIF_FILENAME="${sif_name}"
EOF

  cat >> "${script_path}" << 'SCRIPT_EOF'

TAR_FILE="${SCRIPT_DIR}/${TAR_FILENAME}"
# When --target-dir is provided, output the SIF into <target-dir>/artifact/apptainer/
# otherwise output it alongside this script (default: artifact/apptainer/)
if [[ -n "${TARGET_DIR}" ]]; then
  SIF_FILE="${SUPER_PROJECT_ROOT}/artifact/apptainer/${SIF_FILENAME}"
else
  SIF_FILE="${SCRIPT_DIR}/${SIF_FILENAME}"
fi

# ====Setup HPC super-project directory structure=================================================
echo "[info] Setting up HPC super-project directory structure under: ${SUPER_PROJECT_ROOT}" 1>&2

declare -a REQUIRED_DIRS=(
  "artifact/apptainer"
  "artifact/optuna_storage"
  "artifact/slurm_jobs_logs"
  "artifact/tensorboard_tmp"
  "data/external_data"
  "data/repository_data"
  "data/shared_data"
  "slurm_jobs"
)

for dir in "${REQUIRED_DIRS[@]}"; do
  target="${SUPER_PROJECT_ROOT}/${dir}"
  if [[ ! -d "${target}" ]]; then
    echo "[info]   Creating missing directory: ${dir}" 1>&2
    mkdir -p "${target}"
  else
    echo "[info]   OK: ${dir}" 1>&2
  fi
done

echo "[info] Directory structure validated." 1>&2

# ====Convert tar archive to Apptainer SIF========================================================
if [[ ! -f "${TAR_FILE}" ]]; then
  echo "[error] Docker tar archive not found: ${TAR_FILE}" 1>&2
  exit 1
fi

echo "[info] Building Apptainer SIF from Docker tar archive..." 1>&2
echo "[info]   Input:  ${TAR_FILE}" 1>&2
echo "[info]   Output: ${SIF_FILE}" 1>&2

SCRIPT_EOF

  # Inject profile-specific Apptainer cache configuration (before module load)
  if [[ "${profile}" == "valeria" ]]; then
    cat >> "${script_path}" << 'SCRIPT_EOF'
# ====Valeria HPC Apptainer cache configuration====================================================
# Must run before module load apptainer so APPTAINER_TMPDIR/APPTAINER_CACHEDIR are set first.
# val-mktemp-dir allocates space on the HPC scratch filesystem (not subject to home quota).
# La ligne source /etc/profile.d/val-utils.sh est requise car les configurations sous profile.d
# ne sont pas disponibles par défaut pour les travaux en lots.
# Ref https://doc.s3.valeria.science/fr/calcul/apptainer.html
# shellcheck source=/dev/null
[[ -f /etc/profile.d/val-utils.sh ]] && source /etc/profile.d/val-utils.sh
if command -v val-mktemp-dir &>/dev/null; then
  export APPTAINER_CACHEDIR="$( val-mktemp-dir )"
  export APPTAINER_TMPDIR="$( val-mktemp-dir )"
else
  export APPTAINER_CACHEDIR="$( mktemp -d )"
  export APPTAINER_TMPDIR="$( mktemp -d )"
fi

SCRIPT_EOF
  else
    cat >> "${script_path}" << 'SCRIPT_EOF'
# ====Apptainer cache configuration================================================================
# Set APPTAINER_CACHEDIR and APPTAINER_TMPDIR to scratch space before loading the apptainer
# module to avoid Lustre home quota issues during SIF build.
# Ref: https://docs.alliancecan.ca/wiki/Apptainer
export APPTAINER_CACHEDIR="$( mktemp -d )"
export APPTAINER_TMPDIR="$( mktemp -d )"

SCRIPT_EOF
  fi

  cat >> "${script_path}" << 'SCRIPT_EOF'
if command -v module &>/dev/null; then
  module load apptainer
fi

# Build the SIF to APPTAINER_TMPDIR (scratch space, set above) to avoid Lustre home quota
# issues during the build process, then move the completed SIF to the final destination.
# APPTAINER_TMPDIR is set above via val-mktemp-dir (Valeria) or mktemp -d (other HPC).
# Ref: https://doc.s3.valeria.science/fr/calcul/apptainer.html
SIF_STAGING_DIR="${APPTAINER_TMPDIR:-/tmp}"
SIF_TMP="${SIF_STAGING_DIR}/${SIF_FILENAME}"

echo "[info]   Staging: ${SIF_TMP}" 1>&2

if ! apptainer build \
    --mksquashfs-args="-comp zstd -Xcompression-level 19" \
    "${SIF_TMP}" \
    "docker-archive:${TAR_FILE}"; then
  echo "[error] Apptainer build failed. The tar archive has been preserved: ${TAR_FILE}" 1>&2
  exit 1
fi

echo "[info] Moving SIF from staging to final destination..." 1>&2
mv "${SIF_TMP}" "${SIF_FILE}"
echo "[done] SIF file created: ${SIF_FILE}" 1>&2

# ====Cleanup: delete the tar archive after successful SIF conversion==============================
echo "[info] Deleting tar archive to free disk space: ${TAR_FILE}" 1>&2
rm -f "${TAR_FILE}"
echo "[done] Tar archive deleted: ${TAR_FILE}" 1>&2
SCRIPT_EOF

  chmod +x "${script_path}"
  return 0
}


# =================================================================================================
# Generates a dna_registry_to_apptainer_sif_converter.sh helper script to be run on the HPC server.
#
# This script is generated by the '--push' pipeline (dna build slurm --apptainer <profile> --push).
# It builds an Apptainer SIF from a Docker registry image using 'apptainer build docker://'.
#
# Unlike dna_tar_to_apptainer_sif_converter.sh (which works with a local .tar archive), this
# script uses 'apptainer build docker://' to fetch the image directly from a Docker registry.
#
# It is NOT executed locally — transfer it to the HPC server and run it there.
#
# Expected HPC super-project directory structure (created by the generated script):
#   super-project/
#     ├── .dockerized_norlab/          ← DNA configuration
#     ├── artifact/apptainer/          ← this script + built SIF
#     ├── artifact/optuna_storage/
#     ├── artifact/slurm_jobs_logs/
#     ├── artifact/tensorboard_tmp/
#     ├── data/external_data/
#     ├── data/repository_data/
#     ├── data/shared_data/
#     └── slurm_jobs/                  ← sbatch scripts
#
# Usage:
#   $ dna::generate_registry_to_apptainer_sif_script "hub/image:tag" "myproject-slurm.sif" "/output/dir" ["valeria"]
#
# Positional arguments:
#   image_ref   - Full Docker image reference (e.g., 'norlabulaval/myproject-slurm:tag')
#   sif_name    - Output SIF filename (e.g., 'myproject-slurm.sif')
#   output_dir  - Directory where the dna_registry_to_apptainer_sif_converter.sh script will be written
#   profile     - Optional HPC server profile name (e.g., 'valeria'). When 'valeria', injects
#                 val-mktemp-dir Apptainer cache configuration.
#
# Outputs:
#   Writes dna_registry_to_apptainer_sif_converter.sh to output_dir
#
# Returns:
#   0 on success, 1 on failure
# =================================================================================================
function dna::generate_registry_to_apptainer_sif_script() {
  local image_ref="${1:?err}"
  local sif_name="${2:?err}"
  local output_dir="${3:?err}"
  local profile="${4:-}"

  local script_path="${output_dir}/dna_registry_to_apptainer_sif_converter.sh"

  n2st::print_msg "Generating Apptainer SIF registry-pull helper script: ${script_path}"

  cat > "${script_path}" << 'SCRIPT_EOF'
#!/bin/bash
# =================================================================================================
# Auto-generated by DNA: dna build slurm --apptainer <profile> --push
# Run this script on the HPC server to build an Apptainer SIF from a Docker registry image.
#
# This script:
#   1. Sets up and validates the expected super-project directory structure on the HPC server.
#   2. Optionally authenticates to the Docker registry interactively via --docker-login.
#   3. Builds the Apptainer SIF to APPTAINER_TMPDIR (val-mktemp-dir scratch on Valeria,
#      mktemp -d on other HPC) then moves it to the final destination to avoid Lustre quota.
#
# Usage (from the super-project root directory on the HPC server):
#   $ bash artifact/apptainer/dna_registry_to_apptainer_sif_converter.sh [OPTIONS]
#
# Options:
#   --target-dir <TARGET-DIRECTORY-PATH>
#       Optional. Path to the HPC super-project root directory.
#       When specified, the Apptainer SIF file will be output to
#       <TARGET-DIRECTORY-PATH>/artifact/apptainer/.
#       Defaults to two levels above this script's directory (the super-project root inferred
#       from the standard artifact/apptainer/ placement).
#   --docker-login
#       Optional. Pass this flag to authenticate interactively with the Docker registry
#       (docker.io) before building the SIF. Apptainer will prompt for credentials.
#
# Requires:
#   - apptainer installed on the HPC server
#   - Network access to docker.io from the HPC server
#
# =================================================================================================
set -e

# ====Argument parsing=============================================================================
TARGET_DIR=""
USE_DOCKER_LOGIN=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --target-dir)
      if [[ -z "${2:-}" ]]; then
        echo "[error] --target-dir requires a path argument." 1>&2
        exit 1
      fi
      TARGET_DIR="${2}"
      shift 2
      ;;
    --docker-login)
      USE_DOCKER_LOGIN=true
      shift
      ;;
    *)
      echo "[error] Unknown argument: $1" 1>&2
      echo "Usage: $0 [--target-dir <PATH>] [--docker-login]" 1>&2
      exit 1
      ;;
  esac
done

# ====Apptainer compatibility======================================================================
echo "[info] This script requires Apptainer >= 1.1.0." 1>&2

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SUPER_PROJECT_ROOT="${TARGET_DIR:-${SUPER_PROJECT_ROOT:-$(cd "${SCRIPT_DIR}/../.." && pwd)}}"
SCRIPT_EOF

  # Inject the image_ref and sif_name variables (expand at generation time)
  cat >> "${script_path}" << EOF
IMAGE_REF="${image_ref}"
SIF_FILENAME="${sif_name}"
EOF

  cat >> "${script_path}" << 'SCRIPT_EOF'

# When --target-dir is provided, output the SIF into <target-dir>/artifact/apptainer/
# otherwise output it alongside this script (default: artifact/apptainer/)
if [[ -n "${TARGET_DIR}" ]]; then
  SIF_DIR="${SUPER_PROJECT_ROOT}/artifact/apptainer"
  mkdir -p "${SIF_DIR}"
else
  SIF_DIR="${SCRIPT_DIR}"
fi
SIF_FILE="${SIF_DIR}/${SIF_FILENAME}"

# ====Validate environment=========================================================================
echo "[info] Image reference: ${IMAGE_REF}" 1>&2
echo "[info] SIF output:      ${SIF_FILE}" 1>&2

# ====Validate super-project directories===========================================================
echo "[info] Validating super-project directory structure under: ${SUPER_PROJECT_ROOT}" 1>&2
declare -a required_dirs=(
  "artifact/apptainer"
  "artifact/optuna_storage"
  "artifact/slurm_jobs_logs"
  "artifact/tensorboard_tmp"
  "data/external_data"
  "data/repository_data"
  "data/shared_data"
  "slurm_jobs"
)
for dir in "${required_dirs[@]}"; do
  mkdir -p "${SUPER_PROJECT_ROOT}/${dir}"
  echo "[info]   ✓ ${dir}" 1>&2
done

SCRIPT_EOF

  # Inject profile-specific Apptainer cache configuration (must come BEFORE module load)
  if [[ "${profile}" == "valeria" ]]; then
    cat >> "${script_path}" << 'SCRIPT_EOF'
# ====Valeria HPC Apptainer cache configuration====================================================
# Must run before module load apptainer so APPTAINER_TMPDIR/APPTAINER_CACHEDIR are set first.
# val-mktemp-dir allocates space on the HPC scratch filesystem (not subject to home quota).
# La ligne source /etc/profile.d/val-utils.sh est requise car les configurations sous profile.d
# ne sont pas disponibles par défaut pour les travaux en lots.
# Ref https://doc.s3.valeria.science/fr/calcul/apptainer.html
# shellcheck source=/dev/null
[[ -f /etc/profile.d/val-utils.sh ]] && source /etc/profile.d/val-utils.sh
if command -v val-mktemp-dir &>/dev/null; then
  export APPTAINER_CACHEDIR="$( val-mktemp-dir )"
  export APPTAINER_TMPDIR="$( val-mktemp-dir )"
else
  export APPTAINER_CACHEDIR="$( mktemp -d )"
  export APPTAINER_TMPDIR="$( mktemp -d )"
fi

SCRIPT_EOF
  else
    cat >> "${script_path}" << 'SCRIPT_EOF'
# ====Apptainer cache configuration================================================================
# Set APPTAINER_CACHEDIR and APPTAINER_TMPDIR to scratch space before loading the apptainer
# module to avoid Lustre home quota issues during SIF build.
# Ref: https://docs.alliancecan.ca/wiki/Apptainer
export APPTAINER_CACHEDIR="$( mktemp -d )"
export APPTAINER_TMPDIR="$( mktemp -d )"

SCRIPT_EOF
  fi

  cat >> "${script_path}" << 'SCRIPT_EOF'
# ====Load Apptainer module (HPC module system)====================================================
if command -v module &>/dev/null; then
  module load apptainer
fi

# ====Build SIF from registry image===============================================================
# Note: --disable-cache is NOT used here because APPTAINER_CACHEDIR is already redirected to
# scratch space above (val-mktemp-dir or mktemp -d), so the cache never lands in the home
# directory. Keeping the cache also enables faster retries if the build fails mid-way.
APPTAINER_BUILD_CMD=(apptainer build --mksquashfs-args="-comp zstd -Xcompression-level 19")
if [[ "${USE_DOCKER_LOGIN}" == true ]]; then
  echo "[info] Authenticating with docker.io interactively (--docker-login)..." 1>&2
  APPTAINER_BUILD_CMD+=(--docker-login)
else
  echo "[info] No --docker-login flag provided — assuming public registry access." 1>&2
fi

# Build the SIF to APPTAINER_TMPDIR (scratch space, set above) to avoid Lustre home quota
# issues during the build process, then move the completed SIF to the final destination.
# APPTAINER_TMPDIR is set above via val-mktemp-dir (Valeria) or mktemp -d (other HPC).
# Ref: https://doc.s3.valeria.science/fr/calcul/apptainer.html
SIF_STAGING_DIR="${APPTAINER_TMPDIR:-/tmp}"
SIF_TMP="${SIF_STAGING_DIR}/${SIF_FILENAME}"

echo "[info] Building Apptainer SIF from Docker registry..." 1>&2
echo "[info]   Source: docker://${IMAGE_REF}" 1>&2
echo "[info]   Staging: ${SIF_TMP}" 1>&2
echo "[info]   Output:  ${SIF_FILE}" 1>&2

if ! "${APPTAINER_BUILD_CMD[@]}" "${SIF_TMP}" "docker://${IMAGE_REF}"; then
  echo "[error] Apptainer build failed for image: ${IMAGE_REF}" 1>&2
  echo "[hint]  Check that the image is accessible from the HPC server." 1>&2
  echo "[hint]  If the registry requires authentication, re-run with: --docker-login" 1>&2
  exit 1
fi

echo "[info] Moving SIF from staging to final destination..." 1>&2
mv "${SIF_TMP}" "${SIF_FILE}"
echo "[done] SIF file created: ${SIF_FILE}" 1>&2
SCRIPT_EOF

  chmod +x "${script_path}"
  return 0
}


# =================================================================================================
# Generates the apptainer exec flags for the slurm service based on the DNA compose structure.
#
# Maps docker-compose.run.slurm.yaml volumes and environment to equivalent apptainer flags.
# Skips X11/display volumes (not applicable on HPC).
#
# Usage:
#   $ flags=$(dna::get_apptainer_slurm_exec_flags "valeria" "/path/to/sif" "sjob001")
#
# Positional arguments:
#   profile         - HPC server profile name (e.g., 'valeria', 'compute_canada', 'mamba')
#   sif_path        - Path to the SIF file on the HPC server (used in script template)
#   dna_sjob_name   - Slurm job name (appended to DN_CONTAINER_NAME at runtime)
#
# Outputs:
#   Writes apptainer exec flags to stdout (one flag per line)
#
# Returns:
#   0 on success, 1 on failure
# =================================================================================================
function dna::get_apptainer_slurm_exec_flags() {
  local profile="${1:?err}"
  local sif_path="${2:?err}"
  local dna_sjob_name="${3:?err}"

  local profile_env_file=".dockerized_norlab/configuration/hpc_server_profile/.env.${profile}"
  local dn_project_path="\${DN_PROJECT_PATH}"
  local super_project_root="\${SUPER_PROJECT_ROOT}"
  local shared_data_path="\${DNA_HOST_SHARED_DATA_PATH:-\${SUPER_PROJECT_ROOT}/data/shared_data/}"

  declare -a flags=()

  # Disable shell evaluation of environment variables (match Docker/OCI behavior)
  flags+=("    --no-eval \\")
  # Clean environment to prevent host env leakage (match Docker isolation behavior)
  flags+=("    --cleanenv \\")
  # Prevent $HOME auto-mount (avoids pip --user package conflicts from host)
  flags+=("    --no-home \\")

  # GPU support — enables NVIDIA GPU access inside the container
  # Equivalent to runtime: nvidia in docker-compose (Docker path)
  # Remove this flag for CPU-only jobs
  flags+=("    --nv \\")

  # Bind mounts (from docker-compose.run.slurm.yaml volumes, minus X11/display)
  flags+=("    --bind /etc/localtime:/etc/localtime:ro \\")
  flags+=("    --bind ${super_project_root}/.dockerized_norlab/configuration/entrypoints/:/entrypoints/:ro \\")
  flags+=("    --bind ${super_project_root}/.dockerized_norlab/dn_container_env_variable/:/dn_container_env_variable/:rw \\")
  flags+=("    --bind ${super_project_root}/artifact/:${dn_project_path}/artifact/:rw \\")
  flags+=("    --bind ${super_project_root}/data/external_data/:${dn_project_path}/data/external_data/:rw \\")
  flags+=("    --bind ${shared_data_path}:${dn_project_path}/data/shared_data/:ro \\")

  # Environment file — passes all static config vars from HPC profile
  # (DN_PROJECT_USER, DN_HOST, IS_SLURM_RUN, DN_ENTRYPOINT_TRACE_EXECUTION, etc.)
  flags+=("    --env-file ${profile_env_file} \\")

  # Dynamic runtime env vars — Slurm-assigned, not known at config time
  # These MUST be passed via --env because they are set by the SLURM scheduler at job runtime
  flags+=("    --env CUDA_VISIBLE_DEVICES=\${CUDA_VISIBLE_DEVICES} \\")
  flags+=("    --env SLURM_JOB_ID=\${SLURM_JOB_ID} \\")
  flags+=("    --env SLURM_TMPDIR=\${SLURM_TMPDIR} \\")
  flags+=("    --env SLURM_JOB_NAME=\${SLURM_JOB_NAME} \\")
  flags+=("    --env SLURM_NODELIST=\${SLURM_NODELIST} \\")
  # Container name — set at runtime by combining base name from HPC profile and slurm job name
  flags+=("    --env DN_CONTAINER_NAME=\${DN_CONTAINER_NAME:?err}-${dna_sjob_name} \\")

  # Working directory (matches Dockerfile WORKDIR / entrypoint cd)
  flags+=("    --pwd ${dn_project_path}/src \\")

  # Writable tmpfs for numba cache and other temp writes
  flags+=("    --writable-tmpfs \\")

  for flag in "${flags[@]}"; do
    echo "${flag}"
  done
  return 0
}


# =================================================================================================
# Generates a standalone Apptainer run script for a slurm job on the HPC server.
#
# The generated script does NOT require DNA to be installed on the HPC server.
# It embeds all necessary apptainer exec flags and entrypoint resolved from the DNA compose configuration.
#
# Usage:
#   $ dna::generate_apptainer_run_script "sjob001" "valeria" \
#       "./artifact/sif/myproject-slurm.sif" "/output/dir" \
#       "launcher/train.py" "--epochs=10"
#
# Positional arguments:
#   dna_sjob_name     - Slurm job ID (used for naming the script and container)
#   profile     - HPC server profile name (e.g., 'valeria', 'compute_canada', 'mamba')
#   sif_path    - Path to the SIF file on the HPC server
#   output_dir  - Directory where the run script will be written
#   python_args - Python command and arguments (all remaining args)
#
# Outputs:
#   Writes run_apptainer_<dna_sjob_name>.sh to output_dir
#   Prints path of generated script to stdout
#
# Returns:
#   0 on success, 1 on failure
# =================================================================================================
function dna::generate_apptainer_run_script() {
  local dna_sjob_name="${1:?err}"
  local profile="${2:?err}"
  local sif_path="${3:?err}"
  local output_dir="${4:?err}"
  shift 4
  local python_args=("$@")

  local script_name="run_apptainer_${dna_sjob_name}.sh"
  local script_path="${output_dir}/${script_name}"
  local profile_env_file=".dockerized_norlab/configuration/hpc_server_profile/.env.${profile}"
  local entrypoint="/dockerized-norlab/project/project-slurm/dn_entrypoint.init.bash"

  n2st::print_msg "Generating Apptainer run script: ${script_path}"

  # Build python args string for embedding in script
  local python_args_str=""
  for arg in "${python_args[@]}"; do
    python_args_str="${python_args_str} \"${arg}\""
  done

  # Get exec flags
  local exec_flags
  exec_flags=$(dna::get_apptainer_slurm_exec_flags "${profile}" "${sif_path}" "${dna_sjob_name}")

  cat > "${script_path}" << EOF
#!/bin/bash
# =================================================================================================
# Auto-generated by DNA: dna run slurm ${dna_sjob_name} --generate-apptainer ${profile}
# Run this script on the HPC server (does NOT require DNA to be installed).
#
# Usage:
#   \$ bash ${script_name} [additional-python-args]
#
# Prerequisites on HPC server:
#   - apptainer installed
#   - SIF file available at: ${sif_path}
#   - Profile env file at: ${profile_env_file}
#
# =================================================================================================
set -e

# ====Apptainer compatibility======================================================================
echo "[info] This script requires Apptainer >= 1.1.0 (for --no-eval, --cleanenv, --env-file comment support)." 1>&2

# ====Configuration================================================================================
DNA_SJOB_NAME="${dna_sjob_name}"
SIF_PATH="\${SIF_PATH:-${sif_path}}"
SUPER_PROJECT_ROOT="\${SUPER_PROJECT_ROOT:-\$(pwd)}"

# Source HPC-specific env (sets DN_PROJECT_PATH, DN_PROJECT_USER, etc.)
# shellcheck source=/dev/null
source "\${SUPER_PROJECT_ROOT}/${profile_env_file}" 2>/dev/null || {
  echo "[warning] Profile env file not found: \${SUPER_PROJECT_ROOT}/${profile_env_file}" 1>&2
}

# ====Sanity checks================================================================================
if [[ ! -f "\${SIF_PATH}" ]]; then
  echo "[error] SIF file not found: \${SIF_PATH}" 1>&2
  echo "[hint] Build it with: bash artifact/apptainer/dna_tar_to_apptainer_sif_converter.sh" 1>&2
  exit 1
fi

if [[ -z "\${DN_PROJECT_PATH}" ]]; then
  echo "[error] DN_PROJECT_PATH is not set. Check \${SUPER_PROJECT_ROOT}/${profile_env_file}" 1>&2
  exit 1
fi

# ====Launch Apptainer=============================================================================
echo "[info] Launching Apptainer slurm job: DNA_SJOB_NAME=\${DNA_SJOB_NAME}"
echo "[info] SIF: \${SIF_PATH}"
echo "[info] Python args:${python_args_str} \$@"

apptainer exec \\
${exec_flags}
    "\${SIF_PATH}" \\
    "${entrypoint}" \\
    ${python_args_str} "\$@"

exit_code=\$?
echo "[info] Apptainer exec (entrypoint: dn_entrypoint.init.bash) exited with code: \${exit_code}"
exit \${exit_code}
EOF

  chmod +x "${script_path}"
  n2st::print_msg_done "Generated: ${script_path}"
  echo "${script_path}"
  return 0
}


# =================================================================================================
# Squashes a Docker image to a single layer while preserving all metadata.
#
# This reduces the tar archive size by collapsing all image layers into a single layer,
# which is especially useful when transferring the archive to an HPC server for Apptainer.
#
# Unlike the naive 'docker export | docker import' approach, this implementation preserves:
#   - ENV variables
#   - ENTRYPOINT / CMD
#   - WORKDIR
#   - LABEL
#   - USER
#
# Algorithm:
#   1. docker inspect → extract ENV, ENTRYPOINT, CMD, WORKDIR, LABEL, USER
#   2. docker create + docker export → export flat filesystem tar
#   3. docker import --change → import tar as a true single-layer image with all metadata
#      preserved via --change flags (no Dockerfile build step, no extra WORKDIR layer)
#   4. docker tag imported image → replace original tag; remove intermediate tag
#
# The squashed image replaces the original tag so downstream commands remain transparent.
# Using 'docker import --change' guarantees exactly 1 RootFS layer regardless of WORKDIR.
#
# Requires: python3 on host (used for JSON parsing of docker inspect output).
#           Falls back to a sed-based approach for simple ENV values if python3 is absent.
#
# Usage:
#   $ dna::squash_docker_image "norlabulaval/myproject-slurm:latest"
#
# Positional argument:
#   image_name  - Full Docker image name including tag (e.g., 'hub/name:tag')
#
# Returns:
#   0 on success, 1 on failure
# =================================================================================================
function dna::squash_docker_image() {
  local image_name="${1:?err}"
  local squashed_tag="${image_name}-squashed"
  local tmp_dir
  tmp_dir=$(mktemp -d)

  n2st::print_msg "Squashing Docker image: ${image_name}"
  n2st::print_msg "  Collapsing all layers into one while preserving metadata (ENV, ENTRYPOINT, CMD, WORKDIR, LABEL, USER)."

  # ....Step 1: Extract metadata from source image...........................................
  local env_vars entrypoint cmd workdir user labels_json
  env_vars=$(docker inspect --format='{{json .Config.Env}}' "${image_name}" 2>/dev/null) || {
    n2st::print_msg_error "Failed to inspect image: ${image_name}"
    rm -rf "${tmp_dir}"
    return 1
  }
  entrypoint=$(docker inspect --format='{{json .Config.Entrypoint}}' "${image_name}" 2>/dev/null)
  cmd=$(docker inspect --format='{{json .Config.Cmd}}' "${image_name}" 2>/dev/null)
  workdir=$(docker inspect --format='{{.Config.WorkingDir}}' "${image_name}" 2>/dev/null)
  user=$(docker inspect --format='{{.Config.User}}' "${image_name}" 2>/dev/null)
  labels_json=$(docker inspect --format='{{json .Config.Labels}}' "${image_name}" 2>/dev/null)

  # ....Step 2: Create temporary container and export filesystem.............................
  local container_id
  container_id=$(docker create "${image_name}") || {
    n2st::print_msg_error "Failed to create temporary container from: ${image_name}"
    rm -rf "${tmp_dir}"
    return 1
  }

  n2st::print_msg "  Exporting filesystem from container ${container_id}..."
  docker export "${container_id}" > "${tmp_dir}/filesystem.tar" || {
    docker rm "${container_id}" 2>/dev/null || true
    rm -rf "${tmp_dir}"
    n2st::print_msg_error "Failed to export container filesystem"
    return 1
  }

  docker rm "${container_id}" || {
    n2st::print_msg_warning "Failed to remove temporary container: ${container_id}"
  }

  # ....Step 3: Build --change flags for docker import to preserve all metadata.............
  #
  # 'docker import --change' sets image config directives without adding filesystem layers,
  # guaranteeing exactly 1 RootFS layer regardless of WORKDIR or other metadata.
  local change_flags=()

  # ENV variables (JSON array → individual --change ENV directives)
  if [[ "${env_vars}" != "null" && -n "${env_vars}" ]]; then
    if command -v python3 &>/dev/null; then
      while IFS= read -r env_line; do
        change_flags+=("--change" "ENV ${env_line}")
      done < <(python3 -c "
import json, sys
envs = json.loads(sys.argv[1])
for e in envs:
    k, _, v = e.partition('=')
    v_escaped = v.replace('\"', '\\\\\"')
    print(f'{k}=\"{v_escaped}\"')
" "${env_vars}")
    else
      # Fallback: single ENV directive with space-separated key=value pairs
      local env_line
      env_line=$(echo "${env_vars}" | tr -d '[]"' | tr ',' ' ')
      change_flags+=("--change" "ENV ${env_line}")
    fi
  fi

  # LABEL (JSON object → --change LABEL directive)
  if [[ "${labels_json}" != "null" && -n "${labels_json}" && "${labels_json}" != "{}" ]]; then
    if command -v python3 &>/dev/null; then
      local label_line
      label_line=$(python3 -c "
import json, sys
labels = json.loads(sys.argv[1])
if labels:
    pairs = ' '.join(f'{k}=\"{v}\"' for k, v in labels.items())
    print(pairs)
" "${labels_json}")
      if [[ -n "${label_line}" ]]; then
        change_flags+=("--change" "LABEL ${label_line}")
      fi
    fi
  fi

  # WORKDIR
  if [[ -n "${workdir}" ]]; then
    change_flags+=("--change" "WORKDIR ${workdir}")
  fi

  # USER
  if [[ -n "${user}" ]]; then
    change_flags+=("--change" "USER ${user}")
  fi

  # ENTRYPOINT (JSON array format)
  if [[ "${entrypoint}" != "null" && -n "${entrypoint}" ]]; then
    change_flags+=("--change" "ENTRYPOINT ${entrypoint}")
  fi

  # CMD (JSON array format)
  if [[ "${cmd}" != "null" && -n "${cmd}" ]]; then
    change_flags+=("--change" "CMD ${cmd}")
  fi

  if [[ "${DNA_DEBUG:-false}" == true ]]; then
    n2st::print_msg "  docker import --change flags:"
    for flag in "${change_flags[@]}"; do
      echo "    ${flag}"
    done
  fi

  # ....Step 4: Import tar as a true single-layer image with all metadata preserved.........
  n2st::print_msg "  Importing squashed filesystem as single-layer image with preserved metadata..."
  docker import "${change_flags[@]}" "${tmp_dir}/filesystem.tar" "${squashed_tag}" || {
    rm -rf "${tmp_dir}"
    n2st::print_msg_error "Failed to import squashed image: ${squashed_tag}"
    return 1
  }

  # ....Step 5: Re-tag squashed image to replace the original................................
  docker tag "${squashed_tag}" "${image_name}" || {
    rm -rf "${tmp_dir}"
    n2st::print_msg_error "Failed to re-tag squashed image to: ${image_name}"
    return 1
  }

  docker rmi "${squashed_tag}" 2>/dev/null || true
  rm -rf "${tmp_dir}"

  n2st::print_msg_done "Image squashed successfully: ${image_name}"
  return 0
}


# =================================================================================================
# Prints the apptainer exec command for the slurm service.
#
# Used by 'dna run slurm --generate-apptainer <profile>' to display the command that will run on the HPC.
# Does NOT execute apptainer locally (macOS compatibility).
#
# Usage:
#   $ dna::print_apptainer_exec_command "valeria" "./artifact/sif/myproject.sif" "sjob001" \
#       "launcher/train.py" "--epochs=10"
#
# Positional arguments:
#   profile         - HPC server profile name
#   sif_path        - Path to the SIF file on the HPC server
#   dna_sjob_name   - Slurm job name (appended to DN_CONTAINER_NAME at runtime)
#   python_args     - Python command and arguments (all remaining args)
#
# Outputs:
#   Writes the full apptainer exec command to stdout
#
# Returns:
#   0 on success
# =================================================================================================
function dna::print_apptainer_exec_command() {
  local profile="${1:?err}"
  local sif_path="${2:?err}"
  local dna_sjob_name="${3:?err}"
  shift 3
  local python_args=("$@")

  local exec_flags
  exec_flags=$(dna::get_apptainer_slurm_exec_flags "${profile}" "${sif_path}" "${dna_sjob_name}")

  local entrypoint="/dockerized-norlab/project/project-slurm/dn_entrypoint.init.bash"

  echo "# Apptainer exec command (run on HPC server, NOT locally)"
  echo "# Generated by: dna run slurm --generate-apptainer ${profile}"
  echo "apptainer exec \\"
  echo "${exec_flags}"
  echo "    \"\${SIF_PATH}\" \\"
  echo "    \"${entrypoint}\" \\"
  for arg in "${python_args[@]}"; do
    printf '    "%s" \\\n' "${arg}"
  done
  # Remove trailing backslash from last line by printing it separately
  return 0
}


# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dna_error_prefix="\033[1;31m[dna error]\033[0m"
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "${dna_error_prefix} This script must be sourced i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  # ....Pre-condition..............................................................................
  test -n "$( declare -f n2st::print_msg )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
fi
