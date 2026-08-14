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
# Normalizes an HPC server profile/target name into a tag- and filename-safe suffix.
#
# The suffix is appended to the slurm Docker image tag and to the Apptainer SIF filename so that
# artifacts produced for different targets (e.g., 'valeria', 'compute_canada') do not collide.
# The normalization replaces underscores with hyphens (e.g., 'compute_canada' -> 'compute-canada'),
# which is safe for both Docker tags and filenames.
#
# Usage:
#   $ suffix="$(dna::apptainer_target_suffix "compute_canada")"  # -> "compute-canada"
#
# Positional argument:
#   profile - HPC server profile/target name (e.g., 'valeria', 'compute_canada', 'mamba')
#
# Outputs:
#   Writes the normalized suffix to stdout.
#
# Returns:
#   0 on success, 1 if profile is empty
# =================================================================================================
function dna::apptainer_target_suffix() {
  local profile="${1:?err}"
  if [[ -z "${profile}" ]]; then
    return 1
  fi
  echo "${profile//_/-}"
  return 0
}


# =================================================================================================
# Generates a dna_tar_to_apptainer_sif_converter.sh helper script to be run on the HPC server.
#
# This script converts the Docker tar archive to an Apptainer SIF file.
#
# It is NOT executed locally — copy it to the HPC server alongside the tar archive.
# The HPC super-project directory structure should be set up beforehand by running
# dna_hpc_server_config.bash (generated by dna::generate_hpc_server_config_script).
#
# Usage:
#   $ dna::generate_apptainer_build_sif_script "myproject-slurm.latest.tar" "myproject-slurm-valeria.sif" "/output/dir" "valeria"
#
# Positional arguments:
#   tar_filename  - Docker tar archive filename (e.g., 'myproject-slurm.latest.tar')
#   sif_name      - Output SIF filename (e.g., 'myproject-slurm-valeria.sif'). Should already
#                   include the target-platform suffix (see dna::apptainer_target_suffix).
#   output_dir    - Directory where the dna_tar_to_apptainer_sif_converter.sh script will be written
#   profile       - Optional HPC server profile/target name (reserved; the SIF is output to
#                   ${SCRATCH}/sif/ on the HPC server regardless of the profile).
#
# The generated script builds the SIF into ${SCRATCH}/sif/ on the HPC server (a compute
# allocation is requested via salloc when not already inside a SLURM job).
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
set -e

# ====Documentation================================================================================
DOCUMENTATION_BUFFER=$( cat <<'EOF'
# =================================================================================================
# Auto-generated by DNA: dna build slurm --apptainer / dna save --apptainer
# Run this script on the HPC server AFTER transferring the artifact/apptainer/ directory.
#
# This script converts the Docker tar archive to an Apptainer SIF file.
# Prerequisite: run dna_hpc_server_config.bash once to set up the directory structure
# and authenticate with the Docker registry.
#
# Usage (from the super-project root directory on the HPC server):
#   $ bash artifact/apptainer/<target>/dna_tar_to_apptainer_sif_converter.sh [OPTIONS]
#
# Options:
#   --source-tar=<path/to/tar>
#       Optional. Path to the Docker tar archive to convert.
#       Defaults to the tar archive expected alongside this script.
#   -h | --help
#       Print this help message and exit.
#
# Environment variables (build tuning, all optional):
#   APPTAINER_ALLOW_ARCH_MISMATCH=1  Skip the abort when the tar's architecture does not match the
#       host (e.g. an arm64 tar on an x86_64 node); the container would then rely on emulation.
#   APPTAINER_BUILD_NO_SALLOC=1   Build in-place instead of re-running inside a compute allocation.
#   APPTAINER_BUILD_ACCOUNT=<acct>  SLURM account for the allocation (auto-detected otherwise).
#   APPTAINER_BUILD_MEM=<mem>       salloc --mem     (default 64G).
#   APPTAINER_BUILD_CPUS=<n>        salloc --cpus-per-task and mksquashfs -processors (default 10).
#   APPTAINER_BUILD_TIME=<hh:mm:ss> salloc --time    (default 6:00:00).
#   APPTAINER_BUILD_COMPRESS=<comp[:level]>
#       squashfs compressor for apptainer >= 1.4.0 (default none — smallest but slowest).
#       Faster alternatives: zstd:3, lz4, or none (near-uncompressed, fastest). Ignored on < 1.4.0.
#
# Requires:
#   - apptainer installed on the HPC server
#   - The Docker tar archive (.tar), either alongside this script or provided via
#     --source-tar=<path/to/tar>
#   - $SCRATCH environment variable set (the SIF file is output to ${SCRATCH}/sif/)
#
# =================================================================================================
EOF
)

# ====Argument parsing=============================================================================
# Capture the original invocation arguments BEFORE the parsing loop consumes them with `shift`,
# so the re-exec into a compute allocation (below) can faithfully replay them.
_ORIG_ARGS=("$@")
SOURCE_TAR=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h)
      echo -e "${DOCUMENTATION_BUFFER}" | sed '/# ===.*/d' | sed 's/^# //' | sed 's/^#//'
      exit 0
      ;;
    --source-tar=*)
      SOURCE_TAR="${1#--source-tar=}"
      shift
      ;;
    *)
      echo "[error] Unknown argument: $1" 1>&2
      echo "Usage: $0 [--source-tar=<path/to/tar>] [--help]" 1>&2
      exit 1
      ;;
  esac
done

# ====Apptainer compatibility======================================================================
echo "[info] This script requires Apptainer >= 1.1.0 (for --no-eval, --cleanenv, --env-file comment support)." 1>&2

# ====Validate environment=========================================================================
if [[ -z "${SCRATCH:-}" ]]; then
  echo "[error] \$SCRATCH is not set. Run this on an HPC login node (or export SCRATCH)." 1>&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_EOF

  # Inject the tar_filename and sif_name variables (expand at generation time)
  cat >> "${script_path}" << EOF
TAR_FILENAME="${tar_filename}"
SIF_FILENAME="${sif_name}"
EOF

  cat >> "${script_path}" << 'SCRIPT_EOF'

# When --source-tar=<path> is provided, use it as the input tar archive,
# otherwise default to the tar archive expected alongside this script.
if [[ -n "${SOURCE_TAR}" ]]; then
  TAR_FILE="${SOURCE_TAR}"
else
  TAR_FILE="${SCRIPT_DIR}/${TAR_FILENAME}"
fi
# Output the SIF to the scratch SIF cache (${SCRATCH}/sif/) instead of alongside this script,
# matching the SIF path lookup in the generated SLURM job scripts.
SIF_DIR="${SCRATCH}/sif"
SIF_FILE="${SIF_DIR}/${SIF_FILENAME}"
mkdir -p "${SIF_DIR}"

# ====Convert tar archive to Apptainer SIF========================================================
if [[ ! -f "${TAR_FILE}" ]]; then
  echo "[error] Docker tar archive not found: ${TAR_FILE}" 1>&2
  exit 1
fi

echo "[info] Building Apptainer SIF from Docker tar archive..." 1>&2
echo "[info]   Input:  ${TAR_FILE}" 1>&2
echo "[info]   Output: ${SIF_FILE}" 1>&2

# ====Architecture guard (fail fast on a wrong-platform tar)======================================
# A multiarch image tag says nothing about whether the tar holds the linux/amd64 or linux/arm64
# manifest; the architecture is decided by how `dna` produced the tar. A wrong-arch tar does NOT
# fail at conversion time (apptainer happily builds it), but the container then silently misbehaves
# on the compute node via emulation. Read the docker-archive's embedded image config `architecture`
# field and compare it to the host arch. Override with APPTAINER_ALLOW_ARCH_MISMATCH=1.
if command -v tar &>/dev/null; then
  case "$( uname -m )" in
    x86_64|amd64)        _HOST_ARCH="amd64" ;;
    aarch64|arm64)       _HOST_ARCH="arm64" ;;
    *)                   _HOST_ARCH="$( uname -m )" ;;
  esac
  _TAR_MANIFEST="$( tar -xOf "${TAR_FILE}" manifest.json 2>/dev/null || true )"
  _TAR_CONFIG_PATH="$( echo "${_TAR_MANIFEST}" | grep -oE '"Config"[[:space:]]*:[[:space:]]*"[^"]+"' | head -1 | sed -E 's/.*"Config"[[:space:]]*:[[:space:]]*"([^"]+)".*/\1/' )"
  _TAR_ARCH=""
  if [[ -n "${_TAR_CONFIG_PATH}" ]]; then
    _TAR_ARCH="$( tar -xOf "${TAR_FILE}" "${_TAR_CONFIG_PATH}" 2>/dev/null \
      | grep -oE '"architecture"[[:space:]]*:[[:space:]]*"[^"]+"' | head -1 \
      | sed -E 's/.*"architecture"[[:space:]]*:[[:space:]]*"([^"]+)".*/\1/' )"
  fi
  if [[ -z "${_TAR_ARCH}" ]]; then
    echo "[warn] Could not determine the architecture of the tar archive; skipping the architecture guard." 1>&2
  elif [[ "${_TAR_ARCH}" != "${_HOST_ARCH}" ]]; then
    echo "[error] Architecture mismatch: the tar archive is a linux/${_TAR_ARCH} image but this host is ${_HOST_ARCH} (uname -m: $( uname -m ))." 1>&2
    echo "[error]   Running a ${_TAR_ARCH} container on a ${_HOST_ARCH} node relies on emulation and the DNA/N2ST" 1>&2
    echo "[error]   entrypoint bootstrap fails inside the SLURM job (N2ST_PATH never gets set)." 1>&2
    echo "[hint]  The image tag is arch-agnostic for a multiarch image — re-save the tar for the" 1>&2
    echo "[hint]   target platform, e.g.:  dna build slurm --apptainer <target> --save" 1>&2
    echo "[hint]  Or build the SIF directly from the registry with the" 1>&2
    echo "[hint]   dna_registry_to_apptainer_sif_converter.sh script (dna build slurm --apptainer <target> --push)." 1>&2
    echo "[hint]  To build anyway despite the mismatch, re-run with APPTAINER_ALLOW_ARCH_MISMATCH=1." 1>&2
    if [[ "${APPTAINER_ALLOW_ARCH_MISMATCH:-0}" == "1" ]]; then
      echo "[warn] APPTAINER_ALLOW_ARCH_MISMATCH=1 set — proceeding despite the architecture mismatch." 1>&2
    else
      exit 1
    fi
  else
    echo "[info] Architecture check passed: tar archive is linux/${_TAR_ARCH}, matching host ${_HOST_ARCH}." 1>&2
  fi
fi

SCRIPT_EOF

  cat >> "${script_path}" << 'SCRIPT_EOF'
# ====Login-node memory-cap mitigation + fast local-disk staging (re-exec into allocation)========
# Building a SIF from a multi-GB docker-archive is memory- and I/O-heavy. On Alliance/Compute Canada
# login nodes, per-user memory is capped and heavy processes are SIGKILLed ("Killed"), so the build
# must run inside a compute allocation. We RE-EXEC this whole script inside the allocation so the
# inner run sees ${SLURM_TMPDIR} (the compute node's fast local disk) for APPTAINER_TMPDIR/CACHEDIR.
# Override resources with APPTAINER_BUILD_MEM/CPUS/TIME/ACCOUNT, or disable with APPTAINER_BUILD_NO_SALLOC=1.
# Ref: https://docs.alliancecan.ca/wiki/Apptainer
if [[ -z "${SLURM_JOB_ID:-}" && "${APPTAINER_BUILD_NO_SALLOC:-0}" != "1" ]] && command -v salloc &>/dev/null; then
  _SALLOC_ARGS=(
    --time="${APPTAINER_BUILD_TIME:-6:00:00}"
    --mem="${APPTAINER_BUILD_MEM:-64G}"
    --cpus-per-task="${APPTAINER_BUILD_CPUS:-10}"
  )
  _SALLOC_ACCOUNT="${APPTAINER_BUILD_ACCOUNT:-${SLURM_ACCOUNT:-}}"
  if [[ -z "${_SALLOC_ACCOUNT}" ]] && command -v sacctmgr &>/dev/null; then
    _SALLOC_ACCOUNT="$( sacctmgr -nP show user "${USER}" format=defaultaccount 2>/dev/null | head -1 )"
    if [[ -z "${_SALLOC_ACCOUNT}" ]]; then
      _SALLOC_ACCOUNT="$( sacctmgr -nP show associations user="${USER}" format=account 2>/dev/null | grep -v '^$' | head -1 )"
    fi
    if [[ -n "${_SALLOC_ACCOUNT}" ]]; then
      echo "[info] Auto-detected SLURM account: ${_SALLOC_ACCOUNT} (override with APPTAINER_BUILD_ACCOUNT)." 1>&2
    fi
  fi
  if [[ -n "${_SALLOC_ACCOUNT}" ]]; then
    _SALLOC_ARGS+=( --account="${_SALLOC_ACCOUNT}" )
  else
    echo "[warn] No SLURM account could be determined. If salloc fails with 'Please specify one of" 1>&2
    echo "[warn]   the following accounts', set APPTAINER_BUILD_ACCOUNT=<account> and re-run." 1>&2
  fi
  _SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  echo "[info] Not inside a SLURM job — re-running this script inside a compute allocation so the" 1>&2
  echo "[info]   heavy build gets enough RAM and stages on the node's fast local disk (\$SLURM_TMPDIR)." 1>&2
  echo "[info]   salloc ${_SALLOC_ARGS[*]} srun bash ${_SELF} ${_ORIG_ARGS[*]}" 1>&2
  # Guard against an infinite re-exec loop if SLURM_JOB_ID is somehow unset inside the job.
  export APPTAINER_BUILD_NO_SALLOC=1
  exec salloc "${_SALLOC_ARGS[@]}" srun bash "${_SELF}" "${_ORIG_ARGS[@]}"
fi
echo "[info] Running the Apptainer build in-place (inside a SLURM job, salloc unavailable, or APPTAINER_BUILD_NO_SALLOC=1)." 1>&2

# ====Apptainer cache configuration================================================================
# Prefer SLURM_TMPDIR (fast local disk, available thanks to the re-exec above) when inside a job,
# otherwise fall back to ${SCRATCH}/tmp (a large disk-backed filesystem, NOT the small RAM-backed
# /tmp which would OOM-Kill the build). Ref: https://docs.alliancecan.ca/wiki/Apptainer
_APPTAINER_SCRATCH_TMP_ROOT="${SLURM_TMPDIR:-${SCRATCH}/tmp}"
mkdir -p "${_APPTAINER_SCRATCH_TMP_ROOT}"
export APPTAINER_CACHEDIR="$( mktemp -d -p "${_APPTAINER_SCRATCH_TMP_ROOT}" )"
export APPTAINER_TMPDIR="$( mktemp -d -p "${_APPTAINER_SCRATCH_TMP_ROOT}" )"

# ====Load Apptainer module (HPC module system)====================================================
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

# Build the SIF to APPTAINER_TMPDIR (scratch space, set above), then move it to the final destination.
SIF_STAGING_DIR="${APPTAINER_TMPDIR:-/tmp}"
SIF_TMP="${SIF_STAGING_DIR}/${SIF_FILENAME}"

echo "[info]   Staging: ${SIF_TMP}" 1>&2

# ====Squashfs compression tuning=================================================================
# apptainer >= 1.4.0 accepts --mksquashfs-args. Tune via APPTAINER_BUILD_COMPRESS="<comp>[:<level>]"
# (default none — fastest, largest SIF). Faster-compressed alternatives: zstd:3, lz4.
_APPTAINER_VERSION="$( apptainer --version 2>/dev/null | grep -oP '[0-9]+\.[0-9]+\.[0-9]+' | head -1 )"
_APPTAINER_MAJOR="$( echo "${_APPTAINER_VERSION}" | cut -d. -f1 )"
_APPTAINER_MINOR="$( echo "${_APPTAINER_VERSION}" | cut -d. -f2 )"
_APPTAINER_BUILD_ARGS=()
if [[ "${_APPTAINER_MAJOR}" -gt 1 ]] || { [[ "${_APPTAINER_MAJOR}" -eq 1 ]] && [[ "${_APPTAINER_MINOR}" -ge 4 ]]; }; then
  _COMPRESS_SPEC="${APPTAINER_BUILD_COMPRESS:-none}"
  _COMP_NAME="${_COMPRESS_SPEC%%:*}"
  _COMP_LEVEL="${_COMPRESS_SPEC#*:}"
  [[ "${_COMP_LEVEL}" == "${_COMPRESS_SPEC}" ]] && _COMP_LEVEL=""
  _MKSQUASHFS_ARGS=()
  if [[ "${_COMP_NAME}" == "none" ]]; then
    _MKSQUASHFS_ARGS=(-noD -noF -noI -noX)
  else
    _MKSQUASHFS_ARGS=(-comp "${_COMP_NAME}")
    if [[ -n "${_COMP_LEVEL}" ]]; then
      case "${_COMP_NAME}" in
        zstd|gzip|xz|lzma) _MKSQUASHFS_ARGS+=(-Xcompression-level "${_COMP_LEVEL}") ;;
        *) echo "[warn] Ignoring compression level '${_COMP_LEVEL}' — not supported for compressor '${_COMP_NAME}'." 1>&2 ;;
      esac
    fi
  fi
  _MKSQUASHFS_PROCS="${APPTAINER_BUILD_CPUS:-${SLURM_CPUS_PER_TASK:-}}"
  if [[ -n "${_MKSQUASHFS_PROCS}" ]]; then
    _MKSQUASHFS_ARGS+=(-processors "${_MKSQUASHFS_PROCS}")
  fi
  echo "[info] Apptainer ${_APPTAINER_VERSION}: squashfs args: ${_MKSQUASHFS_ARGS[*]}" 1>&2
  _APPTAINER_BUILD_ARGS+=(--mksquashfs-args="${_MKSQUASHFS_ARGS[*]}")
else
  echo "[info] Apptainer ${_APPTAINER_VERSION}: --mksquashfs-args unsupported (requires >= 1.4.0); using defaults (APPTAINER_BUILD_COMPRESS ignored)." 1>&2
fi

if ! apptainer build "${_APPTAINER_BUILD_ARGS[@]}" \
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
# The HPC super-project directory structure should be set up beforehand by running
# dna_hpc_server_config.bash (generated by dna::generate_hpc_server_config_script).
#
# Usage:
#   $ dna::generate_registry_to_apptainer_sif_script "hub/image:tag-valeria" "myproject-slurm-valeria.sif" "/output/dir" "valeria"
#
# Positional arguments:
#   image_ref   - Full Docker image reference (e.g., 'norlabulaval/myproject-slurm:tag-valeria')
#   sif_name    - Output SIF filename (e.g., 'myproject-slurm-valeria.sif'). Should already
#                 include the target-platform suffix (see dna::apptainer_target_suffix).
#   output_dir  - Directory where the dna_registry_to_apptainer_sif_converter.sh script will be written
#   profile     - Optional HPC server profile/target name (reserved; the SIF is output to
#                 ${SCRATCH}/sif/ on the HPC server regardless of the profile).
#
# The generated script builds the SIF into ${SCRATCH}/sif/ on the HPC server (a compute
# allocation is requested via salloc when not already inside a SLURM job).
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
set -e

# ====Documentation================================================================================
DOCUMENTATION_BUFFER=$( cat <<'EOF'
# =================================================================================================
# Auto-generated by DNA: dna build slurm --apptainer <profile> --push
# Run this script on the HPC server to build an Apptainer SIF from a Docker registry image.
#
# This script:
#   1. Optionally authenticates to the Docker registry interactively via --docker-login.
#   2. Builds the Apptainer SIF to APPTAINER_TMPDIR (SLURM_TMPDIR-aware mktemp scratch)
#      then moves it to ${SCRATCH}/sif/ to avoid Lustre quota.
#
# Prerequisite: run dna_hpc_server_config.bash once to set up the directory structure
# and authenticate with the Docker registry.
#
# Usage (from the super-project root directory on the HPC server):
#   $ bash artifact/apptainer/<target>/dna_registry_to_apptainer_sif_converter.sh [OPTIONS]
#
# Options:
#   --docker-login
#       Optional. Pass this flag to authenticate interactively with the Docker registry
#       (docker.io) before building the SIF. Apptainer will prompt for credentials.
#       Note: the interactive prompt does not work when the build is re-run inside a SLURM
#       allocation (see below); prefer `apptainer registry login` beforehand (run by
#       dna_hpc_server_config.bash), or combine with APPTAINER_BUILD_NO_SALLOC=1.
#   -h | --help
#       Print this help message and exit.
#
# Environment variables (build tuning, all optional):
#   APPTAINER_BUILD_NO_SALLOC=1   Build in-place instead of re-running inside a compute allocation.
#   APPTAINER_BUILD_ACCOUNT=<acct>  SLURM account for the allocation (auto-detected otherwise).
#   APPTAINER_BUILD_MEM=<mem>       salloc --mem     (default 64G).
#   APPTAINER_BUILD_CPUS=<n>        salloc --cpus-per-task and mksquashfs -processors (default 10).
#   APPTAINER_BUILD_TIME=<hh:mm:ss> salloc --time    (default 6:00:00).
#   APPTAINER_BUILD_COMPRESS=<comp[:level]>
#       squashfs compressor for apptainer >= 1.4.0 (default none — smallest but slowest).
#       Faster alternatives: zstd:3, lz4, or none (near-uncompressed, fastest). Ignored on < 1.4.0.
#
# Requires:
#   - apptainer installed on the HPC server
#   - Network access to docker.io from the HPC server
#   - $SCRATCH environment variable set (the SIF file is output to ${SCRATCH}/sif/)
#
# =================================================================================================
EOF
)

# ====Argument parsing=============================================================================
USE_DOCKER_LOGIN=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h)
      echo -e "${DOCUMENTATION_BUFFER}" | sed '/# ===.*/d' | sed 's/^# //' | sed 's/^#//'
      exit 0
      ;;
    --docker-login)
      USE_DOCKER_LOGIN=true
      shift
      ;;
    *)
      echo "[error] Unknown argument: $1" 1>&2
      echo "Usage: $0 [--docker-login] [--help]" 1>&2
      exit 1
      ;;
  esac
done

# ====Apptainer compatibility======================================================================
echo "[info] This script requires Apptainer >= 1.1.0." 1>&2

# ====Validate environment=========================================================================
if [[ -z "${SCRATCH:-}" ]]; then
  echo "[error] \$SCRATCH is not set. Run this on an HPC login node (or export SCRATCH)." 1>&2
  exit 1
fi
SCRIPT_EOF

  # Inject the image_ref and sif_name variables (expand at generation time)
  cat >> "${script_path}" << EOF
IMAGE_REF="${image_ref}"
SIF_FILENAME="${sif_name}"
EOF

  cat >> "${script_path}" << 'SCRIPT_EOF'

# Output the SIF to the scratch SIF cache (${SCRATCH}/sif/), matching the SIF path lookup in the
# generated SLURM job scripts.
SIF_DIR="${SCRATCH}/sif"
SIF_FILE="${SIF_DIR}/${SIF_FILENAME}"
mkdir -p "${SIF_DIR}"

echo "[info] Image reference: ${IMAGE_REF}" 1>&2
echo "[info] SIF output:      ${SIF_FILE}" 1>&2

# ====Login-node memory-cap mitigation + fast local-disk staging (re-exec into allocation)========
# Building a SIF is memory- and I/O-heavy. On Alliance/Compute Canada login nodes, per-user memory
# is capped and heavy processes are SIGKILLed ("Killed"), so the build must run inside a compute
# allocation. We RE-EXEC this whole script inside the allocation so the inner run sees
# ${SLURM_TMPDIR} (the compute node's fast local disk) for APPTAINER_TMPDIR/CACHEDIR. Override
# resources with APPTAINER_BUILD_MEM/CPUS/TIME/ACCOUNT, or disable with APPTAINER_BUILD_NO_SALLOC=1.
# Ref: https://docs.alliancecan.ca/wiki/Apptainer
if [[ -z "${SLURM_JOB_ID:-}" && "${APPTAINER_BUILD_NO_SALLOC:-0}" != "1" ]] && command -v salloc &>/dev/null; then
  if [[ "${USE_DOCKER_LOGIN}" == true ]]; then
    echo "[warn] --docker-login prompts interactively, which does not work inside a SLURM allocation." 1>&2
    echo "[warn]   Authenticate first with 'apptainer registry login docker://docker.io' (done by" 1>&2
    echo "[warn]   dna_hpc_server_config.bash), or re-run with APPTAINER_BUILD_NO_SALLOC=1." 1>&2
  fi
  _SALLOC_ARGS=(
    --time="${APPTAINER_BUILD_TIME:-6:00:00}"
    --mem="${APPTAINER_BUILD_MEM:-64G}"
    --cpus-per-task="${APPTAINER_BUILD_CPUS:-10}"
  )
  _SALLOC_ACCOUNT="${APPTAINER_BUILD_ACCOUNT:-${SLURM_ACCOUNT:-}}"
  if [[ -z "${_SALLOC_ACCOUNT}" ]] && command -v sacctmgr &>/dev/null; then
    _SALLOC_ACCOUNT="$( sacctmgr -nP show user "${USER}" format=defaultaccount 2>/dev/null | head -1 )"
    if [[ -z "${_SALLOC_ACCOUNT}" ]]; then
      _SALLOC_ACCOUNT="$( sacctmgr -nP show associations user="${USER}" format=account 2>/dev/null | grep -v '^$' | head -1 )"
    fi
    if [[ -n "${_SALLOC_ACCOUNT}" ]]; then
      echo "[info] Auto-detected SLURM account: ${_SALLOC_ACCOUNT} (override with APPTAINER_BUILD_ACCOUNT)." 1>&2
    fi
  fi
  if [[ -n "${_SALLOC_ACCOUNT}" ]]; then
    _SALLOC_ARGS+=( --account="${_SALLOC_ACCOUNT}" )
  else
    echo "[warn] No SLURM account could be determined. If salloc fails with 'Please specify one of" 1>&2
    echo "[warn]   the following accounts', set APPTAINER_BUILD_ACCOUNT=<account> and re-run." 1>&2
  fi
  _SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  echo "[info] Not inside a SLURM job — re-running this script inside a compute allocation so the" 1>&2
  echo "[info]   heavy build gets enough RAM and stages on the node's fast local disk (\$SLURM_TMPDIR)." 1>&2
  echo "[info]   salloc ${_SALLOC_ARGS[*]} srun bash ${_SELF} $*" 1>&2
  # Guard against an infinite re-exec loop if SLURM_JOB_ID is somehow unset inside the job.
  export APPTAINER_BUILD_NO_SALLOC=1
  exec salloc "${_SALLOC_ARGS[@]}" srun bash "${_SELF}" "$@"
fi
echo "[info] Running the Apptainer build in-place (inside a SLURM job, salloc unavailable, or APPTAINER_BUILD_NO_SALLOC=1)." 1>&2

# ====Apptainer cache configuration================================================================
# Prefer SLURM_TMPDIR (fast local disk, available thanks to the re-exec above) when inside a job,
# otherwise fall back to ${SCRATCH}/tmp (a large disk-backed filesystem, NOT the small RAM-backed
# /tmp which would OOM-Kill the build). Ref: https://docs.alliancecan.ca/wiki/Apptainer
_APPTAINER_SCRATCH_TMP_ROOT="${SLURM_TMPDIR:-${SCRATCH}/tmp}"
mkdir -p "${_APPTAINER_SCRATCH_TMP_ROOT}"
export APPTAINER_CACHEDIR="$( mktemp -d -p "${_APPTAINER_SCRATCH_TMP_ROOT}" )"
export APPTAINER_TMPDIR="$( mktemp -d -p "${_APPTAINER_SCRATCH_TMP_ROOT}" )"

# ====Load Apptainer module (HPC module system)====================================================
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

# ====Build SIF from registry image===============================================================
# Note: --disable-cache is NOT used here because APPTAINER_CACHEDIR is already redirected to
# scratch space above, so the cache never lands in the home directory.
if [[ "${USE_DOCKER_LOGIN}" == true ]]; then
  echo "[info] Authenticating with docker.io interactively (--docker-login)..." 1>&2
  _APPTAINER_EXTRA_FLAGS=(--docker-login)
else
  echo "[info] No --docker-login flag provided — assuming public registry access." 1>&2
  _APPTAINER_EXTRA_FLAGS=()
fi

# Build the SIF to APPTAINER_TMPDIR (scratch space, set above), then move it to the final destination.
SIF_STAGING_DIR="${APPTAINER_TMPDIR:-/tmp}"
SIF_TMP="${SIF_STAGING_DIR}/${SIF_FILENAME}"

echo "[info] Building Apptainer SIF from Docker registry..." 1>&2
echo "[info]   Source: docker://${IMAGE_REF}" 1>&2
echo "[info]   Staging: ${SIF_TMP}" 1>&2
echo "[info]   Output:  ${SIF_FILE}" 1>&2

# ====Squashfs compression tuning=================================================================
# apptainer >= 1.4.0 accepts --mksquashfs-args. Tune via APPTAINER_BUILD_COMPRESS="<comp>[:<level>]"
# (default none — fastest, largest SIF). Faster-compressed alternatives: zstd:3, lz4.
_APPTAINER_VERSION="$( apptainer --version 2>/dev/null | grep -oP '[0-9]+\.[0-9]+\.[0-9]+' | head -1 )"
_APPTAINER_MAJOR="$( echo "${_APPTAINER_VERSION}" | cut -d. -f1 )"
_APPTAINER_MINOR="$( echo "${_APPTAINER_VERSION}" | cut -d. -f2 )"
_APPTAINER_BUILD_ARGS=()
if [[ "${_APPTAINER_MAJOR}" -gt 1 ]] || { [[ "${_APPTAINER_MAJOR}" -eq 1 ]] && [[ "${_APPTAINER_MINOR}" -ge 4 ]]; }; then
  _COMPRESS_SPEC="${APPTAINER_BUILD_COMPRESS:-none}"
  _COMP_NAME="${_COMPRESS_SPEC%%:*}"
  _COMP_LEVEL="${_COMPRESS_SPEC#*:}"
  [[ "${_COMP_LEVEL}" == "${_COMPRESS_SPEC}" ]] && _COMP_LEVEL=""
  _MKSQUASHFS_ARGS=()
  if [[ "${_COMP_NAME}" == "none" ]]; then
    _MKSQUASHFS_ARGS=(-noD -noF -noI -noX)
  else
    _MKSQUASHFS_ARGS=(-comp "${_COMP_NAME}")
    if [[ -n "${_COMP_LEVEL}" ]]; then
      case "${_COMP_NAME}" in
        zstd|gzip|xz|lzma) _MKSQUASHFS_ARGS+=(-Xcompression-level "${_COMP_LEVEL}") ;;
        *) echo "[warn] Ignoring compression level '${_COMP_LEVEL}' — not supported for compressor '${_COMP_NAME}'." 1>&2 ;;
      esac
    fi
  fi
  _MKSQUASHFS_PROCS="${APPTAINER_BUILD_CPUS:-${SLURM_CPUS_PER_TASK:-}}"
  if [[ -n "${_MKSQUASHFS_PROCS}" ]]; then
    _MKSQUASHFS_ARGS+=(-processors "${_MKSQUASHFS_PROCS}")
  fi
  echo "[info] Apptainer ${_APPTAINER_VERSION}: squashfs args: ${_MKSQUASHFS_ARGS[*]}" 1>&2
  _APPTAINER_BUILD_ARGS+=(--mksquashfs-args="${_MKSQUASHFS_ARGS[*]}")
else
  echo "[info] Apptainer ${_APPTAINER_VERSION}: --mksquashfs-args unsupported (requires >= 1.4.0); using defaults (APPTAINER_BUILD_COMPRESS ignored)." 1>&2
fi

if ! apptainer build "${_APPTAINER_BUILD_ARGS[@]}" \
    "${_APPTAINER_EXTRA_FLAGS[@]}" \
    "${SIF_TMP}" \
    "docker://${IMAGE_REF}"; then
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
# Generates a dna_hpc_server_config.bash helper script to be run on the HPC server.
#
# This script:
#   1. Sets up the expected super-project directory structure on the HPC server.
#   2. Loads the Apptainer module (module load apptainer).
#   3. Runs 'apptainer registry login docker.io' interactively so that the user can provide
#      their Docker Hub username and password (no secret management required).
#
# It is NOT executed locally — run it once on the HPC server after cloning/transferring the
# super-project to initialize the directory structure and authenticate with the Docker registry.
#
# Expected HPC super-project directory structure (created by this script):
#   super-project/
#     ├── .dockerized_norlab/          ← DNA configuration
#     ├── artifact/apptainer/          ← SIF files and converter scripts
#     ├── artifact/optuna_storage/
#     ├── artifact/slurm_jobs_logs/
#     ├── artifact/tensorboard_tmp/
#     ├── data/external_data/
#     ├── data/repository_data/
#     ├── data/shared_data/
#     └── slurm_jobs/                  ← sbatch scripts
#
# Usage:
#   $ dna::generate_hpc_server_config_script "/output/dir" ["valeria"]
#
# Positional arguments:
#   output_dir  - Directory where dna_hpc_server_config.bash will be written
#   profile     - Optional HPC server profile name (e.g., 'valeria'). Reserved for future
#                 profile-specific logic; currently no profile-specific branching is applied.
#
# Outputs:
#   Writes dna_hpc_server_config.bash to output_dir
#
# Returns:
#   0 on success, 1 on failure
# =================================================================================================
function dna::generate_hpc_server_config_script() {
  local output_dir="${1:?err}"
  local profile="${2:-}"

  local script_path="${output_dir}/dna_hpc_server_config.bash"

  n2st::print_msg "Generating HPC server config script: ${script_path}"

  cat > "${script_path}" << 'SCRIPT_EOF'
#!/bin/bash
set -e

# ====Documentation================================================================================
DOCUMENTATION_BUFFER=$( cat <<'EOF'
# =================================================================================================
# Auto-generated by DNA
# Run this script ONCE on the HPC server after cloning/transferring the super-project to:
#   1. Set up the expected super-project directory structure.
#   2. Load the Apptainer module.
#   3. Authenticate with the Docker registry interactively.
#
# Usage (from the super-project root directory on the HPC server):
#   $ bash artifact/apptainer/<target>/dna_hpc_server_config.bash [--target-dir <TARGET-DIRECTORY-PATH>]
#
# Options:
#   --target-dir <TARGET-DIRECTORY-PATH>
#       Optional. Path to the HPC super-project root directory.
#       When specified, the directory structure will be created under this path.
#       Defaults to three levels above this script's directory (i.e., the super-project root
#       inferred from the standard artifact/apptainer/<target>/ placement).
#   -h | --help
#       Print this help message and exit.
#
# Requires:
#   - Apptainer (or module load apptainer) available on the HPC server
#
# =================================================================================================
EOF
)

# ====Argument parsing=============================================================================
TARGET_DIR=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h)
      echo -e "${DOCUMENTATION_BUFFER}" | sed '/# ===.*/d' | sed 's/^# //' | sed 's/^#//'
      exit 0
      ;;
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
      echo "Usage: $0 [--target-dir <TARGET-DIRECTORY-PATH>] [--help]" 1>&2
      exit 1
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SUPER_PROJECT_ROOT="${TARGET_DIR:-${SUPER_PROJECT_ROOT:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}}"

echo "[info] HPC super-project root: ${SUPER_PROJECT_ROOT}" 1>&2

SCRIPT_EOF

  cat >> "${script_path}" << 'SCRIPT_EOF'
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
  # src/ and utilities/ are bind-mounted read-only at apptainer exec time.
  # Mounting them enables fast code iteration: rsync modified code to the HPC server
  # and re-run without rebuilding the Docker image or converting to SIF each time.
  "src"
  "utilities"
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

# ====Setup Apptainer SIF cache directory (${SCRATCH}/sif)========================================
# The SIF converter scripts (dna_tar_to_apptainer_sif_converter.sh /
# dna_registry_to_apptainer_sif_converter.sh) output the built SIF to ${SCRATCH}/sif/.
if [[ -n "${SCRATCH:-}" ]]; then
  echo "[info] Ensuring Apptainer SIF cache directory exists: ${SCRATCH}/sif" 1>&2
  mkdir -p "${SCRATCH}/sif"
else
  echo "[warn] \$SCRATCH is not set — the SIF converter scripts output to \${SCRATCH}/sif/ and will fail without it." 1>&2
fi

# ====Load Apptainer module (HPC module system)====================================================
if command -v module &>/dev/null; then
  # Try to load the highest available apptainer version; fallback to default
  _APPTAINER_LATEST_VERSION="$( module spider apptainer 2>&1 | grep -oE 'apptainer/[0-9]+.[0-9]+.[0-9]+' | sed 's|apptainer/||' | sort -V | tail -1 )"
  if [[ -n "${_APPTAINER_LATEST_VERSION}" ]]; then
    echo "[info] Loading Apptainer module version: ${_APPTAINER_LATEST_VERSION}" 1>&2
    module load "apptainer/${_APPTAINER_LATEST_VERSION}"
  else
    echo "[info] Loading default Apptainer module" 1>&2
    module load apptainer
  fi
else
  echo "[info] No 'module' command found — assuming apptainer is already in PATH." 1>&2
fi

# ====Authenticate with Docker registry interactively=============================================
echo "[info] Authenticating with Docker registry (docker.io)..." 1>&2
echo "[info] You will be prompted for your Docker Hub username and password." 1>&2
read -rp "[prompt] Docker Hub username: " _DNA_DOCKER_HUB_USERNAME
apptainer registry login --username "${_DNA_DOCKER_HUB_USERNAME}" docker://docker.io
echo "[done] Docker registry authentication complete." 1>&2
SCRIPT_EOF

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
  # Mount src/ and utilities/ read-only to enable fast code iteration without rebuilding the SIF.
  # This allows rsyncing modified code to the HPC server and re-running without rebuilding the
  # Docker image, pushing to the registry, pulling on HPC, or converting to SIF.
  # Git metadata is forwarded so tools inside the container can resolve the repo state.
  flags+=("    --bind ${super_project_root}/src/:${dn_project_path}/src/:ro \\")
  flags+=("    --bind ${super_project_root}/utilities/:${dn_project_path}/utilities/:ro \\")
  flags+=("    --env GIT_DIR=\${DN_PROJECT_PATH}/.git \\")

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
