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

  # Remove any previous (possibly read-only) generated script so we can overwrite it cleanly.
  rm -f "${script_path}"

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
  # SUPER_PROJECT_GIT_DIRNAME is the super-project repository directory baked under /ros2_ws/src/
  # in the image; the content guard validates THAT specific '.git' (see _dna_validate_sif_baked_git).
  # Fail loudly at generation time if it is unknown: without it the guard can only check sibling
  # repos and would silently weaken to a non-specific check.
  if [[ -z "${SUPER_PROJECT_REPO_NAME:-}" ]]; then
    echo "[warn] SUPER_PROJECT_REPO_NAME is empty while generating $(basename "${script_path}"); the" 1>&2
    echo "[warn]   baked-in '.git' content guard cannot target the specific super-project repo and" 1>&2
    echo "[warn]   will only validate sibling repos. Run 'dna' from the super-project so it is set." 1>&2
  fi
  cat >> "${script_path}" << EOF
TAR_FILENAME="${tar_filename}"
SIF_FILENAME="${sif_name}"
SUPER_PROJECT_GIT_DIRNAME="${SUPER_PROJECT_REPO_NAME:-}"
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
if [[ -n "${SLURM_JOB_ID:-}" ]]; then
  echo "[info] Running the Apptainer build on compute node ${SLURMD_NODENAME:-$(hostname -s)} (SLURM job ${SLURM_JOB_ID}) — this is the expected path." 1>&2
elif ! command -v salloc &>/dev/null; then
  echo "[warn] Running the Apptainer build in-place: 'salloc' is unavailable, so the build runs HERE (likely the login node). On Alliance/Compute Canada this risks an OOM SIGKILL — prefer running inside a compute allocation." 1>&2
else
  echo "[warn] Running the Apptainer build in-place because APPTAINER_BUILD_NO_SALLOC=1: the build runs HERE (likely the login node). On Alliance/Compute Canada this risks an OOM SIGKILL — unset APPTAINER_BUILD_NO_SALLOC to build inside a compute allocation." 1>&2
fi

# ====Apptainer cache configuration================================================================
# Prefer SLURM_TMPDIR (fast local disk, available thanks to the re-exec above) when inside a job,
# otherwise fall back to ${SCRATCH}/tmp (a large disk-backed filesystem, NOT the small RAM-backed
# /tmp which would OOM-Kill the build). Ref: https://docs.alliancecan.ca/wiki/Apptainer
# NOTE: Alliance Canada recommends APPTAINER_TMPDIR/CACHEDIR be on a NON-Lustre/GPFS filesystem
#   (${SLURM_TMPDIR}/local disk). ${SCRATCH} is Lustre on Alliance clusters, so the fallback below is
#   only a best-effort last resort; warn loudly so the user knows to build inside a compute allocation.
if [[ -z "${SLURM_TMPDIR:-}" ]]; then
  echo "[warn] SLURM_TMPDIR is not set: falling back to APPTAINER_TMPDIR/CACHEDIR under ${SCRATCH}/tmp," 1>&2
  echo "[warn]   which is a Lustre filesystem on Alliance Canada. Building Apptainer images on Lustre is" 1>&2
  echo "[warn]   discouraged (missing features for --fakeroot/overlay) and slow. Prefer running this" 1>&2
  echo "[warn]   converter inside a compute allocation (the default salloc re-exec) so ${SLURM_TMPDIR}" 1>&2
  echo "[warn]   (fast, node-local, non-Lustre disk) is used instead. Ref: https://docs.alliancecan.ca/wiki/Apptainer" 1>&2
fi
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
# apptainer >= 1.4.0 accepts --mksquashfs-args. Tune via APPTAINER_BUILD_COMPRESS="<comp>[:<level>]".
# Default is 'default': let Apptainer/mksquashfs pick its own compressor. Measured on a real DNA
# slurm image, the uncompressed variant ('none' -> -noD -noF -noI -noX) produced a 21.7 GB SIF vs
# 13.2 GB with the default compressor for byte-identical content — a much heavier read from the
# shared filesystem at every job start, for no benefit. Other values: none, zstd:3, lz4, gzip:6.
_APPTAINER_VERSION="$( apptainer --version 2>/dev/null | grep -oP '[0-9]+\.[0-9]+\.[0-9]+' | head -1 )"
_APPTAINER_MAJOR="$( echo "${_APPTAINER_VERSION}" | cut -d. -f1 )"
_APPTAINER_MINOR="$( echo "${_APPTAINER_VERSION}" | cut -d. -f2 )"
_APPTAINER_BUILD_ARGS=()
if [[ "${_APPTAINER_MAJOR}" -gt 1 ]] || { [[ "${_APPTAINER_MAJOR}" -eq 1 ]] && [[ "${_APPTAINER_MINOR}" -ge 4 ]]; }; then
  _COMPRESS_SPEC="${APPTAINER_BUILD_COMPRESS:-default}"
  _COMP_NAME="${_COMPRESS_SPEC%%:*}"
  _COMP_LEVEL="${_COMPRESS_SPEC#*:}"
  [[ "${_COMP_LEVEL}" == "${_COMPRESS_SPEC}" ]] && _COMP_LEVEL=""
  _MKSQUASHFS_ARGS=()
  if [[ "${_COMP_NAME}" == "default" ]]; then
    : # Let Apptainer/mksquashfs pick the compressor (smallest robust SIF, no tuning).
  elif [[ "${_COMP_NAME}" == "none" ]]; then
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
  if [[ ${#_MKSQUASHFS_ARGS[@]} -gt 0 ]]; then
    echo "[info] Apptainer ${_APPTAINER_VERSION}: squashfs args: ${_MKSQUASHFS_ARGS[*]}" 1>&2
    _APPTAINER_BUILD_ARGS+=(--mksquashfs-args="${_MKSQUASHFS_ARGS[*]}")
  else
    echo "[info] Apptainer ${_APPTAINER_VERSION}: using default squashfs settings (APPTAINER_BUILD_COMPRESS=${_COMPRESS_SPEC})." 1>&2
  fi
else
  echo "[info] Apptainer ${_APPTAINER_VERSION}: --mksquashfs-args unsupported (requires >= 1.4.0); using defaults (APPTAINER_BUILD_COMPRESS ignored)." 1>&2
fi

# ====Two-phase conversion: docker-archive -> sandbox -> SIF======================================
# DO NOT collapse this into a single 'apptainer build <sif> docker-archive:<tar>' call.
# Root cause (reproduced on Alliance Canada/Narval with apptainer 1.4.5, mksquashfs 4.6.1):
#   the FUSED docker-archive -> SIF build silently DROPS content — the baked-in super-project
#   '.git' (~11k files) was missing from the SIF while 'apptainer build' still exited 0. The very
#   same tar, converted in two phases on the very same node, is complete:
#     - 'apptainer build --sandbox' extracted all 11168 '.git' entries;
#     - packing that sandbox into a SIF (with default AND with '-noD -noF -noI -noX' args) kept
#       all 11168 entries and 'git rev-parse HEAD' resolved inside the SIF.
#   Disk (749 GB free) and memory were never a factor. So: extraction is fine, packing is fine,
#   only the fused path loses data => build the sandbox first, validate it, then pack it.
# Bonus: the sandbox lets us validate the '.git' WHERE THE DATA STILL IS, before packing, which
# makes a failure actionable instead of mysterious.
_SANDBOX_DIR="${SIF_STAGING_DIR}/${SIF_FILENAME%.sif}.sandbox"
rm -rf "${_SANDBOX_DIR}"

echo "[info] Phase 1/2: extracting the docker archive into a sandbox: ${_SANDBOX_DIR}" 1>&2
if ! apptainer build --sandbox "${_SANDBOX_DIR}" "docker-archive:${TAR_FILE}"; then
  echo "[error] Apptainer sandbox extraction failed. The tar archive has been preserved: ${TAR_FILE}" 1>&2
  rm -rf "${_SANDBOX_DIR}"
  exit 1
fi

# Validate the baked-in super-project '.git' in the SANDBOX (plain filesystem checks, no container
# runtime needed). Catching a bad extraction here is far cheaper than packing a 13+ GB SIF first.
if [[ -n "${SUPER_PROJECT_GIT_DIRNAME}" ]]; then
  _SBX_GIT="${_SANDBOX_DIR}/ros2_ws/src/${SUPER_PROJECT_GIT_DIRNAME}/.git"
  if [[ ! -d "${_SBX_GIT}/objects" ]] || [[ ! -d "${_SBX_GIT}/refs" ]] \
      || ! GIT_DIR="${_SBX_GIT}" git rev-parse --verify HEAD &>/dev/null; then
    echo "[error] Sandbox validation FAILED: the super-project '.git' is missing or incomplete at" 1>&2
    echo "[error]   ${_SBX_GIT}" 1>&2
    echo "[error]   The extraction of '${TAR_FILE}' did not produce a usable rootfs; no SIF was built." 1>&2
    echo "[hint]  Check free space on \${APPTAINER_TMPDIR} (${APPTAINER_TMPDIR}) and verify the tar" 1>&2
    echo "[hint]   integrity (compare its sha256 with the machine that produced it)." 1>&2
    rm -rf "${_SANDBOX_DIR}"
    exit 1
  fi
  echo "[done] Sandbox validation passed: super-project '.git' extracted completely." 1>&2
fi

echo "[info] Phase 2/2: packing the sandbox into a SIF: ${SIF_TMP}" 1>&2
if ! apptainer build "${_APPTAINER_BUILD_ARGS[@]}" \
    "${SIF_TMP}" \
    "${_SANDBOX_DIR}"; then
  echo "[error] Apptainer build failed. The tar archive has been preserved: ${TAR_FILE}" 1>&2
  rm -rf "${_SANDBOX_DIR}"
  exit 1
fi
# NOTE: the sandbox is deliberately KEPT until the guards below pass, so a failure can be
# investigated (and the SIF repacked) without re-extracting the whole tar.

# ====Content guard: verify the baked-in super-project '.git' survived the conversion=============
# DNA bakes the super-project '.git' into the image so the container stays portable and the DN/N2ST
# bootstrap can resolve PROJECT_PATH/N2ST_PATH via 'git rev-parse'. A partial/truncated conversion
# (e.g. the compute node ran out of space on ${APPTAINER_TMPDIR}/$SLURM_TMPDIR while unpacking a
# large image, or an OOM/SIGKILL on a capped login node) can drop the big super-project '.git' while
# smaller sibling repos under /ros2_ws/src/ survive. Checking "any .git/HEAD exists" is therefore
# NOT enough (it matches a sibling repo and false-passes). We validate the SPECIFIC super-project
# repo as a COMPLETE git repository (HEAD + objects + refs resolvable) and treat it as the hard
# requirement; incomplete sibling repos are only WARNED about (they may be legitimately shallow).
_dna_validate_sif_baked_git() {
  local _sif="$1"
  local _out _rc
  # The check runs INSIDE the container, so 'apptainer exec' itself can fail for reasons that have
  # nothing to do with the image content (nested exec under srun, no loop device, an unreadable or
  # truncated SIF). Capture everything and tell those cases apart instead of blaming the '.git'.
  # IMPORTANT: '--no-mount cwd'. Apptainer auto-binds the current working directory into the container.
  # This converter is typically run from the host super-project root, which on an HPC server does NOT
  # carry '.git' ('.git' is baked into the image). Without this flag, that host dir is mounted over
  # /ros2_ws/src/<project>, MASKING the baked-in '.git' and making a perfectly good SIF fail the guard.
  _out="$( apptainer exec --no-mount cwd "${_sif}" /bin/sh -c '
    expected="'"${SUPER_PROJECT_GIT_DIRNAME}"'"
    rc=0
    echo "DNA_GUARD_RAN"
    if command -v git >/dev/null 2>&1; then has_git=1; else has_git=0; fi
    check_repo() {
      gd="$1"; label="$2"
      if [ ! -d "${gd}" ]; then
        echo "MISSING ${label} repo in SIF: ${gd} (directory absent)"; return 1
      fi
      if [ ! -d "${gd}/objects" ] || [ ! -d "${gd}/refs" ] || [ ! -e "${gd}/HEAD" ]; then
        echo "INCOMPLETE ${label} repo in SIF: ${gd} (objects/refs/HEAD missing)"; return 1
      fi
      if [ "${has_git}" = "1" ]; then
        # safe.directory: inside a SIF the files are owned by root while the runtime uid is the
        # user, so git can refuse a perfectly complete repo with "dubious ownership".
        if ! git -c safe.directory="*" --git-dir="${gd}" rev-parse --verify HEAD >/dev/null 2>&1; then
          echo "UNRESOLVED ${label} repo in SIF: ${gd} (git rev-parse HEAD failed)"; return 1
        fi
      else
        echo "NOGIT no git binary in the image; ${label} repo ${gd} checked structurally only"
      fi
      return 0
    }
    # 1. The super-project repo MUST be present and valid.
    if [ -n "${expected}" ]; then
      check_repo "/ros2_ws/src/${expected}/.git" "super-project" || rc=1
    fi
    # 2. Sibling repos are checked too (partial-drop signal) but only WARNED about, not fatal:
    #    they can legitimately be shallow, a gitdir-file, or have an unborn HEAD.
    for gd in /ros2_ws/src/*/.git; do
      [ -e "${gd}" ] || continue
      case "${gd}" in "/ros2_ws/src/${expected}/.git") continue ;; esac
      check_repo "${gd}" "sibling" >/dev/null 2>&1 || echo "SIBLING_WARN incomplete baked repo (non-fatal): ${gd}"
    done
    exit ${rc}
  ' 2>&1 )"
  _rc=$?
  if ! printf '%s' "${_out}" | grep -q 'DNA_GUARD_RAN'; then
    echo "[error]   Could NOT run the content guard inside the SIF (apptainer exec rc=${_rc})." 1>&2
    echo "[error]   This is a container RUNTIME failure, NOT proof that the '.git' is missing." 1>&2
    printf '%s\n' "${_out}" | sed 's/^/[error]     /' 1>&2
    return 1
  fi
  printf '%s\n' "${_out}" | while IFS= read -r _line; do
    case "${_line}" in
      DNA_GUARD_RAN|'') ;;
      SIBLING_WARN*|NOGIT*)              echo "[warn]    ${_line}" 1>&2 ;;
      MISSING*|INCOMPLETE*|UNRESOLVED*)  echo "[error]   ${_line}" 1>&2 ;;
      *)                                 echo "[info]    ${_line}" 1>&2 ;;
    esac
  done
  return ${_rc}
}

if ! _dna_validate_sif_baked_git "${SIF_TMP}"; then
  echo "[error] Content guard FAILED: the built SIF has a missing/incomplete baked-in '.git'." 1>&2
  echo "[error]   The super-project '.git' (used by the DN/N2ST bootstrap) did not survive conversion." 1>&2
  echo "[error]   This is typically a TRUNCATED extraction: the node ran out of space on" 1>&2
  echo "[error]   \${APPTAINER_TMPDIR} (\$SLURM_TMPDIR/localscratch) while unpacking a large image, or an" 1>&2
  echo "[error]   OOM/SIGKILL on a resource-capped login node. The SIF is INVALID and was NOT installed." 1>&2
  echo "[hint]  Retry inside a compute allocation (default) with more local disk, and/or shrink the SIF" 1>&2
  echo "[hint]   footprint with APPTAINER_BUILD_COMPRESS=zstd:3. As a fallback, build the SIF on a host" 1>&2
  echo "[hint]   with ample disk (e.g. another HPC login node) and copy the .sif to \${SCRATCH}/sif/." 1>&2
  echo "[hint]  The extracted sandbox is KEPT at ${_SANDBOX_DIR} (it validated OK): inspect it, or" 1>&2
  echo "[hint]   repack it manually with: apptainer build <out.sif> ${_SANDBOX_DIR}" 1>&2
  rm -f "${SIF_TMP}"
  exit 1
fi
echo "[done] Content guard passed: super-project '.git' is present and valid in the SIF." 1>&2

echo "[info] Moving SIF from staging to final destination..." 1>&2
mv "${SIF_TMP}" "${SIF_FILE}"

# Re-validate AFTER the move: a cross-filesystem mv (localscratch -> scratch) can itself truncate on
# a full/over-quota destination, and the staging guard above only checked the pre-move copy.
if ! _dna_validate_sif_baked_git "${SIF_FILE}"; then
  echo "[error] Post-move validation FAILED: the installed SIF has a missing/incomplete baked-in '.git'." 1>&2
  echo "[error]   The move to ${SIF_FILE} likely truncated the file (destination full/over-quota)." 1>&2
  echo "[hint]  The extracted sandbox is KEPT at ${_SANDBOX_DIR}: repack it once space is freed." 1>&2
  rm -f "${SIF_FILE}"
  exit 1
fi

# Only now is the sandbox expendable: every failure path above keeps it so a retry never re-extracts.
rm -rf "${_SANDBOX_DIR}"
echo "[done] SIF file created and validated: ${SIF_FILE}" 1>&2

# ====Source tar archive is intentionally KEPT========================================================
# The source Docker tar archive is NOT deleted automatically: it is a costly artifact to rebuild and
# transfer, and keeping it lets you re-run this conversion (e.g. after tuning APPTAINER_BUILD_COMPRESS
# or on a node with more disk) without re-doing 'dna build --save' + rsync. Remove it manually when
# you no longer need it:  rm -f "${TAR_FILE}"
echo "[info] Source tar archive kept (not deleted): ${TAR_FILE}" 1>&2
echo "[info]   Delete it manually to reclaim space once you no longer need it." 1>&2
SCRIPT_EOF

  # Note: the generated script is kept readable/writable/executable by everyone on purpose. It is
  # regenerated/overwritten by 'dna' on the next build (handled via the leading 'rm -f'), and a
  # restrictive mode was a frequent source of rsync/scp failures when copying the artifact to an
  # HPC server where the user name/uid differs from the one on the development machine.
  chmod 0777 "${script_path}"
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
# The generated script builds the SIF into ${SCRATCH}/sif/ on the HPC server and is turn-key: it
# runs STAGE 1 (registry fetch -> sandbox) where it is started — the login node, the only place
# with outbound internet — then re-execs STAGE 2 (the network-free, memory-heavy SIF packing)
# inside a salloc compute allocation. Use --login-node-only (or APPTAINER_BUILD_NO_SALLOC=1) to
# keep both stages in place.
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

  # Remove any previous (possibly read-only) generated script so we can overwrite it cleanly.
  rm -f "${script_path}"

  cat > "${script_path}" << 'SCRIPT_EOF'
#!/bin/bash
set -e

# ====Documentation================================================================================
DOCUMENTATION_BUFFER=$( cat <<'EOF'
# =================================================================================================
# Auto-generated by DNA: dna build slurm --apptainer <profile> --push
# Run this script on the HPC server to build an Apptainer SIF from a Docker registry image.
#
# This is the SINGLE SOURCE OF TRUTH for registry -> SIF conversion on HPC servers: it handles the
# login-node vs compute-node split by itself and runs end-to-end.
#
# STAGE 1 (login node): fetch + extract the registry image into a SANDBOX directory. The registry
#   fetch needs outbound internet, which login nodes have and compute nodes usually do NOT
#   (e.g. Alliance Canada). The OCI blob cache is PERSISTENT so a --force rebuild does not
#   re-download the multi-GB layers.
# STAGE 2 (compute node via salloc, when available): pack the sandbox into a SIF, validate the
#   baked-in super-project '.git', then move it to ${SCRATCH}/sif/. Packing is memory/IO heavy and
#   needs NO network, so it belongs on a compute node with real RAM and fast local disk.
#
# Both stages fall back gracefully: without salloc, stage 2 runs in place (with a warning).
#
# The conversion is deliberately TWO-PHASE (sandbox then pack), never fused: a fused
# `apptainer build <sif> docker://...` was observed to silently DROP content (the baked-in
# super-project '.git') while still exiting 0.
#
# Prerequisite: run dna_hpc_server_config.bash once to set up the directory structure
# and authenticate with the Docker registry.
#
# Usage (from the super-project root directory on the HPC server):
#   $ bash artifact/apptainer/<target>/dna_registry_to_apptainer_sif_converter.sh [OPTIONS]
#
# Options:
#   --force
#       Rebuild even when the destination SIF already exists (default: skip and exit 0 when a
#       VALID SIF is already in place). Layers are re-used from the persistent blob cache.
#   --image <REPO>
#       Override the Docker repository (without tag) baked in by 'dna'.
#   --tag <TAG>
#       Override the Docker image tag baked in by 'dna'.
#   --docker-login
#       Optional. Authenticate interactively with the Docker registry (docker.io) before
#       fetching. The prompt happens during STAGE 1 on the login node, so it always works.
#       For a non-interactive run, export APPTAINER_DOCKER_USERNAME / APPTAINER_DOCKER_PASSWORD
#       instead (the password should be a Docker Hub *access token*).
#   --login-node-only
#       Run BOTH stages in place on the current node (no salloc). Use it on clusters without a
#       job scheduler, or when the login node is unconstrained (e.g. Valeria).
#   -h | --help
#       Print this help message and exit.
#
# Environment variables (build tuning, all optional):
#   APPTAINER_DOCKER_USERNAME / APPTAINER_DOCKER_PASSWORD
#       Non-interactive registry credentials (token recommended) used by the STAGE 1 fetch.
#   APPTAINER_PRESTAGE_CACHEDIR=<dir>
#       Persistent OCI blob cache (default ${SCRATCH}/.apptainer_cache) so a --force rebuild does
#       not re-download the layers. Set APPTAINER_BUILD_NO_PERSISTENT_CACHE=1 for a throw-away cache.
#   APPTAINER_BUILD_NO_SALLOC=1   Never re-exec into a compute allocation (same as --login-node-only).
#   APPTAINER_BUILD_NO_HTTPPROXY=1  Do NOT load the `httpproxy` module on compute nodes.
#   APPTAINER_BUILD_ACCOUNT=<acct>  SLURM account for the allocation (auto-detected otherwise).
#   APPTAINER_BUILD_MEM=<mem>       salloc --mem     (default 64G).
#   APPTAINER_BUILD_CPUS=<n>        salloc --cpus-per-task and mksquashfs -processors (default 10).
#   APPTAINER_BUILD_TIME=<hh:mm:ss> salloc --time    (default 6:00:00).
#   APPTAINER_BUILD_HEARTBEAT_SEC=<n>  Progress heartbeat period in seconds (default 60, 0 disables).
#   APPTAINER_BUILD_STALL_SEC=<n>   Warn when the staging area stops growing for that long
#       (default 900). Purely informational; it never kills the build.
#   APPTAINER_BUILD_COMPRESS=<comp[:level]>
#       squashfs compressor for apptainer >= 1.4.0 (default 'default' = apptainer's own choice).
#       Alternatives: zstd:3, lz4, gzip:6, or none (uncompressed, largest). Ignored on < 1.4.0.
#
# Requires:
#   - apptainer installed on the HPC server
#   - Network access to docker.io from the HPC server (on isolated compute nodes the `httpproxy`
#     module is loaded automatically to provide it; see APPTAINER_BUILD_NO_HTTPPROXY)
#   - $SCRATCH environment variable set (the SIF file is output to ${SCRATCH}/sif/)
#
# =================================================================================================
EOF
)

# ====Argument parsing=============================================================================
USE_DOCKER_LOGIN=false
FORCE_REBUILD=false
LOGIN_NODE_ONLY=false
OVERRIDE_IMAGE=""
OVERRIDE_TAG=""
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
    --force)
      FORCE_REBUILD=true
      shift
      ;;
    --login-node-only)
      LOGIN_NODE_ONLY=true
      shift
      ;;
    --image)
      OVERRIDE_IMAGE="${2:?[error] --image requires a value}"
      shift 2
      ;;
    --tag)
      OVERRIDE_TAG="${2:?[error] --tag requires a value}"
      shift 2
      ;;
    *)
      echo "[error] Unknown argument: $1" 1>&2
      echo "Usage: $0 [--force] [--image <REPO>] [--tag <TAG>] [--docker-login] [--login-node-only] [--help]" 1>&2
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
  # SUPER_PROJECT_GIT_DIRNAME is the super-project repository directory baked under /ros2_ws/src/
  # in the image; the content guard validates THAT specific '.git' (see _dna_validate_sif_baked_git).
  # Fail loudly at generation time if it is unknown: without it the guard can only check sibling
  # repos and would silently weaken to a non-specific check.
  if [[ -z "${SUPER_PROJECT_REPO_NAME:-}" ]]; then
    echo "[warn] SUPER_PROJECT_REPO_NAME is empty while generating $(basename "${script_path}"); the" 1>&2
    echo "[warn]   baked-in '.git' content guard cannot target the specific super-project repo and" 1>&2
    echo "[warn]   will only validate sibling repos. Run 'dna' from the super-project so it is set." 1>&2
  fi
  cat >> "${script_path}" << EOF
IMAGE_REF="${image_ref}"
SIF_FILENAME="${sif_name}"
SUPER_PROJECT_GIT_DIRNAME="${SUPER_PROJECT_REPO_NAME:-}"
EOF

  cat >> "${script_path}" << 'SCRIPT_EOF'

# ====Image reference resolution (--image / --tag overrides)======================================
# IMAGE_REF is baked in by 'dna' as '<repo>:<tag>'. --image replaces the repo part, --tag the tag.
_BAKED_REPO="${IMAGE_REF%:*}"
_BAKED_TAG="${IMAGE_REF##*:}"
if [[ "${IMAGE_REF}" != *:* ]]; then
  _BAKED_REPO="${IMAGE_REF}"
  _BAKED_TAG="latest"
fi
IMAGE_REF="${OVERRIDE_IMAGE:-${_BAKED_REPO}}:${OVERRIDE_TAG:-${_BAKED_TAG}}"

# Output the SIF to the scratch SIF cache (${SCRATCH}/sif/), matching the SIF path lookup in the
# generated SLURM job scripts.
SIF_DIR="${SCRATCH}/sif"
SIF_FILE="${SIF_DIR}/${SIF_FILENAME}"
mkdir -p "${SIF_DIR}"

echo "[info] Image reference: ${IMAGE_REF}" 1>&2
echo "[info] SIF output:      ${SIF_FILE}" 1>&2

# The sandbox produced by STAGE 1 lives on the SHARED filesystem (${SCRATCH}) so that the STAGE 2
# compute node — a different machine — can read it. Node-local disk is used for the SIF staging.
_SANDBOX_ROOT="${APPTAINER_BUILD_SANDBOX_ROOT:-${SCRATCH}/tmp}"
_SANDBOX_DIR="${_SANDBOX_ROOT}/${SIF_FILENAME%.sif}.sandbox"

# ====Progress heartbeat / stall detection========================================================
# A multi-GB fetch/extract/pack prints nothing for long stretches; without feedback it is impossible
# to tell "slow" from "hung". The heartbeat reports elapsed time and the growing size of the
# watched path, and WARNS (never kills) when it stops growing.
_HEARTBEAT_PID=""
_dna_heartbeat_start() {
  local _label="$1" _watch="$2"
  local _period="${APPTAINER_BUILD_HEARTBEAT_SEC:-60}"
  [[ "${_period}" -gt 0 ]] 2>/dev/null || return 0
  local _stall="${APPTAINER_BUILD_STALL_SEC:-900}"
  (
    _t0=${SECONDS} _last_size=-1 _last_change=${SECONDS}
    while true; do
      sleep "${_period}"
      _size="$( du -sk "${_watch}" 2>/dev/null | cut -f1 )"
      _size="${_size:-0}"
      echo "[info] ${_label}: elapsed $(( SECONDS - _t0 ))s, staged $(( _size / 1024 )) MiB" 1>&2
      if [[ "${_size}" != "${_last_size}" ]]; then
        _last_size="${_size}"
        _last_change=${SECONDS}
      elif [[ $(( SECONDS - _last_change )) -ge ${_stall} ]]; then
        echo "[warn] ${_label}: no progress for $(( SECONDS - _last_change ))s — the build may be stalled" 1>&2
        echo "[warn]   (slow registry, throttled shared filesystem, or a hung proxy). Not killing it." 1>&2
        _last_change=${SECONDS}
      fi
    done
  ) &
  _HEARTBEAT_PID=$!
}
_dna_heartbeat_stop() {
  if [[ -n "${_HEARTBEAT_PID}" ]]; then
    kill "${_HEARTBEAT_PID}" 2>/dev/null || true
    wait "${_HEARTBEAT_PID}" 2>/dev/null || true
    _HEARTBEAT_PID=""
  fi
}
trap '_dna_heartbeat_stop' EXIT

# ====Watchdog based on the remaining SLURM time==================================================
# Inside an allocation, a build that outlives the allocation is killed mid-write, leaving a
# truncated SIF. Cap it slightly BELOW the remaining wall time so we exit cleanly and can say why.
_dna_timeout_args() {
  command -v timeout &>/dev/null || return 0
  [[ -n "${SLURM_JOB_ID:-}" ]] || return 0
  command -v squeue &>/dev/null || return 0
  local _left
  _left="$( squeue -h -j "${SLURM_JOB_ID}" -o %L 2>/dev/null | tr -d ' ' )"
  [[ -n "${_left}" ]] || return 0
  local _sec=0 _d=0 _hms="${_left}"
  if [[ "${_hms}" == *-* ]]; then _d="${_hms%%-*}"; _hms="${_hms#*-}"; fi
  local IFS=':' _parts
  read -r -a _parts <<< "${_hms}"
  case "${#_parts[@]}" in
    3) _sec=$(( 10#${_parts[0]} * 3600 + 10#${_parts[1]} * 60 + 10#${_parts[2]} )) ;;
    2) _sec=$(( 10#${_parts[0]} * 60 + 10#${_parts[1]} )) ;;
    1) _sec=$(( 10#${_parts[0]} * 60 )) ;;
  esac
  _sec=$(( _sec + 10#${_d} * 86400 - 120 ))
  [[ "${_sec}" -gt 60 ]] || return 0
  echo "timeout --signal=TERM ${_sec}"
}

# ====Stage selection==============================================================================
# STAGE 1 'fetch' needs the internet (login node); STAGE 2 'pack' needs RAM + fast local disk
# (compute node) and NO network. The inner salloc run is identified by DNA_SIF_STAGE2_SANDBOX.
if [[ -n "${DNA_SIF_STAGE2_SANDBOX:-}" ]]; then
  _DNA_STAGE="pack"
  _SANDBOX_DIR="${DNA_SIF_STAGE2_SANDBOX}"
else
  _DNA_STAGE="fetch+pack"
fi

# ====Skip when a valid SIF is already in place (unless --force)==================================
if [[ "${_DNA_STAGE}" != "pack" && -f "${SIF_FILE}" && "${FORCE_REBUILD}" != true ]]; then
  echo "[info] SIF already present: ${SIF_FILE}" 1>&2
  echo "[info]   Nothing to do. Re-run with --force to rebuild it (cached layers are re-used)." 1>&2
  exit 0
fi

# ====Apptainer cache configuration================================================================
# Prefer SLURM_TMPDIR (fast node-local disk) when inside a job, otherwise fall back to
# ${SCRATCH}/tmp (a large disk-backed filesystem, NOT the small RAM-backed /tmp which would
# OOM-Kill the build). Ref: https://docs.alliancecan.ca/wiki/Apptainer
_APPTAINER_SCRATCH_TMP_ROOT="${SLURM_TMPDIR:-${SCRATCH}/tmp}"
mkdir -p "${_APPTAINER_SCRATCH_TMP_ROOT}"
export APPTAINER_TMPDIR="$( mktemp -d -p "${_APPTAINER_SCRATCH_TMP_ROOT}" )"

# The OCI blob cache is PERSISTENT by default so a --force rebuild (or a STAGE 2 retry) does not
# re-download the multi-GB layers. It must live on a large shared filesystem, never in $HOME.
if [[ "${APPTAINER_BUILD_NO_PERSISTENT_CACHE:-0}" == "1" ]]; then
  export APPTAINER_CACHEDIR="$( mktemp -d -p "${_APPTAINER_SCRATCH_TMP_ROOT}" )"
  echo "[info] Using a throw-away OCI blob cache (APPTAINER_BUILD_NO_PERSISTENT_CACHE=1)." 1>&2
else
  export APPTAINER_CACHEDIR="${APPTAINER_PRESTAGE_CACHEDIR:-${SCRATCH}/.apptainer_cache}"
  mkdir -p "${APPTAINER_CACHEDIR}"
  echo "[info] Persistent OCI blob cache: ${APPTAINER_CACHEDIR}" 1>&2
fi

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
  # Compute nodes on some HPC clusters (e.g. Alliance Canada / compute_canada) have no direct
  # outbound internet access. The `httpproxy` module exports proxy env vars that give them outbound
  # HTTP(S). It MUST NOT be loaded on a login node (which has direct internet): routing docker.io
  # through the compute-node proxy makes the pull fail with 'index.docker.io: Forbidden'.
  # STAGE 2 (pack) needs NO network at all, so httpproxy is only relevant when the FETCH itself is
  # running inside an allocation (i.e. the whole script was started from within a SLURM job).
  # Load it best-effort. Disable with APPTAINER_BUILD_NO_HTTPPROXY=1.
  if [[ "${_DNA_STAGE}" != "pack" ]] && [[ -n "${SLURM_JOB_ID:-}" ]] \
      && [[ "${APPTAINER_BUILD_NO_HTTPPROXY:-0}" != "1" ]] && module spider httpproxy &>/dev/null; then
    echo "[info] Fetching from inside a SLURM job — loading httpproxy module (outbound internet)." 1>&2
    module load httpproxy || echo "[warn] Failed to load httpproxy module; registry pull may time out on isolated compute nodes." 1>&2
  fi
fi

# ====Registry credentials=========================================================================
# Note: --disable-cache is NOT used here because APPTAINER_CACHEDIR is already redirected to
# scratch space above, so the cache never lands in the home directory.
_APPTAINER_EXTRA_FLAGS=()
if [[ -n "${APPTAINER_DOCKER_USERNAME:-}" && -n "${APPTAINER_DOCKER_PASSWORD:-}" ]]; then
  # Consumed directly by apptainer — nothing to do beyond reporting it (never echo the secret).
  echo "[info] Using non-interactive registry credentials for user '${APPTAINER_DOCKER_USERNAME}'." 1>&2
elif [[ "${USE_DOCKER_LOGIN}" == true ]]; then
  echo "[info] Authenticating with docker.io interactively (--docker-login)..." 1>&2
  _APPTAINER_EXTRA_FLAGS=(--docker-login)
else
  echo "[info] No credentials provided — assuming public registry access (or a prior 'apptainer registry login')." 1>&2
fi

# Stage the SIF on APPTAINER_TMPDIR (node-local when inside an allocation), then move it to the
# final destination on the shared filesystem.
SIF_STAGING_DIR="${APPTAINER_TMPDIR:-/tmp}"
SIF_TMP="${SIF_STAGING_DIR}/${SIF_FILENAME}"

# ====Squashfs compression tuning=================================================================
# apptainer >= 1.4.0 accepts --mksquashfs-args. Tune via APPTAINER_BUILD_COMPRESS="<comp>[:<level>]".
# Default is 'default': let Apptainer/mksquashfs pick its own compressor. Measured on a real DNA
# slurm image, the uncompressed variant ('none' -> -noD -noF -noI -noX) produced a 21.7 GB SIF vs
# 13.2 GB with the default compressor for byte-identical content — a much heavier read from the
# shared filesystem at every job start, for no benefit. Other values: none, zstd:3, lz4, gzip:6.
_APPTAINER_VERSION="$( apptainer --version 2>/dev/null | grep -oP '[0-9]+\.[0-9]+\.[0-9]+' | head -1 )"
_APPTAINER_MAJOR="$( echo "${_APPTAINER_VERSION}" | cut -d. -f1 )"
_APPTAINER_MINOR="$( echo "${_APPTAINER_VERSION}" | cut -d. -f2 )"
_APPTAINER_BUILD_ARGS=()
if [[ "${_APPTAINER_MAJOR}" -gt 1 ]] || { [[ "${_APPTAINER_MAJOR}" -eq 1 ]] && [[ "${_APPTAINER_MINOR}" -ge 4 ]]; }; then
  _COMPRESS_SPEC="${APPTAINER_BUILD_COMPRESS:-default}"
  _COMP_NAME="${_COMPRESS_SPEC%%:*}"
  _COMP_LEVEL="${_COMPRESS_SPEC#*:}"
  [[ "${_COMP_LEVEL}" == "${_COMPRESS_SPEC}" ]] && _COMP_LEVEL=""
  _MKSQUASHFS_ARGS=()
  if [[ "${_COMP_NAME}" == "default" ]]; then
    : # Let Apptainer/mksquashfs pick the compressor (smallest robust SIF, no tuning).
  elif [[ "${_COMP_NAME}" == "none" ]]; then
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
  if [[ ${#_MKSQUASHFS_ARGS[@]} -gt 0 ]]; then
    echo "[info] Apptainer ${_APPTAINER_VERSION}: squashfs args: ${_MKSQUASHFS_ARGS[*]}" 1>&2
    _APPTAINER_BUILD_ARGS+=(--mksquashfs-args="${_MKSQUASHFS_ARGS[*]}")
  else
    echo "[info] Apptainer ${_APPTAINER_VERSION}: using default squashfs settings (APPTAINER_BUILD_COMPRESS=${_COMPRESS_SPEC})." 1>&2
  fi
else
  echo "[info] Apptainer ${_APPTAINER_VERSION}: --mksquashfs-args unsupported (requires >= 1.4.0); using defaults (APPTAINER_BUILD_COMPRESS ignored)." 1>&2
fi

# ====Two-phase conversion: docker:// -> sandbox -> SIF===========================================
# DO NOT collapse this into a single 'apptainer build <sif> docker://...' call: the FUSED build was
# observed on Alliance Canada/Narval (apptainer 1.4.5) to silently DROP the baked-in super-project
# '.git' while exiting 0, whereas '--sandbox' extraction + a separate pack kept every file.
# The two phases also map exactly onto the node capabilities: extraction needs the NETWORK (login
# node), packing needs RAM and fast local disk (compute node).

# ----STAGE 1: fetch + extract into a sandbox (needs the internet)--------------------------------
if [[ "${_DNA_STAGE}" != "pack" ]]; then
  # Alliance Canada / Compute Canada cap per-user RAM (~4 GB) and CPU-time on login nodes and
  # SIGKILL heavy processes ("Killed"). Stage 1 must run there anyway (compute nodes cannot reach
  # Docker Hub's blob CDN), so warn up-front: for very large images the '--save' tar pipeline is
  # the reliable alternative. CC_CLUSTER is exported on all Alliance clusters.
  if [[ -z "${SLURM_JOB_ID:-}" && -n "${CC_CLUSTER:-}" ]]; then
    echo "[warn] Alliance Canada login node detected (CC_CLUSTER=${CC_CLUSTER}): RAM and CPU-time are" 1>&2
    echo "[warn]   capped here, so a large registry fetch can be SIGKILLed ('Killed'). Only the" 1>&2
    echo "[warn]   network-bound stage 1 runs here; if it gets killed, use the '--save' tar pipeline" 1>&2
    echo "[warn]   (dna_tar_to_apptainer_sif_converter.sh) or copy a prebuilt amd64 SIF instead." 1>&2
  fi
  mkdir -p "${_SANDBOX_ROOT}"
  rm -rf "${_SANDBOX_DIR}"

  echo "[info] Stage 1/2: fetching and extracting the registry image into a sandbox." 1>&2
  echo "[info]   Source:  docker://${IMAGE_REF}" 1>&2
  echo "[info]   Sandbox: ${_SANDBOX_DIR}" 1>&2
  _dna_heartbeat_start "Stage 1/2 (fetch+extract)" "${_SANDBOX_DIR}"
  # shellcheck disable=SC2046
  if ! $( _dna_timeout_args ) apptainer build --sandbox \
      "${_APPTAINER_EXTRA_FLAGS[@]}" \
      "${_SANDBOX_DIR}" \
      "docker://${IMAGE_REF}"; then
    _dna_heartbeat_stop
    rm -rf "${_SANDBOX_DIR}"
    echo "[error] Stage 1/2 FAILED: could not fetch/extract image: ${IMAGE_REF}" 1>&2
    echo "[hint]  Check that the image is accessible from this node." 1>&2
    echo "[hint]  If the registry requires authentication, re-run with --docker-login or export" 1>&2
    echo "[hint]    APPTAINER_DOCKER_USERNAME / APPTAINER_DOCKER_PASSWORD (access token)." 1>&2
    echo "[hint]  A 'Killed' means the process was SIGKILLed by the login-node resource limits" 1>&2
    echo "[hint]    (Alliance Canada caps RAM/CPU-time). Use the '--save' tar pipeline instead." 1>&2
    echo "[hint]  A 'Forbidden' from index.docker.io usually means the httpproxy module is loaded on" 1>&2
    echo "[hint]    a login node; run 'module unload httpproxy' and retry." 1>&2
    echo "[hint]  A 'dial tcp ... i/o timeout' means this node has no outbound internet: run stage 1" 1>&2
    echo "[hint]    on the LOGIN node (the default)." 1>&2
    exit 1
  fi
  _dna_heartbeat_stop

  # Validate the baked-in super-project '.git' in the SANDBOX (plain filesystem checks, no container
  # runtime needed) so a bad extraction is caught before packing a multi-GB SIF.
  if [[ -n "${SUPER_PROJECT_GIT_DIRNAME}" ]]; then
    _SBX_GIT="${_SANDBOX_DIR}/ros2_ws/src/${SUPER_PROJECT_GIT_DIRNAME}/.git"
    if [[ ! -d "${_SBX_GIT}/objects" ]] || [[ ! -d "${_SBX_GIT}/refs" ]] \
        || ! GIT_DIR="${_SBX_GIT}" git rev-parse --verify HEAD &>/dev/null; then
      echo "[error] Sandbox validation FAILED: the super-project '.git' is missing or incomplete at" 1>&2
      echo "[error]   ${_SBX_GIT}" 1>&2
      echo "[error]   The extraction of '${IMAGE_REF}' did not produce a usable rootfs; no SIF was built." 1>&2
      rm -rf "${_SANDBOX_DIR}"
      exit 1
    fi
    echo "[done] Sandbox validation passed: super-project '.git' extracted completely." 1>&2
  fi

  # ----Hand STAGE 2 over to a compute node------------------------------------------------------
  # Packing is memory/IO heavy and needs NO network, so it belongs in an allocation. The sandbox
  # lives on the shared filesystem, so the compute node reads exactly what we just extracted.
  if [[ -z "${SLURM_JOB_ID:-}" && "${LOGIN_NODE_ONLY}" != true \
        && "${APPTAINER_BUILD_NO_SALLOC:-0}" != "1" ]] && command -v salloc &>/dev/null; then
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
    echo "[info] Stage 2/2 runs inside a compute allocation (RAM + node-local disk, no network needed)." 1>&2
    echo "[info]   salloc ${_SALLOC_ARGS[*]} srun bash ${_SELF}" 1>&2
    export DNA_SIF_STAGE2_SANDBOX="${_SANDBOX_DIR}"
    exec salloc "${_SALLOC_ARGS[@]}" srun bash "${_SELF}" "$@"
  fi
  if [[ -n "${SLURM_JOB_ID:-}" ]]; then
    echo "[info] Already inside SLURM job ${SLURM_JOB_ID} — running stage 2 here." 1>&2
  elif [[ "${LOGIN_NODE_ONLY}" == true || "${APPTAINER_BUILD_NO_SALLOC:-0}" == "1" ]]; then
    echo "[info] Running stage 2 in place (--login-node-only / APPTAINER_BUILD_NO_SALLOC=1)." 1>&2
  else
    echo "[warn] 'salloc' is unavailable — running the memory-heavy packing in place. On a" 1>&2
    echo "[warn]   resource-capped login node this can be SIGKILLed; the guards below will catch it." 1>&2
  fi
fi

# ----STAGE 2: pack the sandbox into a SIF (no network needed)------------------------------------
if [[ ! -d "${_SANDBOX_DIR}" ]]; then
  echo "[error] Stage 2/2 cannot start: the sandbox is missing: ${_SANDBOX_DIR}" 1>&2
  echo "[hint]  Re-run the script (stage 1 re-uses the persistent blob cache, so no re-download)." 1>&2
  exit 1
fi
echo "[info] Stage 2/2: packing the sandbox into a SIF." 1>&2
echo "[info]   Sandbox: ${_SANDBOX_DIR}" 1>&2
echo "[info]   Staging: ${SIF_TMP}" 1>&2
echo "[info]   Output:  ${SIF_FILE}" 1>&2
_dna_heartbeat_start "Stage 2/2 (pack)" "${SIF_TMP}"
# shellcheck disable=SC2046
if ! $( _dna_timeout_args ) apptainer build "${_APPTAINER_BUILD_ARGS[@]}" \
    "${SIF_TMP}" \
    "${_SANDBOX_DIR}"; then
  _dna_heartbeat_stop
  echo "[error] Stage 2/2 FAILED: could not pack the sandbox into a SIF (image: ${IMAGE_REF})." 1>&2
  echo "[hint]  The sandbox is KEPT at ${_SANDBOX_DIR} so a retry skips the download entirely." 1>&2
  rm -f "${SIF_TMP}"
  exit 1
fi
_dna_heartbeat_stop

# ====Content guard: verify the baked-in super-project '.git' survived the conversion=============
# DNA bakes the super-project '.git' into the image so the container stays portable and the DN/N2ST
# bootstrap can resolve PROJECT_PATH/N2ST_PATH via 'git rev-parse'. A partial/truncated conversion
# can drop the big super-project '.git' while smaller sibling repos under /ros2_ws/src/ survive, so
# checking "any .git/HEAD exists" is NOT enough (it matches a sibling repo and false-passes). We
# validate the SPECIFIC super-project repo as a COMPLETE git repository (HEAD + objects + refs
# resolvable) as the hard requirement; incomplete sibling repos are only WARNED about.
_dna_validate_sif_baked_git() {
  local _sif="$1"
  local _out _rc
  # The check runs INSIDE the container, so 'apptainer exec' itself can fail for reasons that have
  # nothing to do with the image content (nested exec under srun, no loop device, an unreadable or
  # truncated SIF). Capture everything and tell those cases apart instead of blaming the '.git'.
  # IMPORTANT: '--no-mount cwd'. Apptainer auto-binds the current working directory into the container.
  # This converter is typically run from the host super-project root, which on an HPC server does NOT
  # carry '.git' ('.git' is baked into the image). Without this flag, that host dir is mounted over
  # /ros2_ws/src/<project>, MASKING the baked-in '.git' and making a perfectly good SIF fail the guard.
  _out="$( apptainer exec --no-mount cwd "${_sif}" /bin/sh -c '
    expected="'"${SUPER_PROJECT_GIT_DIRNAME}"'"
    rc=0
    echo "DNA_GUARD_RAN"
    if command -v git >/dev/null 2>&1; then has_git=1; else has_git=0; fi
    check_repo() {
      gd="$1"; label="$2"
      if [ ! -d "${gd}" ]; then
        echo "MISSING ${label} repo in SIF: ${gd} (directory absent)"; return 1
      fi
      if [ ! -d "${gd}/objects" ] || [ ! -d "${gd}/refs" ] || [ ! -e "${gd}/HEAD" ]; then
        echo "INCOMPLETE ${label} repo in SIF: ${gd} (objects/refs/HEAD missing)"; return 1
      fi
      if [ "${has_git}" = "1" ]; then
        # safe.directory: inside a SIF the files are owned by root while the runtime uid is the
        # user, so git can refuse a perfectly complete repo with "dubious ownership".
        if ! git -c safe.directory="*" --git-dir="${gd}" rev-parse --verify HEAD >/dev/null 2>&1; then
          echo "UNRESOLVED ${label} repo in SIF: ${gd} (git rev-parse HEAD failed)"; return 1
        fi
      else
        echo "NOGIT no git binary in the image; ${label} repo ${gd} checked structurally only"
      fi
      return 0
    }
    # 1. The super-project repo MUST be present and valid.
    if [ -n "${expected}" ]; then
      check_repo "/ros2_ws/src/${expected}/.git" "super-project" || rc=1
    fi
    # 2. Sibling repos are checked too (partial-drop signal) but only WARNED about, not fatal:
    #    they can legitimately be shallow, a gitdir-file, or have an unborn HEAD.
    for gd in /ros2_ws/src/*/.git; do
      [ -e "${gd}" ] || continue
      case "${gd}" in "/ros2_ws/src/${expected}/.git") continue ;; esac
      check_repo "${gd}" "sibling" >/dev/null 2>&1 || echo "SIBLING_WARN incomplete baked repo (non-fatal): ${gd}"
    done
    exit ${rc}
  ' 2>&1 )"
  _rc=$?
  if ! printf '%s' "${_out}" | grep -q 'DNA_GUARD_RAN'; then
    echo "[error]   Could NOT run the content guard inside the SIF (apptainer exec rc=${_rc})." 1>&2
    echo "[error]   This is a container RUNTIME failure, NOT proof that the '.git' is missing." 1>&2
    printf '%s\n' "${_out}" | sed 's/^/[error]     /' 1>&2
    return 1
  fi
  printf '%s\n' "${_out}" | while IFS= read -r _line; do
    case "${_line}" in
      DNA_GUARD_RAN|'') ;;
      SIBLING_WARN*|NOGIT*)              echo "[warn]    ${_line}" 1>&2 ;;
      MISSING*|INCOMPLETE*|UNRESOLVED*)  echo "[error]   ${_line}" 1>&2 ;;
      *)                                 echo "[info]    ${_line}" 1>&2 ;;
    esac
  done
  return ${_rc}
}

if ! _dna_validate_sif_baked_git "${SIF_TMP}"; then
  echo "[error] Content guard FAILED: the built SIF has a missing/incomplete baked-in '.git'." 1>&2
  echo "[error]   The super-project '.git' (used by the DN/N2ST bootstrap) did not survive conversion." 1>&2
  echo "[hint]  The sandbox is KEPT at ${_SANDBOX_DIR}: inspect it, then re-run to repack." 1>&2
  rm -f "${SIF_TMP}"
  exit 1
fi
echo "[done] Content guard passed: super-project '.git' is present and valid in the SIF." 1>&2

echo "[info] Moving SIF from staging to final destination..." 1>&2
mv "${SIF_TMP}" "${SIF_FILE}"

# Re-validate AFTER the move: a cross-filesystem mv can itself truncate on a full/over-quota
# destination, and the staging guard above only checked the pre-move copy.
if ! _dna_validate_sif_baked_git "${SIF_FILE}"; then
  echo "[error] Post-move validation FAILED: the installed SIF has a missing/incomplete baked-in '.git'." 1>&2
  echo "[error]   The move to ${SIF_FILE} likely truncated the file (destination full/over-quota)." 1>&2
  rm -f "${SIF_FILE}"
  exit 1
fi

# Only now is the sandbox expendable: every failure path above keeps it so a retry never re-downloads.
rm -rf "${_SANDBOX_DIR}"
echo "[done] SIF file created and validated: ${SIF_FILE}" 1>&2
SCRIPT_EOF

  # Note: the generated script is kept readable/writable/executable by everyone on purpose. It is
  # regenerated/overwritten by 'dna' on the next build (handled via the leading 'rm -f'), and a
  # restrictive mode was a frequent source of rsync/scp failures when copying the artifact to an
  # HPC server where the user name/uid differs from the one on the development machine.
  chmod 0777 "${script_path}"
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

  # Remove any previous (possibly read-only) generated script so we can overwrite it cleanly.
  rm -f "${script_path}"

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

# ====Generate artifact/apptainer/README.md=======================================================
# Document the auto-generated apptainer artifact directory so users don't mistakenly edit the
# generated converter/config scripts, and have a quick usage guide at hand.
_APPTAINER_ARTIFACT_DIR="${SUPER_PROJECT_ROOT}/artifact/apptainer"
mkdir -p "${_APPTAINER_ARTIFACT_DIR}"
echo "[info] Writing ${_APPTAINER_ARTIFACT_DIR}/README.md" 1>&2
cat > "${_APPTAINER_ARTIFACT_DIR}/README.md" <<'README_EOF'
# artifact/apptainer/

> ⚠️ **AUTO-GENERATED DIRECTORY** — the scripts under `artifact/apptainer/<target>/` are generated
> by `dna` commands (`dna build slurm --apptainer <target> --save|--push`). **Do not edit them by
> hand**: they are silently overwritten on the next `dna` build. Put any
> custom logic elsewhere.

## Contents

Each `<target>/` subdirectory (e.g. `valeria/`, `compute_canada/`, `mamba/`) contains:

- `dna_hpc_server_config.bash` — Run **once** on the HPC server to create the super-project
  directory structure, load the Apptainer module, and authenticate with the Docker registry. It
  also (re)writes this README.
- `dna_tar_to_apptainer_sif_converter.sh` — Converts a transferred Docker **tar archive** (produced
  by `dna build ... --save`) into a `.sif`. Builds inside a compute allocation (salloc) by default,
  so it works on memory-capped login nodes. **Recommended on Alliance Canada / Compute Canada.**
- `dna_registry_to_apptainer_sif_converter.sh` — Builds a `.sif` directly from a Docker **registry**
  image (produced by `dna build ... --push`). Requires outbound internet on the build node; it is
  unreliable on Alliance Canada clusters (login-node kill + no compute-node blob-CDN access).

The built `.sif` is written to `${SCRATCH}/sif/`.

## Quick usage (on the HPC server, from the super-project root)

```bash
# 1. One-time setup + registry login:
bash artifact/apptainer/<target>/dna_hpc_server_config.bash

# 2a. Tar pipeline (recommended on Alliance Canada) — after rsync-ing the .tar next to the script:
bash artifact/apptainer/<target>/dna_tar_to_apptainer_sif_converter.sh

# 2b. OR registry pipeline (needs internet on the build node):
bash artifact/apptainer/<target>/dna_registry_to_apptainer_sif_converter.sh

# 3. Submit the job:
sbatch slurm_jobs/<your-job>/slurm_job.<name>.apptainer.<target>.bash
```

Both converters run a post-build **content guard** that fails loudly if the baked-in super-project
`.git` directory did not survive the SIF conversion (a symptom of a truncated / OOM-killed build on
a resource-capped login node). Pass `--help` to any script for the full option list.
README_EOF
echo "[done] artifact/apptainer/README.md written." 1>&2

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

  # Note: the generated script is kept readable/writable/executable by everyone on purpose. It is
  # regenerated/overwritten by 'dna' on the next build (handled via the leading 'rm -f'), and a
  # restrictive mode was a frequent source of rsync/scp failures when copying the artifact to an
  # HPC server where the user name/uid differs from the one on the development machine.
  chmod 0777 "${script_path}"
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
  # Prevent the current-working-directory auto-mount. Apptainer auto-binds $PWD into the container;
  # when the job is launched from the host super-project root, that host dir (which does NOT carry
  # '.git' on the HPC server — '.git' is baked into the image) is mounted over ${DN_PROJECT_PATH},
  # MASKING the baked-in '.git' and breaking the DN/N2ST bootstrap. Explicit --bind of src/,
  # utilities/, artifact/, data/ below still overlays the live host code as intended.
  flags+=("    --no-mount cwd \\")

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
