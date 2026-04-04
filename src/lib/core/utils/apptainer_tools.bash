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
#   $ bash artifact/apptainer/dna_tar_to_apptainer_sif_converter.sh
#
# Requires:
#   - apptainer installed on the HPC server
#   - The Docker tar archive in the same directory as this script (artifact/apptainer/)
#
# =================================================================================================
set -e

# ====Apptainer compatibility======================================================================
echo "[info] This script requires Apptainer >= 1.1.0 (for --no-eval, --cleanenv, --env-file comment support)." 1>&2

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SUPER_PROJECT_ROOT="${SUPER_PROJECT_ROOT:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
SCRIPT_EOF

  # Inject the tar_filename and sif_name variables (expand at generation time)
  cat >> "${script_path}" << EOF
TAR_FILENAME="${tar_filename}"
SIF_FILENAME="${sif_name}"
EOF

  cat >> "${script_path}" << 'SCRIPT_EOF'

TAR_FILE="${SCRIPT_DIR}/${TAR_FILENAME}"
SIF_FILE="${SCRIPT_DIR}/${SIF_FILENAME}"

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

apptainer build "${SIF_FILE}" "docker-archive:${TAR_FILE}"

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
#   $ flags=$(dna::get_apptainer_slurm_exec_flags "valeria" "/path/to/sif")
#
# Positional arguments:
#   profile   - HPC server profile name (e.g., 'valeria', 'compute_canada', 'mamba')
#   sif_path  - Path to the SIF file on the HPC server (used in script template)
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
# It embeds all necessary apptainer exec flags resolved from the DNA compose configuration.
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
  exec_flags=$(dna::get_apptainer_slurm_exec_flags "${profile}" "${sif_path}")

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
    ${entrypoint} \\
    ${python_args_str} "\$@"

exit_code=\$?
echo "[info] Apptainer exec exited with code: \${exit_code}"
exit \${exit_code}
EOF

  chmod +x "${script_path}"
  n2st::print_msg_done "Generated: ${script_path}"
  echo "${script_path}"
  return 0
}


# =================================================================================================
# Prints the apptainer exec command equivalent for the slurm service.
#
# Used by 'dna run slurm --generate-apptainer <profile>' to display the command that will run on the HPC.
# Does NOT execute apptainer locally (macOS compatibility).
#
# Usage:
#   $ dna::print_apptainer_exec_command "valeria" "./artifact/sif/myproject.sif" \
#       "launcher/train.py" "--epochs=10"
#
# Positional arguments:
#   profile     - HPC server profile name
#   sif_path    - Path to the SIF file on the HPC server
#   python_args - Python command and arguments (all remaining args)
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
  shift 2
  local python_args=("$@")

  local entrypoint="/dockerized-norlab/project/project-slurm/dn_entrypoint.init.bash"
  local exec_flags
  exec_flags=$(dna::get_apptainer_slurm_exec_flags "${profile}" "${sif_path}")

  echo "# Apptainer exec command (run on HPC server, NOT locally)"
  echo "# Generated by: dna run slurm --generate-apptainer ${profile}"
  echo "apptainer exec \\"
  echo "${exec_flags}"
  echo "    \"\${SIF_PATH}\" \\"
  printf '    %s \\\n' "${entrypoint}"
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
