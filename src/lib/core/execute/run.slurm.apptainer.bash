#!/bin/bash
DOCUMENTATION_RUN_SLURM_APPTAINER=$( cat <<'EOF'
# =================================================================================================
# Generate Apptainer exec command/script for running a slurm job on an HPC server.
#
# Design constraints:
#   - Apptainer is Linux-only and NOT supported on macOS.
#   - This script GENERATES the apptainer exec command/script — it does NOT execute 'apptainer'.
#   - Generated scripts are self-contained (no DNA dependency on the HPC server).
#   - Supported HPC server archetypes:
#       Mamba (NorLab): Docker + Apptainer runtime → use 'dna run slurm' (Docker) or --ga mamba (Apptainer)
#       Valeria:        Apptainer runtime, no DNA on server  → use generated standalone script
#       Compute Canada: Apptainer runtime, no DNA on server  → use generated standalone script
#
# Usage:
#   $ source run.slurm.apptainer.bash
#   $ dna::run_slurm_apptainer <sjob-id> --apptainer <profile> [<optional-flag>] [--] <python-args>
#
# Or via dna CLI:
#   $ dna run slurm --apptainer <profile> <sjob-id> [--] <python-args>
#
# Optional flags:
#   --apptainer <profile>                             HPC server profile (e.g., valeria, compute_canada, mamba)
#   --sif-path <path>                                 Path to the SIF file on the HPC server
#                                                     (default: artifact/apptainer/<image>-slurm.sif)
#   --output-dir <path>                               Directory for generated scripts
#                                                     (default: artifact/apptainer/)
#   --print-only                                      Print apptainer exec command to stdout only
#                                                     (do not write script file)
#   --log-name <name>                                 Log file name (for script header comment)
#   -h | --help                                       Show this help message
#
# Positional arguments:
#   <sjob-id>              (required) Used to ID the slurm job and name the generated script
#   <python-args>          (required) The python command with flags
#
# Globals:
#   read DNA_ROOT
#   read SUPER_PROJECT_ROOT
#   read DN_PROJECT_IMAGE_NAME
#   read PROJECT_TAG
#
# =================================================================================================
EOF
)

# ....Script setup.................................................................................
pushd "$(pwd)" >/dev/null || exit 1

# .................................................................................................
function dna::show_help_apptainer() {
  echo -e "${MSG_DIMMED_FORMAT}"
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo -e "$0 --help"
  echo -e "${DOCUMENTATION_RUN_SLURM_APPTAINER}" | sed '/\# ====.*/d' | sed 's/^\# //' | sed 's/^\#//'
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo -e "${MSG_END_FORMAT}"
}

# =================================================================================================
# Generate the Apptainer exec command/script for a slurm job on an HPC server.
#
# Does NOT execute apptainer locally (macOS compatible).
# Generates a standalone run script that can be copied to the HPC server and executed there.
#
# Usage:
#   $ dna::run_slurm_apptainer <sjob-id> --apptainer <profile> [OPTIONS] [--] <python-args>
#
# Returns:
#   0 on success
#   1 on error
# =================================================================================================
function dna::run_slurm_apptainer() {
  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)

  # Positional argument
  local SJOB_ID="$1"
  shift

  if [[ "${SJOB_ID}" == "--help" ]] || [[ "${SJOB_ID}" == "-h" ]]; then
    dna::show_help_apptainer
    return 0
  fi

  # ....Set env variables (pre cli)................................................................
  local apptainer_profile=""
  local sif_path=""
  local output_dir=""
  local print_only=false
  local log_name="slurm_apptainer_job"
  declare -a python_arg=()

  # ....cli........................................................................................
  while [[ $# -gt 0 ]]; do
    case $1 in
      --apptainer)
        apptainer_profile="${2:?'--apptainer requires a <profile> argument'}"
        shift
        shift
        ;;
      --sif-path)
        sif_path="${2:?'--sif-path requires a path argument'}"
        shift
        shift
        ;;
      --output-dir)
        output_dir="${2:?'--output-dir requires a path argument'}"
        shift
        shift
        ;;
      --print-only)
        print_only=true
        shift
        ;;
      --log-name)
        log_name="${2}"
        shift
        shift
        ;;
      -h | --help)
        dna::show_help_apptainer
        return 0
        ;;
      --) # no more options
        shift
        python_arg=("$@")
        break
        ;;
      *) # Default case
        python_arg=("$@")
        break
        ;;
    esac
  done

  # ....Sanity checks..............................................................................
  test -n "${SJOB_ID}" || { n2st::print_msg_error "Missing sjob-id mandatory positional argument!"; return 1; }
  test -n "${apptainer_profile}" || { n2st::print_msg_error "Missing --apptainer <profile> flag!"; return 1; }
  test -n "${python_arg[0]}" || { n2st::print_msg_error "Missing <python-args> mandatory positional argument!"; return 1; }

  # ....Load apptainer tools.......................................................................
  source "${DNA_LIB_PATH:?err}/core/utils/apptainer_tools.bash" || return 1

  # ....Validate profile env file..................................................................
  dna::check_apptainer_profile_env_file "${apptainer_profile}" || return 1

  # ....Set env variables (post cli)...............................................................
  local apptainer_save_dir="${SUPER_PROJECT_ROOT:?err}/artifact/apptainer"
  local default_sif_path="artifact/apptainer/${DN_PROJECT_IMAGE_NAME:?err}-slurm.sif"

  if [[ -z "${sif_path}" ]]; then
    sif_path="${default_sif_path}"
  fi

  if [[ -z "${output_dir}" ]]; then
    output_dir="${apptainer_save_dir}"
  fi

  mkdir -p "${output_dir}" || {
    n2st::print_msg_error "Failed to create output directory: ${output_dir}"
    return 1
  }

  # ====Begin======================================================================================
  n2st::print_msg "Generating Apptainer run artifacts for profile: ${apptainer_profile}"
  n2st::print_msg "  SJOB_ID: ${SJOB_ID}"
  n2st::print_msg "  SIF path (on HPC): ${sif_path}"
  n2st::print_msg "  Python args: ${python_arg[*]}"

  if [[ "${print_only}" == true ]]; then
    # ....Print mode: output apptainer exec command to stdout only...............................
    n2st::print_msg "Apptainer exec command (run on HPC server, NOT locally):"
    echo ""
    dna::print_apptainer_exec_command \
      "${apptainer_profile}" \
      "${sif_path}" \
      "${python_arg[@]}"
  else
    # ....Script generation mode: write standalone run script....................................
    local generated_script
    generated_script=$(dna::generate_apptainer_run_script \
      "${SJOB_ID}" \
      "${apptainer_profile}" \
      "${sif_path}" \
      "${output_dir}" \
      "${python_arg[@]}") || {
      n2st::print_msg_error "Failed to generate Apptainer run script"
      return 1
    }

    n2st::print_msg_done "Apptainer run script generated: ${generated_script}"
    echo ""
    n2st::print_msg "Next steps:
  1. Ensure SIF exists on HPC (build with bash artifact/apptainer/dna_tar_to_apptainer_sif_converter.sh if needed)
  2. Transfer script to HPC: scp ${generated_script} user@hpc:/path/to/project/
  3. Submit job on HPC: sbatch slurm_job.apptainer.${apptainer_profile}.template.bash
     Or run directly: bash $(basename "${generated_script}")"
  fi

  # ....Teardown...................................................................................
  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  popd >/dev/null || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}


# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dna_error_prefix="\033[1;31m[dna error]\033[0m"
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  # ....Source project shell-scripts dependencies..................................................
  script_path="$(realpath -q "${BASH_SOURCE[0]:-.}")"
  script_path_parent="$(dirname "${script_path}")"
  if [[ -z "$( declare -f dna::import_lib_and_dependencies )" ]]; then
    source "${script_path_parent}/../utils/import_dna_lib.bash" || exit 1
    source "${script_path_parent}/../utils/load_super_project_config.bash" || exit 1
  fi

  # ....Execute....................................................................................
  if [[ "${DNA_CLEAR_CONSOLE_ACTIVATED}" == "true" ]]; then
    clear
  fi
  n2st::norlab_splash "${DNA_SPLASH_NAME_FULL:?err}" "${DNA_GIT_REMOTE_URL}" "negative"
  n2st::print_formated_script_header "$(basename "$0")" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  dna::run_slurm_apptainer "$@"
  fct_exit_code=$?
  n2st::print_formated_script_footer "$(basename "$0")" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  exit "${fct_exit_code}"
else
  # This script is being sourced, ie: __name__="__source__"

  # ....Pre-condition..............................................................................
  test -n "$( declare -f dna::import_lib_and_dependencies )" || { echo -e "${dna_error_prefix} The DNA lib is not loaded!" 1>&2 && exit 1; }
  test -n "$( declare -f n2st::print_msg )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
  test -n "${SUPER_PROJECT_ROOT}" || { echo -e "${dna_error_prefix} The super project DNA configuration is not loaded!" 1>&2 && exit 1; }
fi
