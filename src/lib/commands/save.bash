#!/bin/bash
# lib/commands/save.bash

DOCUMENTATION_BUFFER_SAVE=$( cat <<'EOF'
# =================================================================================================
# Save Docker image to file for offline use
#
# Usage:
#   $ dna save [OPTIONS] DIRPATH SERVICE
#
# Options:
#   --help, -h                    Show this help message
#   --apptainer <profile>         Also generate Apptainer artifacts (slurm service only):
#                                   - build_sif.sh helper script (run on HPC to convert tar→SIF)
#                                   - Apptainer metadata in meta.txt
#                                 <profile> selects .env.<profile> server configuration
#                                 e.g., dna save --apptainer valeria DIRPATH slurm
#                                 Note: apptainer is NOT executed locally (macOS compatible)
#
# Arguments:
#   DIRPATH                       Directory path where to save the image
#   SERVICE                       Service to save (develop, deploy, or slurm with --apptainer)
#
# Notes:
#   - Creates a portable archive containing the Docker image and necessary files
#   - For deploy service: includes full project structure for self-contained deployment
#   - For develop service: includes only the Docker image (assumes project is cloned on target)
#   - For slurm + --apptainer: saves tar archive and generates build_sif.sh for HPC conversion
#   - Output directory follows pattern: dna-save-<SERVICE>-<REPO_NAME>-<timestamp>
#
# =================================================================================================
EOF
)

# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dna_error_prefix="\033[1;31m[dna error]\033[0m"
test -n "$( declare -f dna::import_lib_and_dependencies )" || { echo -e "${dna_error_prefix} The DNA lib is not loaded!" 1>&2 && exit 1; }
test -n "$( declare -f n2st::print_msg )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
test -d "${DNA_ROOT:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }
test -d "${DNA_LIB_PATH:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dna::save_command() {

    # ....Set env variables (pre cli)..............................................................
    local dirpath=""
    local service=""
    local apptainer_profile=""
    local original_command="$*"
    local line_format="${MSG_LINE_CHAR_BUILDER_LVL2}"
    local line_style="${MSG_LINE_STYLE_LVL2}"

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dna::command_help_menu "${DOCUMENTATION_BUFFER_SAVE:?err}"
                exit 0
                ;;
            --apptainer)
                if [[ -z "$2" ]]; then
                    dna::illegal_command_msg "save" "${original_command}" "The --apptainer flag requires a <profile> argument (e.g., valeria, compute_canada, mamba).\n"
                    return 1
                fi
                apptainer_profile="$2"
                shift 2
                ;;
            develop|deploy|slurm)
                # If service is already set, it's an error
                if [[ -n "${service}" ]]; then
                    dna::illegal_command_msg "save" "${original_command}" "Only one SERVICE can be specified.\n"
                    return 1
                fi
                service="$1"
                shift
                ;;
            *)
                # If dirpath is not set, this is the dirpath argument
                if [[ -z "${dirpath}" ]]; then
                    dirpath="$1"
                    shift
                elif [[ -z "${service}" ]]; then
                    # This might be an invalid service
                    dna::illegal_command_msg "save" "${original_command}" "Invalid SERVICE: $1. Valid services are: develop, deploy.\n"
                    return 1
                else
                    dna::illegal_command_msg "save" "${original_command}" "Unknown argument: $1\n"
                    return 1
                fi
                ;;
        esac
    done

    # ....Validate arguments.......................................................................
    if [[ -z "${dirpath}" ]]; then
        dna::illegal_command_msg "save" "${original_command}" "DIRPATH argument is required.\n"
        return 1
    fi

    if [[ -z "${service}" ]]; then
        dna::illegal_command_msg "save" "${original_command}" "SERVICE argument is required. Valid services are: develop, deploy.\n"
        return 1
    fi

    if [[ -z "${apptainer_profile}" && "${service}" == "slurm" ]]; then
        dna::illegal_command_msg "save" "${original_command}" "SERVICE=slurm requires the --apptainer <profile> flag.\n"
        return 1
    fi

    if [[ "${service}" != "develop" && "${service}" != "deploy" && "${service}" != "slurm" ]]; then
        dna::illegal_command_msg "save" "${original_command}" "Invalid SERVICE: ${service}. Valid services are: develop, deploy, slurm (with --apptainer).\n"
        return 1
    fi

    if [[ -n "${apptainer_profile}" && "${service}" != "slurm" ]]; then
        dna::illegal_command_msg "save" "${original_command}" "The --apptainer flag can only be used with SERVICE=slurm.\n"
        return 1
    fi


    # ....Load dependencies........................................................................
    source "${DNA_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1
    if [[ -n "${apptainer_profile}" ]]; then
        source "${DNA_LIB_PATH}/core/utils/apptainer_tools.bash" || return 1
        # Override DN_PROJECT_USER with the HPC server username from the profile env file
        dna::load_apptainer_profile_env "${apptainer_profile}" || return 1
    fi

    # ....Validate dirpath.........................................................................
    if [[ ! -d "${dirpath}" ]]; then
        n2st::print_msg_error "Directory does not exist: ${dirpath}"
        return 1
    fi

    # ....Set env variables (post cli).............................................................
    local timestamp
    timestamp=$(date +"%Y%m%d%H%M")
    local save_dir_name="dna-save-${service}-${SUPER_PROJECT_REPO_NAME}-${timestamp}"
    local save_dir_path="${dirpath}/${save_dir_name}"
    local image_name=${DN_PROJECT_HUB:?err}/${DN_PROJECT_IMAGE_NAME:?err}-${service}:${PROJECT_TAG:?err}
    local tar_filename="${DN_PROJECT_IMAGE_NAME}-${service}.${PROJECT_TAG}.tar"

    # ....Begin....................................................................................
    n2st::print_formated_script_header "save ${service} image procedure" "${line_format}" "${line_style}"

    # Create save directory
    n2st::print_msg "Creating save directory: ${save_dir_path}"
    mkdir -p "${save_dir_path}" || {
        n2st::print_msg_error "Failed to create save directory: ${save_dir_path}"
        return 1
    }

    # Save Docker image
    n2st::print_msg "Saving Docker image: ${image_name}"
    docker image save --output "${save_dir_path}/${tar_filename}" "${image_name}" || {
        n2st::print_msg_error "Failed to save Docker image: ${image_name}"
        return 1
    }

    # Create meta.txt file
    n2st::print_msg "Creating metadata file"
    dna::create_save_metadata "${save_dir_path}/meta.txt" "${service}" "${tar_filename}" "${timestamp}" "${apptainer_profile}" || {
        n2st::print_msg_error "Failed to create metadata file"
        return 1
    }

    # Generate Apptainer artifacts if requested
    if [[ -n "${apptainer_profile}" ]]; then
        dna::check_apptainer_profile_env_file "${apptainer_profile}" || return 1
        local sif_name="${DN_PROJECT_IMAGE_NAME}-slurm.sif"
        dna::generate_apptainer_build_sif_script \
            "${tar_filename}" \
            "${sif_name}" \
            "${save_dir_path}" || {
            n2st::print_msg_error "Failed to generate build_sif.sh"
            return 1
        }
        n2st::print_msg_done "Apptainer build_sif.sh generated in: ${save_dir_path}"
    fi

    # For deploy service, copy project structure
    if [[ "${service}" == "deploy" ]]; then
        dna::copy_project_structure_for_deploy "${save_dir_path}" || {
            n2st::print_msg_error "Failed to copy project structure"
            return 1
        }
    fi

    # Sanity check
    if [[ ! -f "${save_dir_path}/${tar_filename}" ]]; then
        n2st::print_msg_error "Docker image archive file ${MSG_DIMMED_FORMAT}${tar_filename}${MSG_END_FORMAT} not found in ${MSG_DIMMED_FORMAT}${save_dir_path}${MSG_END_FORMAT}"
        return 1
    fi


    n2st::print_msg_done "Save completed successfully"
    n2st::print_msg "Saved to: ${save_dir_path}"
    n2st::print_formated_script_footer "save ${service} image procedure" "${line_format}" "${line_style}"
    return 0
}

# =================================================================================================
# Create metadata file with important information
#
# Arguments:
#   $1: Path to metadata file
#   $2: Service name (develop or deploy)
# =================================================================================================
function dna::create_save_metadata() {
    local meta_file="$1"
    local service="$2"
    local tar_filename="$3"
    local timestamp="$4"
    local apptainer_profile="${5:-}"

    local current_branch
    current_branch=$(cd "${SUPER_PROJECT_ROOT:?err}" && git branch --show-current 2>/dev/null || echo "unknown")

    local current_commit
    current_commit=$(cd "${SUPER_PROJECT_ROOT}" && git rev-parse HEAD 2>/dev/null || echo "unknown")

    local the_hostname
    the_hostname="$(hostname -s)"

    n2st::set_which_architecture_and_os

    cat > "${meta_file}" << EOF
# DNA Save Metadata
#   Generated on: $(date)
#   From host:
#     Name: ${the_hostname}
#     Architecture and OS: ${IMAGE_ARCH_AND_OS:?err}

# Configuration
DNA_CONFIG_SCHEME_VERSION=${DNA_CONFIG_SCHEME_VERSION:-unknown}
DN_PROJECT_GIT_REMOTE_URL=${DN_PROJECT_GIT_REMOTE_URL:-unknown}
DN_PROJECT_ALIAS_PREFIX=${DN_PROJECT_ALIAS_PREFIX:-unknown}

# Project Information
SUPER_PROJECT_REPO_NAME=${SUPER_PROJECT_REPO_NAME:-unknown}
SERVICE=${service}
IMAGE_NAME=${DN_PROJECT_IMAGE_NAME:?err}-${service}.${PROJECT_TAG:?err}

# Git Information
BRANCH=${current_branch}
COMMIT=${current_commit}

# Save Information
SAVE_DATE=$(date)
SAVE_TIMESTAMP=${timestamp:?err}
TAR_FILENAME=${tar_filename:?err}
EOF

    # Append Apptainer metadata if profile is set
    if [[ -n "${apptainer_profile}" ]]; then
        local sif_name="${DN_PROJECT_IMAGE_NAME:?err}-slurm.sif"
        cat >> "${meta_file}" << EOF

# Apptainer Information
APPTAINER_PROFILE=${apptainer_profile}
APPTAINER_TARGET_PLATFORM=linux/amd64
DN_PROJECT_USER=${DN_PROJECT_USER:-unknown}
SIF_NAME=${sif_name}
SIF_BUILD_CMD=apptainer build ${sif_name} docker-archive:${tar_filename:?err}
EOF
    fi

    return 0
}

# =================================================================================================
# Copy project structure for deploy service
#
# Arguments:
#   $1: Save directory path
# =================================================================================================
function dna::copy_project_structure_for_deploy() {
    local save_dir_path="$1"
    local project_copy_path="${save_dir_path}/${SUPER_PROJECT_REPO_NAME}"

    n2st::print_msg "Copying project structure for deploy service"

    # Create project directory
    mkdir -p "${project_copy_path}" || return 1

    # ....Copy .dockerized_norlab directory................................................
    echo -e "       ↳ Copying .dockerized_norlab configuration"
    cp -r "${SUPER_PROJECT_ROOT}/.dockerized_norlab" "${project_copy_path}/" || return 1

    # ....Copy .git directory (full copy for complete git history).................................
    echo -e "       ↳ Copying .git directory"
    # Use rsync or cp with better error handling for git files with permission issues
    if command -v rsync >/dev/null 2>&1; then
        rsync -a --ignore-errors "${SUPER_PROJECT_ROOT}/.git/" "${project_copy_path}/.git/" 2>/dev/null || {
            n2st::print_msg_warning "Some git files could not be copied due to permission issues, but core git data was preserved"
        }
    else
        # Fallback to cp with error handling
        cp -r "${SUPER_PROJECT_ROOT}/.git" "${project_copy_path}/" 2>/dev/null || {
            n2st::print_msg_warning "Some git files could not be copied due to permission issues"
            # Try to copy at least the essential git files
            mkdir -p "${project_copy_path}/.git"
            cp -r "${SUPER_PROJECT_ROOT}/.git/config" "${project_copy_path}/.git/" 2>/dev/null || true
            cp -r "${SUPER_PROJECT_ROOT}/.git/HEAD" "${project_copy_path}/.git/" 2>/dev/null || true
            cp -r "${SUPER_PROJECT_ROOT}/.git/refs" "${project_copy_path}/.git/" 2>/dev/null || true
        }
    fi

    # ....Create empty artifact and data directories...............................................
    echo -e "       ↳ Creating artifact directories"
    mkdir -p "${project_copy_path}/artifact" || return 1
    echo -e "       ↳ Creating data directories"
    mkdir -p "${project_copy_path}/data/external_data" || return 1
    mkdir -p "${project_copy_path}/data/repository_data" || return 1
    mkdir -p "${project_copy_path}/data/shared_data" || return 1

    return 0
}
