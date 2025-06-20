#!/bin/bash
# lib/commands/save.bash

DOCUMENTATION_BUFFER_SAVE=$( cat <<'EOF'
# =================================================================================================
# Save Docker image to file for offline use
#
# Usage:
#   $ dnp save [OPTIONS] DIRPATH SERVICE
#
# Options:
#   --help, -h                    Show this help message
#
# Arguments:
#   DIRPATH                       Directory path where to save the image
#   SERVICE                       Service to save (develop or deploy)
#
# Notes:
#   - Creates a portable archive containing the Docker image and necessary files
#   - For deploy service: includes full project structure for self-contained deployment
#   - For develop service: includes only the Docker image (assumes project is cloned on target)
#   - Output directory follows pattern: dnp-save-<SERVICE>-<REPO_NAME>-<timestamp>
#
# =================================================================================================
EOF
)

# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
test -n "$( declare -f dnp::import_lib_and_dependencies )" || { echo -e "${dnp_error_prefix} The DNP lib is not loaded!" ; exit 1 ; }
test -n "$( declare -f n2st::print_msg )" || { echo -e "${dnp_error_prefix} The N2ST lib is not loaded!" ; exit 1 ; }
test -d "${DNP_ROOT:?err}" || { echo -e "${dnp_error_prefix} library load error!" ; exit 1 ; }
test -d "${DNP_LIB_PATH:?err}" || { echo -e "${dnp_error_prefix} library load error!" ; exit 1 ; }

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dnp::save_command() {

    # ....Set env variables (pre cli)..............................................................
    local dirpath=""
    local service=""
    local original_command="$*"

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_SAVE:?err}"
                exit 0
                ;;
            develop|deploy)
                # If service is already set, it's an error
                if [[ -n "${service}" ]]; then
                    dnp::illegal_command_msg "save" "${original_command}" "Only one SERVICE can be specified.\n"
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
                    dnp::illegal_command_msg "save" "${original_command}" "Invalid SERVICE: $1. Valid services are: develop, deploy.\n"
                    return 1
                else
                    dnp::illegal_command_msg "save" "${original_command}" "Unknown argument: $1\n"
                    return 1
                fi
                ;;
        esac
    done

    # ....Validate arguments.......................................................................
    if [[ -z "${dirpath}" ]]; then
        dnp::illegal_command_msg "save" "${original_command}" "DIRPATH argument is required.\n"
        return 1
    fi

    if [[ -z "${service}" ]]; then
        dnp::illegal_command_msg "save" "${original_command}" "SERVICE argument is required. Valid services are: develop, deploy.\n"
        return 1
    fi

    if [[ "${service}" != "develop" && "${service}" != "deploy" ]]; then
        dnp::illegal_command_msg "save" "${original_command}" "Invalid SERVICE: ${service}. Valid services are: develop, deploy.\n"
        return 1
    fi

    # ....Load dependencies........................................................................
    source "${DNP_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1

    # ....Validate dirpath.........................................................................
    if [[ ! -d "${dirpath}" ]]; then
        n2st::print_msg_error "Directory does not exist: ${dirpath}"
        return 1
    fi

    # ....Set env variables (post cli).............................................................
    local timestamp
    timestamp=$(date +"%Y%m%d%H%M")
    local save_dir_name="dnp-save-${service}-${SUPER_PROJECT_REPO_NAME}-${timestamp}"
    local save_dir_path="${dirpath}/${save_dir_name}"
    local image_name=${DN_PROJECT_HUB:?err}/${DN_PROJECT_IMAGE_NAME:?err}-${service}:${PROJECT_TAG:?err}
    local tar_filename="${DN_PROJECT_IMAGE_NAME}-${service}.${PROJECT_TAG}.tar"

    # ....Begin....................................................................................
    n2st::print_formated_script_header "Save ${service} image procedure" "${MSG_LINE_CHAR_BUILDER_LVL1}"

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
    dnp::create_save_metadata "${save_dir_path}/meta.txt" "${service}" "${tar_filename}" "${timestamp}" || {
        n2st::print_msg_error "Failed to create metadata file"
        return 1
    }

    # For deploy service, copy project structure
    if [[ "${service}" == "deploy" ]]; then
        dnp::copy_project_structure_for_deploy "${save_dir_path}" || {
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
    n2st::print_formated_script_footer "Save ${service} image procedure" "${MSG_LINE_CHAR_BUILDER_LVL1}"

    return 0
}

# =================================================================================================
# Create metadata file with important information
#
# Arguments:
#   $1: Path to metadata file
#   $2: Service name (develop or deploy)
# =================================================================================================
function dnp::create_save_metadata() {
    local meta_file="$1"
    local service="$2"
    local tar_filename="$3"
    local timestamp="$4"

    local current_branch
    current_branch=$(cd "${SUPER_PROJECT_ROOT:?err}" && git branch --show-current 2>/dev/null || echo "unknown")

    local current_commit
    current_commit=$(cd "${SUPER_PROJECT_ROOT}" && git rev-parse HEAD 2>/dev/null || echo "unknown")

    local the_hostname
    the_hostname="$(hostname -s)"

    n2st::set_which_architecture_and_os

    cat > "${meta_file}" << EOF
# DNP Save Metadata
#   Generated on: $(date)
#   From host:
#     Name: ${the_hostname}
#     Architecture and OS: ${IMAGE_ARCH_AND_OS:?err}

# Configuration
DNP_CONFIG_SCHEME_VERSION=${DNP_CONFIG_SCHEME_VERSION:-unknown}
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

    return 0
}

# =================================================================================================
# Copy project structure for deploy service
#
# Arguments:
#   $1: Save directory path
# =================================================================================================
function dnp::copy_project_structure_for_deploy() {
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

    # ....Create empty artifact and external_data directories......................................
    echo -e "       ↳ Creating artifact directory"
    mkdir -p "${project_copy_path}/artifact" || return 1
    echo -e "       ↳ Creating external_data directory"
    mkdir -p "${project_copy_path}/external_data" || return 1

    return 0
}
