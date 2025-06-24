#!/bin/bash
# lib/commands/load.bash

DOCUMENTATION_BUFFER_LOAD=$( cat <<'EOF'
# =================================================================================================
# Load Docker image from file for offline use
#
# Usage:
#   $ dnp load [OPTIONS] [SAVE_DIR_PATH]
#
# Options:
#   --help, -h                    Show this help message
#
# Arguments:
#   SAVE_DIR_PATH                 Path to the saved directory (dnp-save-<SERVICE>-<REPO_NAME>-<timestamp>)
#                                 If not provided, assumes current directory is SAVE_DIR_PATH or searches
#                                 for DNP configuration directory.
#
# Notes:
#   - Loads Docker image from the saved archive
#   - For deploy service: changes to the loaded project directory after loading
#   - For develop service: executes alias dnp-<DN_PROJECT_ALIAS_PREFIX>-cd after loading
#   - Validates metadata and archive integrity before loading
#
# =================================================================================================
EOF
)

# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
test -n "$( declare -f dnp::import_lib_and_dependencies )" || { echo -e "${dnp_error_prefix} The DNP lib is not loaded!" 1>&2 && exit 1; }
test -n "$( declare -f n2st::print_msg )" || { echo -e "${dnp_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
test -d "${DNP_ROOT:?err}" || { echo -e "${dnp_error_prefix} library load error!" 1>&2 && exit 1; }
test -d "${DNP_LIB_PATH:?err}" || { echo -e "${dnp_error_prefix} library load error!" 1>&2 && exit 1; }

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dnp::load_command() {

    # ....Set env variables (pre cli)..............................................................
    local save_dir_path=""
    local original_command="$*"
    local line_format="${MSG_LINE_CHAR_BUILDER_LVL2}"
    local line_style="${MSG_LINE_STYLE_LVL2}"

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_LOAD}"
                exit 0
                ;;
            *)
                # If save_dir_path is not set, this is the save directory path argument
                if [[ -z "${save_dir_path}" ]]; then
                    save_dir_path="$1"
                    shift
                else
                    dnp::illegal_command_msg "load" "${original_command}" "Unknown argument: $1\n"
                    return 1
                fi
                ;;
        esac
    done


    # ....Determine save directory path................................................................
    if [[ -z "${save_dir_path}" ]]; then
        # No SAVE_DIR_PATH provided, assume we are already in a 'dnp save' directory
        #  - Case 1: cwd is at the root
        #  - Case 2: cwd is deeper in the directory structure
        if [[ -f "meta.txt" ]]; then
            save_dir_path="$(pwd)"
            n2st::print_msg "Using current directory as save directory: ${save_dir_path}"
        else
            # Try to find super project root (assume its a dnp saved deploy project)
            local original_cwd
            original_cwd="$(pwd)"

            # Source the load_super_project_config to get access to dnp::cd_to_dnp_super_project_root
            source "${DNP_LIB_PATH}/core/utils/load_super_project_config.bash" --no-execute || return 1

            if dnp::cd_to_dnp_super_project_root; then
                if [[ -f "meta.txt" ]]; then
                    save_dir_path="$(pwd)"
                    n2st::print_msg "Found meta.txt in super project root: ${save_dir_path}"
                else
                    cd "${original_cwd}" || true
                    n2st::print_msg_error "No meta.txt found in current directory or super project root. Please provide SAVE_DIR_PATH or ensure meta.txt exists in current directory."
                    return 1
                fi
            else
                cd "${original_cwd}" || true
                n2st::print_msg_error "No meta.txt found in current directory and could not locate super project root. Please provide SAVE_DIR_PATH."
                return 1
            fi
        fi
    fi

    if [[ ! -d "${save_dir_path}" ]]; then
        n2st::print_msg_error "Save directory does not exist: ${save_dir_path}"
        return 1
    fi

    # ....Validate save directory structure...........................................................
    local meta_file="${save_dir_path}/meta.txt"
    if [[ ! -f "${meta_file}" ]]; then
        n2st::print_msg_error "Invalid save directory: meta.txt not found in ${save_dir_path}"
        return 1
    fi

    # ....Extract metadata.............................................................................
    local service
    local image_name
    local repo_name
    local alias_prefix
    local tar_filename

    service=$(grep "^SERVICE=" "${meta_file}" | cut -d'=' -f2)
    image_name=$(grep "^IMAGE_NAME=" "${meta_file}" | cut -d'=' -f2)
    repo_name=$(grep "^SUPER_PROJECT_REPO_NAME=" "${meta_file}" | cut -d'=' -f2)
    alias_prefix=$(grep "^DN_PROJECT_ALIAS_PREFIX=" "${meta_file}" | cut -d'=' -f2)
    tar_filename=$(grep "^TAR_FILENAME=" "${meta_file}" | cut -d'=' -f2)

    if [[ -z "${service}" ]] || [[ -z "${image_name}" ]] || [[ -z "${repo_name}" ]] || [[ -z "${alias_prefix}" ]] || [[ -z "${tar_filename}" ]]; then
        n2st::print_msg_error "Invalid metadata file: missing required fields"
        return 1
    fi

    # ....Validate tar file exists first...............................................................
    if [[ ! -f "${save_dir_path}/${tar_filename}" ]]; then
        n2st::print_msg_error "Docker image archive file ${MSG_DIMMED_FORMAT}.tar${MSG_END_FORMAT} not found in ${MSG_DIMMED_FORMAT}${save_dir_path}${MSG_END_FORMAT}"
        return 1
    fi


    # ....Begin....................................................................................
    n2st::print_formated_script_header "load ${service} image procedure" "${line_format}" "${line_style}"

    # Display metadata information
    n2st::print_msg "Loading saved image:
  Service: ${service}
  Image: ${image_name}
  Repository: ${repo_name}
  Archive: ${tar_filename}
"

    # Load Docker image
    n2st::print_msg "Loading Docker image from: $(basename "${tar_filename}")"
    docker image load --input "${save_dir_path}/${tar_filename}" || {
        n2st::print_msg_error "Failed to load Docker image from: ${save_dir_path}/${tar_filename}" && return 1
    }
    n2st::print_msg_done "Load completed successfully"
    n2st::print_formated_script_footer "load ${service} image procedure" "${line_format}" "${line_style}"

    # Handle post-load actions based on service type
    if [[ "${service}" == "deploy" ]]; then
        dnp::handle_deploy_post_load "${save_dir_path}" "${repo_name}" || {
            n2st::print_msg_error "Failed to handle deploy post-load actions"
            return 1
        }
    elif [[ "${service}" == "develop" ]]; then
        dnp::handle_develop_post_load "${alias_prefix}" || {
            n2st::print_msg_error "Failed to handle develop post-load actions"
            return 1
        }
    else
      n2st::print_msg_error "Could not complete post-load actions for ${service}"
      return 1
    fi

    return 0
}

# =================================================================================================
# Handle post-load actions for deploy service
#
# Arguments:
#   $1: Save directory path
#   $2: Repository name
# =================================================================================================
function dnp::handle_deploy_post_load() {
    local save_dir_path="$1"
    local repo_name="$2"
    local project_path="${save_dir_path}/${repo_name}"

    if [[ ! -d "${project_path}" ]]; then
        n2st::print_msg_error "Project directory not found: ${project_path}"
        return 1
    fi

    n2st::print_msg_done "Load completed successfully. To change to the project directory, run:
    cd \"${project_path}\"

You can then run 'dnp up deploy' or 'dnp run deploy' commands"
    return 0
}

# =================================================================================================
# Handle post-load actions for develop service
#
# Arguments:
#   $1: DN_PROJECT_ALIAS_PREFIX
# =================================================================================================
function dnp::handle_develop_post_load() {
    local alias_prefix="$1"

    if [[ -n "${alias_prefix}" ]]; then
        local alias_command="dnp-${alias_prefix}-cd"
        n2st::print_msg "Check aliases: ${alias_command}"

        if command -v "${alias_command}" >/dev/null 2>&1; then
            if "${alias_command}"; then
                return 0
            else
                n2st::print_msg_warning "Failed to execute alias: ${alias_command}"
                n2st::print_msg "You may need to manually navigate to your project directory"
            fi
        else
            n2st::print_msg_warning "Alias not found: ${alias_command}
    Please manually navigate to your project directory and run ${MSG_DIMMED_FORMAT}dnp [up|run] develop${MSG_END_FORMAT}"
        fi
    else
        n2st::print_msg_warning "DN_PROJECT_ALIAS_PREFIX not found in metadata
    Please manually navigate to your project directory and run ${MSG_DIMMED_FORMAT}dnp [up|run] develop${MSG_END_FORMAT}"
    fi

    return 0
}
