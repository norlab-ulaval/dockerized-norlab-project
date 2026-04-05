#!/bin/bash
# =================================================================================================
# Helper functions for configuration scheme patching.
#
# Usage:
#   source patch_helper.bash
#
# =================================================================================================

# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dna_error_prefix="\033[1;31m[dna error]\033[0m"
test -n "$( declare -f n2st::print_msg )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }

# =================================================================================================
# Check if the super project configuration scheme is outdated and trigger patching if needed.
#
# If patching is performed, it updates the DNA_CONFIG_SCHEME_VERSION in the super project's 
# meta dotenv file and the current environment upon completion.
#
# Usage:
#   $ dna::patch_check_and_run
#
# Returns:
#   0 on success or if no patching is needed
#   1 on patching failure
# =================================================================================================
function dna::patch_check_and_run() {
    # Initialize "yes to all" flag
    local DNA_PATCH_YES_TO_ALL="false"

    # Ensure super project configurations are loaded
    if [[ -z "${SUPER_PROJECT_ROOT}" ]] || [[ -z "${DNA_CONFIG_SCHEME_VERSION}" ]]; then
        # If not loaded, try to load them
        if [[ -f "${DNA_LIB_PATH:?err}/core/utils/load_super_project_config.bash" ]]; then
            source "${DNA_LIB_PATH}/core/utils/load_super_project_config.bash" --no-execute || return 0
            dna::load_super_project_configurations > /dev/null 2>&1 || return 0
        else
            return 0
        fi
    fi

    # Compare versions
    local current_scheme="${DNA_CONFIG_SCHEME_VERSION}"
    local target_scheme="${DNA_RELEASE_CONFIG_SCHEME_VERSION:?err}"

    if [[ "${current_scheme}" -lt "${target_scheme}" ]]; then
        n2st::print_msg_warning "Super project configuration scheme (v${current_scheme}) is outdated (target v${target_scheme})."
        
        local next_scheme
        for (( v=current_scheme; v<target_scheme; v++ )); do
            next_scheme=$((v+1))
            dna::patch_apply_version_patch "${v}" "${next_scheme}" || return 1
        done

        # Update DNA_CONFIG_SCHEME_VERSION in the super project meta dotenv file
        local super_project_meta_dna_dotenv=".env.${SUPER_PROJECT_REPO_NAME:?err}"
        local meta_dotenv_path="${SUPER_PROJECT_ROOT:?err}/.dockerized_norlab/${super_project_meta_dna_dotenv}"

        if [[ -f "${meta_dotenv_path}" ]]; then
            n2st::seek_and_modify_string_in_file "DNA_CONFIG_SCHEME_VERSION=${current_scheme}" "DNA_CONFIG_SCHEME_VERSION=${target_scheme}" "${meta_dotenv_path}"
        else
            n2st::print_msg_error "Super project meta dotenv file not found: ${meta_dotenv_path}"
            return 1
        fi
        
        n2st::print_msg_done "Super project configuration scheme successfully updated to v${target_scheme}."
        n2st::print_msg_warning "Be advised, you might need to restart your terminal session for some changes to take effect."
    fi

    return 0
}

# =================================================================================================
# Apply a specific configuration scheme patch.
#
# This function locates and executes a patch script for a specific version transition.
#
# After successful execution of the patch script, it updates the DNA_CONFIG_SCHEME_VERSION
# in the current environment.
#
# Usage:
#   $ dna::patch_apply_version_patch <from_version> <to_version>
#
# Arguments:
#   from_version: The current configuration scheme version of the super project.
#   to_version: The target configuration scheme version after the patch.
#
# Global variables used:
#   added_resources: Array to keep track of added resources for reporting.
#
# Returns:
#   0 on success
#   1 on failure (patch script not found or failed to execute)
# =================================================================================================
function dna::patch_apply_version_patch() {
    local from_v="$1"
    local to_v="$2"
    local patch_script="config_scheme_${from_v}to${to_v}.bash"
    local patch_path="${DNA_LIB_PATH:?err}/core/patches/${patch_script}"

    if [[ ! -f "${patch_path}" ]]; then
        n2st::print_msg_error "Patch script not found: ${patch_path}"
        return 1
    fi

    n2st::print_msg "Applying configuration scheme patch: v${from_v} → v${to_v}..."
    
    # Array to keep track of added resources for reporting within the patch script
    local -a added_resources=()

    # Execute the patch script
    # shellcheck disable=SC1090
    source "${patch_path}"
    
    local exit_code=$?
    if [[ ${exit_code} -ne 0 ]]; then
        n2st::print_msg_error "Failed to apply patch v${from_v} → v${to_v}"
        return 1
    fi

    # Report added resources for this patch
    if [[ ${#added_resources[@]} -gt 0 ]]; then
        n2st::print_msg "The following resources were added to the super project by patch v${from_v} → v${to_v}:"
        for res in "${added_resources[@]}"; do
            n2st::print_msg "  - ${res}"
        done
    else
        n2st::print_msg "No changes were made to the super project structure."
    fi
    
    # Update the environment variable for subsequent patches in the same run
    export DNA_CONFIG_SCHEME_VERSION="${to_v}"

    return 0
}

# =================================================================================================
# Prompt the user for confirmation and handle "yes to all" logic.
#
# This function updates the 'DNA_PATCH_YES_TO_ALL' variable if the user selects 'a'.
# It returns the choice in the 'REPLY' variable.
#
# Usage:
#   $ dna::patch_prompt_user <prompt_msg>
# =================================================================================================
function dna::patch_prompt_user() {
    local prompt_msg="$1"

    if [[ "${DNA_PATCH_YES_TO_ALL}" == "true" ]]; then
        REPLY="y"
        return
    fi

    local user_input
    read -r -n 1 -p "${prompt_msg} [y/N/a] " user_input
    echo

    if [[ "${user_input}" == "a" || "${user_input}" == "A" ]]; then
        DNA_PATCH_YES_TO_ALL="true"
        REPLY="y"
    else
        REPLY="${user_input}"
    fi
}

# =================================================================================================
# Add a missing file to the super project if it doesn't already exist.
#
# This function uses the 'added_resources' array to track modifications.
#
# Usage:
#   $ dna::patch_add_file_if_missing <source_file> <target_file> <description>
#
# Arguments:
#   source_file: Path to the template file (relative to src/lib/template).
#   target_file: Path to the target file in the super project (relative to SUPER_PROJECT_ROOT).
#   description: A brief description of the file for the user prompt.
#
# Global variables used:
#   added_resources: Array to keep track of added resources for reporting.
#
# Returns:
#   0 on success or if skipped
# =================================================================================================
function dna::patch_add_file_if_missing() {
    local source_file="$1"
    local target_file="$2"
    local description="$3"

    if [[ ! -f "${SUPER_PROJECT_ROOT}/${target_file}" ]]; then
        n2st::print_msg "Missing ${description}: ${target_file}"
        
        dna::patch_prompt_user "Add missing file?"
        local user_input="${REPLY}"
        
        if [[ "${user_input}" == "y" || "${user_input}" == "Y" ]]; then
            dna::portable_copy "${DNA_LIB_PATH}/template/${source_file}" "${SUPER_PROJECT_ROOT}/${target_file}" "${SUPER_PROJECT_ROOT}"
            added_resources+=("${target_file} (file)")
        else
            n2st::print_msg_warning "Skipping ${target_file}. This might cause issues."
        fi
    fi
}

# =================================================================================================
# Add a missing directory to the super project if it doesn't already exist.
#
# This function uses the 'added_resources' array to track modifications.
#
# Usage:
#   $ dna::patch_add_directory_if_missing <source_dir> <target_dir> <description>
#
# Arguments:
#   source_dir: Path to the template directory (relative to src/lib/template).
#   target_dir: Path to the target directory in the super project (relative to SUPER_PROJECT_ROOT).
#   description: A brief description of the directory for the user prompt.
#
# Global variables used:
#   added_resources: Array to keep track of added resources for reporting.
#
# Returns:
#   0 on success or if skipped
# =================================================================================================
function dna::patch_add_directory_if_missing() {
    local source_dir="$1"
    local target_dir="$2"
    local description="$3"

    if [[ ! -d "${SUPER_PROJECT_ROOT}/${target_dir}" ]]; then
        n2st::print_msg "Missing ${description}: ${target_dir}"
        
        dna::patch_prompt_user "Add missing directory?"
        local user_input="${REPLY}"
        
        if [[ "${user_input}" == "y" || "${user_input}" == "Y" ]]; then
            # Directories handled by rsync in portable_copy
            dna::portable_copy "${DNA_LIB_PATH}/template/${source_dir}/" "${SUPER_PROJECT_ROOT}/${target_dir}" "${SUPER_PROJECT_ROOT}"
            added_resources+=("${target_dir} (directory)")
        else
            n2st::print_msg_warning "Skipping ${target_dir}. This might cause issues."
        fi
    fi
}

# =================================================================================================
# Add missing content to a file in the super project if a search string is not found.
#
# Usage:
#   $ dna::patch_add_content_if_missing <target_file> <search_string> <content_to_add> <description>
#
# Arguments:
#   target_file: Path to the file in the super project (relative to SUPER_PROJECT_ROOT).
#   search_string: String to search for in the target file to determine if it needs patching.
#   content_to_add: The content to append to the file if search_string is not found.
#   description: A brief description of the content being added for the user prompt.
#
# Global variables used:
#   added_resources: Array to keep track of added resources for reporting.
#
# Returns:
#   0 on success or if skipped
#   1 on failure
# =================================================================================================
function dna::patch_add_content_if_missing() {
    local target_file="$1"
    local search_string="$2"
    local content_to_add="$3"
    local description="$4"

    local full_target_path="${SUPER_PROJECT_ROOT}/${target_file}"

    if [[ ! -f "${full_target_path}" ]]; then
        n2st::print_msg_error "Target file not found for patching: ${target_file}"
        return 1
    fi

    if ! grep -qF "${search_string}" "${full_target_path}"; then
        n2st::print_msg "Missing ${description} in ${target_file}"
        
        dna::patch_prompt_user "Add missing content?"
        local user_input="${REPLY}"
        
        if [[ "${user_input}" == "y" || "${user_input}" == "Y" ]]; then
            echo -e "${content_to_add}" >> "${full_target_path}"
            added_resources+=("${target_file} (content update)")
        else
            n2st::print_msg_warning "Skipping content update for ${target_file}. This might cause issues."
        fi
    fi
}

# =================================================================================================
# Modify content in a file in the super project using a search and replace pattern.
#
# Usage:
#   $ dna::patch_modify_content <target_file> <search_pattern> <replace_pattern> <description>
#
# Arguments:
#   target_file: Path to the file in the super project (relative to SUPER_PROJECT_ROOT).
#   search_pattern: String pattern to search for in the target file.
#   replace_pattern: String pattern to replace the search_pattern with.
#   description: A brief description of the modification for the user prompt.
#
# Global variables used:
#   added_resources: Array to keep track of added resources for reporting.
#
# Returns:
#   0 on success or if skipped
#   1 on failure
# =================================================================================================
function dna::patch_modify_content() {
    local target_file="$1"
    local search_pattern="$2"
    local replace_pattern="$3"
    local description="$4"

    local full_target_path="${SUPER_PROJECT_ROOT}/${target_file}"

    if [[ ! -f "${full_target_path}" ]]; then
        n2st::print_msg_error "Target file not found for patching: ${target_file}"
        return 1
    fi

    if grep -qF "${search_pattern}" "${full_target_path}"; then
        n2st::print_msg "Modifying ${description} in ${target_file}"
        
        dna::patch_prompt_user "Apply modification?"
        local user_input="${REPLY}"
        
        if [[ "${user_input}" == "y" || "${user_input}" == "Y" ]]; then
            n2st::seek_and_modify_string_in_file "${search_pattern}" "${replace_pattern}" "${full_target_path}"
            added_resources+=("${target_file} (content modification)")
        else
            n2st::print_msg_warning "Skipping modification for ${target_file}. This might cause issues."
        fi
    fi
}
