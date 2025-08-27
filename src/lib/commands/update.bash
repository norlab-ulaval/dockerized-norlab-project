#!/bin/bash
# lib/commands/update.bash

DOCUMENTATION_BUFFER_UPDATE=$( cat <<'EOF'
# =================================================================================================
# Update Dockerized-NorLab project application (DNA) to latest release
#
#   1. Fetch DNA repository release tags;
#   2. Update dna local version to latest release if user accept to proceed, flag `--yes` is used
#      or auto-update is enable.
#
# Usage:
#   $ dna update [OPTIONS]
#
# Options:
#   -y, --yes              Auto update DNA without confirmation
#   --status               Show update information and exit
#   --toggle-auto          Enable/disable daily auto-update capability
#   --help, -h             Show this help message
#
# About auto-update:
#   - Perform daily auto-update check on the first daily use of `dna` command (except on dna [help|version|update])
#   - If auto-update is enable: performs update automatically
#   - If auto-update is disable: warns user and asks for confirmation when update is available
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

# ::::Helper functions:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dna::update_fetch_remote_latest_version() {
    # Fetch remote tags and get the latest version
    local latest_remote_version
    
    cd "${DNA_ROOT}" || return 1
    
    # Fetch remote tags
    git fetch --tags origin >/dev/null 2>&1 || {
        n2st::print_msg_error_and_exit "Failed to fetch remote tags from origin"
    }
    
    # Get latest version tag (assuming semantic versioning)
    latest_remote_version=$(git tag -l | grep -E '^v?[0-9]+\.[0-9]+\.[0-9]+' | sort -V | tail -n1)
    
    if [[ -z "${latest_remote_version}" ]]; then
        n2st::print_msg_error_and_exit "Could not determine latest remote version"
    fi
    
    # Remove 'v' prefix if present for comparison
    latest_remote_version="${latest_remote_version#v}"
    echo "${latest_remote_version}"
}

function dna::update_is_remote_newer() {
    # Check if remote version is newer than local version
    # Returns: 0 (success) if remote is newer, 1 (failure) if not newer
    local local_version="$1"
    local remote_version="$2"
    
    # If versions are equal, remote is not newer
    if [[ "${local_version}" == "${remote_version}" ]]; then
        return 1
    fi
    
    # Use sort -V to compare semantic versions
    # The newer version will be last when sorted
    local newer_version
    newer_version=$(printf '%s\n' "${local_version}" "${remote_version}" | sort -V | tail -n1)
    
    # If remote version is the newer one, return success
    if [[ "${newer_version}" == "${remote_version}" ]]; then
        return 0
    else
        return 1
    fi
}

function dna::update_get_auto_update_setting() {
    # Check DNA_AUTO_UPDATE value in .env.dockerized-norlab-project.local
    local env_file="${DNA_ROOT:?err}/.env.dockerized-norlab-project.local"
    local auto_update_value
    
    if [[ -f "${env_file}" ]]; then
        # Extract DNA_AUTO_UPDATE value from env file
        auto_update_value=$(grep "^DNA_AUTO_UPDATE=" "${env_file}" 2>/dev/null | cut -d'=' -f2 | tr -d '"' | tr -d "'")
    fi
    
    echo "${auto_update_value:-false}"
}

function dna::update_toggle_auto_update_setting() {
    # Toggle DNA_AUTO_UPDATE between true/false in .env.dockerized-norlab-project.local
    local env_file="${DNA_ROOT:?err}/.env.dockerized-norlab-project.local"
    local current_value
    local new_value
    
    # Get current value
    current_value=$(dna::update_get_auto_update_setting)

    # Show current value
    n2st::print_msg "Current DNA_AUTO_UPDATE: ${current_value}"

    # Determine new value (toggle)
    if [[ "${current_value}" == "true" ]]; then
        new_value="false"
    else
        new_value="true"
    fi

    # Apply the toggle
    if [[ -f "${env_file}" ]]; then
        # Check if DNA_AUTO_UPDATE already exists
        if grep -q "^DNA_AUTO_UPDATE=" "${env_file}"; then
            # Update existing setting
            n2st::seek_and_modify_string_in_file "^DNA_AUTO_UPDATE=.*" "DNA_AUTO_UPDATE=${new_value}" "${env_file}"
        else
            # Add new setting
            echo "DNA_AUTO_UPDATE=${new_value}" >> "${env_file}"
        fi
    else
        # Create new file
        sudo touch "${env_file}"
        echo "DNA_AUTO_UPDATE=${new_value}" > "${env_file}"
    fi

    n2st::print_msg "DNA_AUTO_UPDATE toggled to ${new_value} in ${env_file}"
}

function dna::update_perform_update() {
    # Perform the actual update
    cd "${DNA_ROOT}" || return 1
    
    n2st::print_msg "Updating DNA repository..."
    
    if git pull origin >/dev/null 2>&1; then
        n2st::print_msg "DNA successfully updated to latest version"
        return 0
    else
        n2st::print_msg_error_and_exit "Failed to update DNA repository"
    fi
}

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dna::update_command() {
    local auto_yes=false
    local toggle_auto=false
    local status=false

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dna::command_help_menu "${DOCUMENTATION_BUFFER_UPDATE:?err}"
                exit 0
                ;;
            -y|--yes)
                auto_yes=true
                shift
                ;;
            --status)
                status=true
                shift
                ;;
            --toggle-auto)
                toggle_auto=true
                shift
                ;;
            *)
                dna::unknown_option_msg "update" "$*"
                return 1
                ;;
        esac
    done

    # Handle --toggle-auto flag
    if [[ "${toggle_auto}" == true ]]; then
        dna::update_toggle_auto_update_setting || n2st::print_msg_error_and_exit "Unable to toggle auto-update! Might require sudo."
        return 0
    fi

    # Get current and remote versions
    local current_version="${DNA_VERSION:?err}"
    local remote_version

    if [[ ${status} == false ]]; then
      n2st::print_msg "Checking for DNA updates..."
    fi

    remote_version=$(dna::update_fetch_remote_latest_version)

    # Check if remote version is newer
    local msg
    if dna::update_is_remote_newer "${current_version}" "${remote_version}"; then
        # Remote version is newer - update available
        msg="Update available: ${current_version} → ${remote_version}"
    else
        # Remote version is not newer (equal or local is newer)
        msg="Already up to date (version ${current_version})"
    fi
    if [[ ${status} == true ]]; then
        #dna::help_header
        #n2st::print_msg "Update status"
        echo "Dockerized-NorLab project application:"
        echo "   $msg"
        echo "   Auto-update: $(dna::update_get_auto_update_setting)"
        #dna::help_footer
        return 0
    else
        n2st::print_msg "$msg"
        if ! dna::update_is_remote_newer "${current_version}" "${remote_version}"; then
          return 0
        fi
    fi

    # Check auto-update behavior
    if [[ "${auto_yes}" == true ]]; then
        # Force update with --yes flag
        dna::update_perform_update
    else
        # Check DNA_AUTO_UPDATE setting
        local auto_update_setting
        auto_update_setting=$(dna::update_get_auto_update_setting)
        
        if [[ "${auto_update_setting}" == "true" ]]; then
            # Auto-update enabled
            n2st::print_msg "Auto-update enabled, updating DNA..."
            dna::update_perform_update
        else
            # Ask user for confirmation
            n2st::print_msg "Would you like to update DNA now? [y/N]"
            read -n 1 -r response
            echo
            if [[ ${response} =~ ^(y|Y)$ ]]; then
              dna::update_perform_update
            else
              n2st::print_msg "DNA update skipped. Run 'dna update --yes' to update without confirmation."
              return 0
            fi
        fi
    fi
    
    return 0
}
