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
#   --include-prerelease   Consider both main and beta branches for updates
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
function dna::update_determine_latest_release_branch() {
    # Determine which branch (main or beta) has the latest release
    # Returns: "main" or "beta"
    local main_version
    local beta_version
    
    cd "${DNA_ROOT}" || return 1
    
    # Fetch all remote branches and tags
    git fetch --tags origin >/dev/null 2>&1 || {
        n2st::print_msg_error_and_exit "Failed to fetch remote branches and tags from origin"
    }
    
    # Get latest version from main branch
    main_version=$(git tag -l --merged origin/main 2>/dev/null | grep -E '^v?[0-9]+\.[0-9]+\.[0-9]+$' | sort -V | tail -n1)
    
    # Get latest version from beta branch (including beta tags)
    beta_version=$(git tag -l --merged origin/beta 2>/dev/null | grep -E '^v?[0-9]+\.[0-9]+\.[0-9]+(\-beta\.[0-9]+)?$' | sort -V | tail -n1)
    
    # Remove 'v' prefix if present for comparison
    main_version="${main_version#v}"
    beta_version="${beta_version#v}"
    
    # If no versions found, default to main
    if [[ -z "${main_version}" && -z "${beta_version}" ]]; then
        echo "main"
        return 0
    elif [[ -z "${main_version}" ]]; then
        echo "beta"
        return 0
    elif [[ -z "${beta_version}" ]]; then
        echo "main"
        return 0
    fi
    
    # Compare versions to determine which branch has the latest release
    local newer_version
    newer_version=$(printf '%s\n' "${main_version}" "${beta_version}" | sort -V | tail -n1)
    
    if [[ "${newer_version}" == "${beta_version}" ]]; then
        echo "beta"
    else
        echo "main"
    fi
    return 0
}

function dna::update_fetch_remote_latest_version() {
    # Fetch remote tags and get the latest version from the appropriate branch
    # If --include-prerelease flag is used, get latest from both branches, otherwise only main
    local target_branch="${1:-auto}"
    local latest_remote_version
    
    cd "${DNA_ROOT}" || return 1
    
    # Fetch remote tags and branches
    git fetch --tags origin >/dev/null 2>&1 || {
        n2st::print_msg_error_and_exit "Failed to fetch remote tags from origin"
    }
    
    if [[ "${target_branch}" == "beta" ]]; then
        # Get latest version from beta branch (including beta tags)
        latest_remote_version=$(git tag -l --merged origin/beta 2>/dev/null | grep -E '^v?[0-9]+\.[0-9]+\.[0-9]+(\-beta\.[0-9]+)?$' | sort -V | tail -n1)
    elif [[ "${target_branch}" == "main" ]]; then
        # Get latest version from main branch
        latest_remote_version=$(git tag -l --merged origin/main 2>/dev/null | grep -E '^v?[0-9]+\.[0-9]+\.[0-9]+$' | sort -V | tail -n1)
    else
        # Auto-determine which branch has the latest release
        local release_branch
        release_branch=$(dna::update_determine_latest_release_branch)
        
        if [[ "${release_branch}" == "beta" ]]; then
            latest_remote_version=$(git tag -l --merged origin/beta 2>/dev/null | grep -E '^v?[0-9]+\.[0-9]+\.[0-9]+(\-beta\.[0-9]+)?$' | sort -V | tail -n1)
        else
            latest_remote_version=$(git tag -l --merged origin/main 2>/dev/null | grep -E '^v?[0-9]+\.[0-9]+\.[0-9]+$' | sort -V | tail -n1)
        fi
    fi
    
    if [[ -z "${latest_remote_version}" ]]; then
        n2st::print_msg_error_and_exit "Could not determine latest remote version for branch: ${target_branch}"
    fi
    
    # Remove 'v' prefix if present for comparison
    latest_remote_version="${latest_remote_version#v}"
    echo "${latest_remote_version}"
}

function dna::update_is_remote_newer() {
    # Check if remote version is newer than local version
    # Returns: 0 (success) if remote is newer, 1 (failure) if not newer
    local local_version="$1"
    local latest_remote_version="$2"
    
    # If versions are equal, remote is not newer
    if [[ "${local_version}" == "${latest_remote_version}" ]]; then
        return 1
    fi
    
    # Use sort -V to compare semantic versions
    # The newer version will be last when sorted
    local newer_version
    newer_version=$(printf '%s\n' "${local_version}" "${latest_remote_version}" | sort -V | tail -n1)
    
    # If remote version is the newer one, return success
    if [[ "${newer_version}" == "${latest_remote_version}" ]]; then
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

function dna::update_get_auto_update_prerelease_setting() {
    # Check DNA_INCLUDE_PRERELEASE value in .env.dockerized-norlab-project.local
    local env_file="${DNA_ROOT:?err}/.env.dockerized-norlab-project.local"
    local auto_update_prerelease_value
    
    if [[ -f "${env_file}" ]]; then
        # Extract DNA_INCLUDE_PRERELEASE value from env file
        auto_update_prerelease_value=$(grep "^DNA_INCLUDE_PRERELEASE=" "${env_file}" 2>/dev/null | cut -d'=' -f2 | tr -d '"' | tr -d "'")
    fi
    
    echo "${auto_update_prerelease_value:-false}"
}

function dna::update_toggle_auto_update_setting() {
    # Toggle DNA_AUTO_UPDATE or DNA_INCLUDE_PRERELEASE between true/false in .env.dockerized-norlab-project.local
    # Parameters: $1 = "prerelease" to toggle prerelease setting, otherwise toggle regular setting
    local env_file="${DNA_ROOT:?err}/.env.dockerized-norlab-project.local"
    local current_value
    local new_value
    local setting_type="${1:-regular}"
    local setting_name
    local setting_description
    
    if [[ "${setting_type}" == "prerelease" ]]; then
        setting_name="DNA_INCLUDE_PRERELEASE"
        setting_description="DNA_INCLUDE_PRERELEASE"
        current_value=$(dna::update_get_auto_update_prerelease_setting)
    else
        setting_name="DNA_AUTO_UPDATE"
        setting_description="DNA_AUTO_UPDATE"
        current_value=$(dna::update_get_auto_update_setting)
    fi

    # Show current value
    n2st::print_msg "Current ${setting_description}: ${current_value}"

    # Determine new value (toggle)
    if [[ "${current_value}" == "true" ]]; then
        new_value="false"
    else
        new_value="true"
    fi

    # Apply the toggle
    if [[ -f "${env_file}" ]]; then
        # Check if setting already exists
        if grep -q "^${setting_name}=" "${env_file}"; then
            # Update existing setting
            n2st::seek_and_modify_string_in_file "^${setting_name}=.*" "${setting_name}=${new_value}" "${env_file}"
        else
            # Add new setting
            echo "${setting_name}=${new_value}" >> "${env_file}"
        fi
    else
        # Create new file
        sudo touch "${env_file}"
        echo "${setting_name}=${new_value}" > "${env_file}"
    fi

    n2st::print_msg "${setting_description} toggled to ${new_value} in ${env_file}"
}

function dna::update_perform_update() {
    # Perform the actual update with branch checkout
    # Parameters: $1 = target_branch (optional: "beta", "main", or "auto")
    local target_branch="${1:-auto}"
    local checkout_branch
    
    cd "${DNA_ROOT}" || return 1
    
    # Determine which branch to checkout
    if [[ "${target_branch}" == "beta" ]]; then
        checkout_branch="beta"
    elif [[ "${target_branch}" == "main" ]]; then
        checkout_branch="main"
    else
        # Auto-determine the branch with latest release
        checkout_branch=$(dna::update_determine_latest_release_branch)
    fi
    
    n2st::print_msg "Updating DNA repository to latest release from '${checkout_branch}' branch..."
    
    # Checkout the target branch
    if ! git checkout "${checkout_branch}" >/dev/null 2>&1; then
        n2st::print_msg_error_and_exit "Failed to checkout branch: ${checkout_branch}"
    fi
    
    # Pull the latest changes
    if git pull --recurse-submodules origin "${checkout_branch}" >/dev/null 2>&1; then
        n2st::print_msg "DNA successfully updated to latest version from '${checkout_branch}' branch"
        return 0
    else
        n2st::print_msg_error_and_exit "Failed to update DNA repository from branch: ${checkout_branch}"
    fi
}

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dna::update_command() {
    local auto_yes=false
    local toggle_auto=false
    local status=false
    local include_prerelease_flag=false

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
            --include-prerelease)
                include_prerelease_flag=true
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
        if [[ "${include_prerelease_flag}" == true ]]; then
            dna::update_toggle_auto_update_setting "prerelease" || n2st::print_msg_error_and_exit "Unable to toggle auto-update prerelease! Might require sudo."
        else
            dna::update_toggle_auto_update_setting || n2st::print_msg_error_and_exit "Unable to toggle auto-update! Might require sudo."
        fi
        return 0
    fi

    # Determine target branch based on --include-prerelease flag or DNA_INCLUDE_PRERELEASE setting
    local target_branch="main"
    local auto_update_prerelease_setting
    auto_update_prerelease_setting=$(dna::update_get_auto_update_prerelease_setting)
    
    if [[ "${include_prerelease_flag}" == true ]] || [[ "${auto_update_prerelease_setting}" == "true" ]]; then
        target_branch="auto"
    fi

    # Get current and remote versions
    local current_version="${DNA_VERSION:?err}"
    local latest_remote_version

    if [[ ${status} == false ]]; then
      n2st::print_msg "Checking for DNA updates..."
    fi

    latest_remote_version=$(dna::update_fetch_remote_latest_version "${target_branch}")

    # Check if remote version is newer
    local msg
    if dna::update_is_remote_newer "${current_version}" "${latest_remote_version}"; then
        # Remote version is newer - update available
        msg="Update available: ${current_version} → ${latest_remote_version}"
    else
        # Remote version is not newer (equal or local is newer)
        msg="Already up to date (version ${current_version})"
    fi
    if [[ ${status} == true ]]; then
        #dna::help_header
        #n2st::print_msg "Update status"
        echo "Dockerized-NorLab project application:"
        echo "   $msg"
        echo "   Include pre-release: ${auto_update_prerelease_setting}"
        echo "   Auto-update: $(dna::update_get_auto_update_setting)"
        #dna::help_footer
        return 0
    else
        n2st::print_msg "$msg"
        if ! dna::update_is_remote_newer "${current_version}" "${latest_remote_version}"; then
          return 0
        fi
    fi

    # Check auto-update behavior
    if [[ "${auto_yes}" == true ]]; then
        # Force update with --yes flag
        dna::update_perform_update "${target_branch}"
    else
        # Check DNA_AUTO_UPDATE and DNA_INCLUDE_PRERELEASE settings
        local auto_update_setting
        auto_update_setting=$(dna::update_get_auto_update_setting)
        
        if [[ "${auto_update_setting}" == "true" ]] || [[ "${auto_update_prerelease_setting}" == "true" ]]; then
            # Auto-update enabled (either regular or prerelease)
            n2st::print_msg "Auto-update enabled, updating DNA..."
            dna::update_perform_update "${target_branch}"
        else
            # Ask user for confirmation
            n2st::print_msg "Would you like to update DNA now? [y/N]"
            read -n 1 -r response
            echo
            if [[ ${response} =~ ^(y|Y)$ ]]; then
              dna::update_perform_update "${target_branch}"
            else
              n2st::print_msg "DNA update skipped. Run 'dna update --yes' to update without confirmation."
              return 0
            fi
        fi
    fi
    
    return 0
}
