#!/bin/bash
# =================================================================================================
# DNA project update helper script providing functions for managing repository updates and versioning.
#
# This script contains utility functions for the DNA project's update system, including version
# comparison, branch management, auto-update configuration, and repository synchronization.
# Functions handle semantic versioning, remote repository interactions, and configuration file
# management for automated updates.
#
# Usage:
#   $ source update_helper.bash
#
# Globals:
#   Read DNA_ROOT: Root directory path of the DNA project repository
#
# Dependencies:
#   - N2ST library functions (n2st::print_msg, n2st::print_msg_error, n2st::seek_and_modify_string_in_file)
#   - Git command line tool
#   - Standard Unix utilities (grep, sort, cut, tr)
#
# Returns:
#   1 if N2ST library is not loaded or DNA_ROOT is not set
# =================================================================================================

# Note: most fct are indirectly tested via 'test_update.bats' and 'test_dna.bats' for now
# (NICE TO HAVE) ToDo: implement missing dedicated unit-tests in test_update_helper.bats (ref task NMO-785)
#   - [ ] implement direct unit-test ->  function dna::update_determine_latest_release_branch()
#   - [ ] implement direct unit-test ->  function dna::update_fetch_remote_latest_version()
#   - [x] implement direct unit-test ->  function dna::update_is_remote_newer()
#   - [x] implement direct unit-test ->  function dna::update_get_auto_update_setting()
#   - [x] implement direct unit-test ->  function dna::update_get_auto_update_prerelease_setting()
#   - [ ] implement direct unit-test ->  function dna::update_toggle_auto_update_setting()
#   - [ ] implement direct unit-test ->  function dna::update_perform_update()
#   - [x] implement direct unit-test ->  function dna::should_run_daily_update()
#   - [x] implement direct unit-test ->  function dna::update_timestamp()

# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dna_error_prefix="\033[1;31m[dna error]\033[0m"
test -n "$( declare -f n2st::print_msg )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
test -d "${DNA_ROOT:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }

# ::::Update helper functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

# =================================================================================================
# Determines which branch (main or beta) has the latest release version.
#
# Fetches all remote branches and tags from origin, then compares the latest semantic version tags
# from both main and beta branches to determine which branch contains the most recent release.
#
# Usage:
#   $ latest_branch=$(dna::update_determine_latest_release_branch)
#
# Globals:
#   Read DNA_ROOT: The root directory of the DNA project
#
# Outputs:
#   Writes to stdout: "main" or "beta" (the branch with latest release)
#
# Returns:
#   0 on success
#   1 if failed to change directory or fetch from remote
# =================================================================================================
function dna::update_determine_latest_release_branch() {
    # Determine which branch (main or beta) has the latest release
    # Returns: "main" or "beta"

    # ....Setup....................................................................................
    local main_version
    local beta_version
    local tmp_cwd
    tmp_cwd=$(pwd)

    cd "${DNA_ROOT:?err}" || return 1

    # ....Begin....................................................................................
    # Fetch all remote branches and tags
    git fetch --tags origin >/dev/null 2>&1 || {
        n2st::print_msg_error "Failed to fetch remote branches and tags from origin";
        cd "${tmp_cwd}";
        return 1;
    }

    # Get latest version from main branch
    main_version=$(git tag -l --merged origin/main 2>/dev/null | grep -E '^v?[0-9]+\.[0-9]+\.[0-9]+$' | sort -V | tail -n1)

    # Get latest version from beta branch (including beta tags)
    beta_version=$(git tag -l --merged origin/beta 2>/dev/null | grep -E '^v?[0-9]+\.[0-9]+\.[0-9]+(\-beta\.[0-9]+)?$' | sort -V | tail -n1)

    cd "${tmp_cwd}" || { n2st::print_msg_error "Return to original dir error"; return 1; }

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

# =================================================================================================
# Fetches the latest version tag from the specified remote branch.
#
# Determines and returns the latest semantic version tag from the remote repository. The function
# can target specific branches (main, beta) or auto-determine the branch with the latest release.
# It fetches remote tags and filters them based on semantic versioning patterns, returning the
# highest version found.
#
# Usage:
#   $ dna::update_fetch_remote_latest_version [main|beta|auto]
#
# Positional arguments:
#   target branch      Target branch to fetch version from (Optional).
#                      Options: "main", "beta", or "auto".
#                      Defaults to "auto" which auto-determines the branch with the latest release.
#
# Globals:
#   Read DNA_ROOT: Directory path to the DNA repository root
#
# Outputs:
#   Writes the latest version number (without 'v' prefix) to stdout
#   Writes error messages to stderr via n2st::print_msg_error
#
# Returns:
#   0 on success, 1 on failure (git fetch failure or no version found)
#
# =================================================================================================
function dna::update_fetch_remote_latest_version() {
    # Fetch remote tags and get the latest version from the appropriate branch
    # If --include-prerelease flag is used, get latest from both branches, otherwise only main
    local target_branch="${1:-auto}"
    local latest_remote_version
    local tmp_cwd
    tmp_cwd=$(pwd)

    cd "${DNA_ROOT:?err}" || return 1

    # Fetch remote tags and branches
    git fetch --tags origin >/dev/null 2>&1 || {
        n2st::print_msg_error "Failed to fetch remote tags from origin";
        cd "${tmp_cwd}";
        return 1;
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

    cd "${tmp_cwd}" || { n2st::print_msg_error "Return to original dir error"; return 1; }

    if [[ -z "${latest_remote_version}" ]]; then
        n2st::print_msg_error "Could not determine latest remote version for branch: ${target_branch}"
        return 1
    fi

    # Remove 'v' prefix if present for comparison
    latest_remote_version="${latest_remote_version#v}"
    echo "${latest_remote_version}"
    return 0
}

# =================================================================================================
# Compares two semantic version strings to determine if remote version is newer than local.
#
# This function uses version sort (-V) to compare semantic versions and returns success (0) if the
# remote version is newer than the local version, or failure (1) if versions are equal or local
# is newer.
#
# Usage:
#   $ dna::update_is_remote_newer "<LOCAL_VERSION>" "<LATEST_REMOTE_VERSION>"
#   $ dna::update_is_remote_newer "1.2.3" "1.3.0"
#
# Positional arguments:
#   LOCAL_VERSION              The current local version string
#   LATEST_REMOTE_VERSION      The remote latest version string to compare against
#
# Returns:
#   0    Remote version is newer than local version
#   1    Remote version is not newer (equal or older) than local version
#
# =================================================================================================
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

#======================================================================================================
# Retrieves the auto-update setting from the DNA project configuration file.
#
# Reads the DNA_AUTO_UPDATE value from .env.dockerized-norlab-project.local file and returns it.
# If the file doesn't exist or the variable is not set, returns 'false' as the default value.
#
# Usage:
#   $ auto_update=$(dna::update_get_auto_update_setting)
#   $ if [[ "$(dna::update_get_auto_update_setting)" == "true" ]]; then echo "Auto-update enabled"; fi
#
# Globals:
#   Read DNA_ROOT: Root directory path of the DNA project
#
# Outputs:
#   Writes the auto-update setting value to stdout ('true', 'false', or custom value)
#
# Returns:
#   0: Always successful
#
#======================================================================================================
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

# =================================================================================================
# Get the auto update prerelease setting from the DNA configuration file.
#
# Reads the DNA_INCLUDE_PRERELEASE value from the .env.dockerized-norlab-project.local file
# and returns it. If the file doesn't exist or the variable is not set, returns "false".
#
# Usage:
#   $ setting=$(dna::update_get_auto_update_prerelease_setting)
#
# Globals:
#   Read DNA_ROOT: The root directory of the DNA project
#
# Outputs:
#   stdout: The value of DNA_INCLUDE_PRERELEASE (true/false), defaults to "false"
#
# Returns:
#   0: Always successful
#
# =================================================================================================
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

# =================================================================================================
# Toggles DNA auto-update or prerelease setting between true/false.
#
# Toggles the DNA_AUTO_UPDATE or DNA_INCLUDE_PRERELEASE setting in the
# .env.dockerized-norlab-project.local file. If the file doesn't exist, it will be created.
# If the setting doesn't exist in the file, it will be added.
#
# Usage:
#   $ dna::update_toggle_auto_update_setting [regular|prerelease]
#
# Positional arguments:
#   Setting type     "regular" toggles DNA_AUTO_UPDATE, "prerelease" to toggle DNA_INCLUDE_PRERELEASE
#                    (default: "regular")
#
# Globals:
#   Read DNA_ROOT: Root directory path for DNA project
#
# Outputs:
#   Writes current and new setting values to stdout via n2st::print_msg
#   Write to .env.dockerized-norlab-project.local
#
# Returns:
#   0 on success
# =================================================================================================
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
    return 0
}

# =================================================================================================
# Performs DNA repository update by checking out and pulling from target branch.
#
# Updates the DNA repository to the latest version from a specified branch or automatically
# determines the latest release branch. Changes to DNA_ROOT directory and performs git operations
# to sync with remote repository.
#
# Usage:
#   $ dna::update_perform_update [beta|main|auto]
#
# Positional arguments:
#   target branch       (optional) Branch to update from ("beta", "main", or "auto")
#                       Defaults to "auto" if not specified
#
# Globals:
#   Read DNA_ROOT: Root directory path of the DNA repository
#
# Outputs:
#   Writes status messages to stdout via n2st::print_msg
#   Writes error messages to stderr via n2st::print_msg_error
#
# Returns:
#   0: Update completed successfully
#   1: Update failed
# =================================================================================================
function dna::update_perform_update() {
    # Perform the actual update with branch checkout
    # Parameters: $1 = target_branch (optional: "beta", "main", or "auto")
    local target_branch="${1:-auto}"
    local checkout_branch
    local tmp_cwd
    local git_output
    tmp_cwd=$(pwd)

    cd "${DNA_ROOT:?err}" || return 1

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

    # Pre-flight check: warn (but do not fail) if working tree is dirty, as this
    # is a common cause of checkout/pull failures on CI servers.
    if ! git diff --quiet HEAD -- 2>/dev/null || ! git diff --cached --quiet 2>/dev/null; then
        n2st::print_msg_warning "DNA repository at '${DNA_ROOT}' has uncommitted local changes. This may cause the update to fail. Current status:"
        git status --short 1>&2 || true
    fi

    # Checkout the target branch. Capture combined git output so that, on failure,
    # the actual git error is surfaced to the user for diagnostic purposes
    # (ref: NMO "Failed to update DNA repository from branch" issue on CI servers).
    if ! git_output=$(git checkout "${checkout_branch}" 2>&1); then
        n2st::print_msg_error "Failed to checkout branch: ${checkout_branch}"
        printf '%s\n' "${git_output}" 1>&2
        cd "${tmp_cwd}" || { n2st::print_msg_error "Return to original dir error"; return 1; }
        return 1
    fi

    # Pull the latest changes (capture output for diagnostics on failure)
    if git_output=$(git pull --recurse-submodules origin "${checkout_branch}" 2>&1); then
        n2st::print_msg "DNA successfully updated to latest version from '${checkout_branch}' branch"
        cd "${tmp_cwd}" || { n2st::print_msg_error "Return to original dir error"; return 1; }
        return 0
    else
        n2st::print_msg_error "Failed to update DNA repository from branch: ${checkout_branch}"
        printf '%s\n' "${git_output}" 1>&2
        cd "${tmp_cwd}" || { n2st::print_msg_error "Return to original dir error"; return 1; }
        return 1
    fi
}

# ::::Auto-update helper functions:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

# =================================================================================================
# Determines if the daily update check should run based on timestamp comparison.
#
# Checks if an update check has already been performed today by comparing the current date
# with the date stored in a timestamp file. Creates the timestamp file if it doesn't exist.
#
# Usage:
#   $ if dna::should_run_daily_update; then echo "Should update"; fi
#
# Returns:
#   0 if daily update should run (no check today or file doesn't exist)
#   1 if daily update should not run (already checked today)
# =================================================================================================
function dna::should_run_daily_update() {
    # Check if daily update should run based on timestamp
    local timestamp_file="/tmp/.dna_last_update_check"
    local current_date
    local last_check_date

    current_date=$(date +%Y-%m-%d)

    if [[ -f "${timestamp_file}" ]]; then
        last_check_date=$(cat "${timestamp_file}" 2>/dev/null)
        if [[ "${last_check_date}" == "${current_date}" ]]; then
            return 1  # Already updated today
        fi
    fi

    return 0  # Should update
}

# =================================================================================================
# Update the timestamp file with current date.
#
# Creates or overwrites a timestamp file containing the current date in YYYY-MM-DD format. This
# function is typically used to track the last update check for the DNA system.
#
# Usage:
#   $ dna::update_timestamp
#
# Outputs:
#   Writes current date to /tmp/.dna_last_update_check file
#
# Returns:
#   0 on success
# =================================================================================================
function dna::update_timestamp() {
    # Update the timestamp file with current date
    local timestamp_file="/tmp/.dna_last_update_check"
    local current_date

    current_date=$(date +%Y-%m-%d)
    echo "${current_date}" > "${timestamp_file}"
    return 0
}
