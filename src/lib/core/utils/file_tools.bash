#!/bin/bash
# =================================================================================================
# File related utility functions.
#
# Usage:
#   source file_tools.bash
#
# =================================================================================================

# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dna_error_prefix="\033[1;31m[dna error]\033[0m"
test -n "$( declare -f n2st::print_msg )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }

function dna::dimmed_rsync() {
    local DIM
    local RESET
    DIM=$(tput dim 2>/dev/null || echo -e "\033[2m")
    RESET=$(tput sgr0 2>/dev/null || echo -e "\033[0m")
    n2st::draw_horizontal_line_across_the_terminal_window "─" "${DIM}"
    rsync "$@" | sed "s/.*/${DIM}&${RESET}/" || return 1
}

function dna::portable_copy() {
    # Copy function using rsync with backup functionality
    # Arguments: source destination [super_project_root]
    local source="$1"
    local destination="$2"
    local super_project_root="${3:-$(pwd)}"

    # Use rsync with backup functionality (no --update flag to preserve existing files)
    local rsync_flags=()
    # rsync_flags+=(--progress)
    rsync_flags+=(--verbose)
    rsync_flags+=(--backup --suffix='.old')

    # Ensure parent directory exists
    mkdir -p "$(dirname "${destination}")"

    if [[ -d "${source}" ]]; then
        # For directories, ensure trailing slash for proper rsync behavior
        dna::dimmed_rsync "${rsync_flags[@]}" --recursive "${source%/}/" "${destination}"
    else
        # For files
        dna::dimmed_rsync "${rsync_flags[@]}" "${source}" "${destination}"
    fi
    echo

    # Validate file ownership and permissions match the super project
    dna::validate_file_ownership_and_permissions "${destination}" "${super_project_root}" || return 1

    # Optional: add to git if in a git repo
    if [[ -d ".git" ]]; then
        git add "${destination}" > /dev/null 2>&1 || true
    fi
    return 0
}

function dna::get_owner() {
    # Cross platform implementation
    if [[ "$(uname)" == "Darwin" ]]; then
        stat -f '%Su' "$1"
    else
        stat -c '%U' "$1"
    fi
}

function dna::get_group() {
    # Cross platform implementation
    if [[ "$(uname)" == "Darwin" ]]; then
        stat -f '%Sg' "$1"
    else
        stat -c '%G' "$1"
    fi
}

function dna::get_permission() {
    # Cross platform implementation
    if [[ "$(uname)" == "Darwin" ]]; then
        stat -f '%A' "$1"
    else
        stat -c '%a' "$1"
    fi
}

function dna::validate_file_ownership_and_permissions() {
    # Validate that copied files/directories have ownership and permissions matching the super project
    # Arguments: target_path super_project_root
    local target_path="$1"
    local super_project_root="$2"

    # Get super project ownership and permissions
    local super_project_owner
    local super_project_group

    super_project_owner=$(dna::get_owner "${super_project_root}" 2>/dev/null)
    super_project_group=$(dna::get_group "${super_project_root}" 2>/dev/null)

    # Recursively fix ownership and permissions for the target path
    if [[ -d "$target_path" ]]; then
        # For directories, apply to all contents
        find "$target_path" -type f -exec chown "${super_project_owner}:${super_project_group}" {} \; 2>/dev/null || true
        find "$target_path" -type d -exec chown "${super_project_owner}:${super_project_group}" {} \; 2>/dev/null || true
        find "$target_path" -type f -exec chmod 644 {} \; 2>/dev/null || true
        find "$target_path" -type d -exec chmod 755 {} \; 2>/dev/null || true
    else
        # For files
        chown "${super_project_owner}:${super_project_group}" "$target_path" 2>/dev/null || true
        chmod 644 "$target_path" 2>/dev/null || true
    fi

    return 0
}
