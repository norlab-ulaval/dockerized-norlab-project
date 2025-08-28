#!/bin/bash

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

# ::::Setup::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
source "${DNA_LIB_PATH:?err}/core/utils/update_helper.bash" || exit 1

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dna::update_command() {
    local force_update=false
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
                force_update=true
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
            dna::update_toggle_auto_update_setting "prerelease" || {
              n2st::print_msg_error "Unable to toggle auto-update prerelease! Might require sudo.";
              return 1;
              }
        else
            dna::update_toggle_auto_update_setting || {
              n2st::print_msg_error "Unable to toggle auto-update! Might require sudo.";
              return 1;
              }
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
    fi

    if [[ "${force_update}" == true ]]; then
        # Force update with --yes flag
        dna::update_perform_update "${target_branch}"
    elif ! dna::update_is_remote_newer "${current_version}" "${latest_remote_version}"; then
        # No update available
        return 0
    else
        # Check auto-update behavior
        local auto_update_setting
        auto_update_setting=$(dna::update_get_auto_update_setting)

        # Check DNA_AUTO_UPDATE and DNA_INCLUDE_PRERELEASE settings
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
