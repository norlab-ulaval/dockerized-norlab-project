#!/bin/bash
# lib/commands/version.bash

DOCUMENTATION_BUFFER_VERSION=$( cat <<'EOF'
# =================================================================================================
# Show Dockerized-NorLab-Project version
#
# Usage:
#   $ dnp version [OPTIONS]
#
# Options:
#   --help, -h             Show this help message
#
# =================================================================================================
EOF
)

# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
test -n "$( declare -F dnp::import_lib_and_dependencies )" || { echo -e "${dnp_error_prefix} The DNP lib is not loaded!" ; exit 1 ; }
test -n "$( declare -F n2st::print_msg )" || { echo -e "${dnp_error_prefix} The N2ST lib is not loaded!" ; exit 1 ; }
test -d "${DNP_ROOT:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }
test -d "${DNP_LIB_PATH:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dnp::version_command() {
    local remaining_args=()

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_VERSION}"
                exit 0
                ;;
            *)
                remaining_args+=("$@")
                break
                ;;
        esac
    done

    # Read version from version.txt
    if [[ -f "${DNP_ROOT}/version.txt" ]]; then
        local VERSION="$(cat "${DNP_ROOT}/version.txt")"
        echo "Dockerized-NorLab-Project version: ${VERSION}"
    else
        echo "Error: version.txt not found." >&2
        echo "Could not determine Dockerized-NorLab-Project version." >&2
        exit 1
    fi

    return 0
}

