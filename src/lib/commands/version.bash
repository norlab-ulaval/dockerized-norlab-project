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

test -d "${DNP_ROOT:?err}" || { echo "The DNP lib load error!" ; exit 1 ; }
test -d "${DNP_LIB_PATH:?err}" || { echo "The DNP lib load error!" ; exit 1 ; }

function dnp::version() {
    local remaining_args=()

    # Parse options
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_VERSION}"
                ;;
            *)
                remaining_args+=("$@")
                break
                ;;
        esac
    done

#    # Import DNP lib to get access to DNP_ROOT
#    source "${DNP_LIB_PATH}/core/utils/import_dnp_lib.bash"

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

