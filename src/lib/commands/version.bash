#!/bin/bash
# lib/commands/version.bash

DOCUMENTATION_BUFFER_VERSION=$( cat <<'EOF'
# =================================================================================================
# Show Dockerized-NorLab Project version
#
# Usage:
#   $ dna version [OPTIONS]
#
# Options:
#   --help, -h             Show this help message
#
# =================================================================================================
EOF
)

# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dna_error_prefix="\033[1;31m[DNA error]\033[0m"
test -n "$( declare -f dna::import_lib_and_dependencies )" || { echo -e "${dna_error_prefix} The DNA lib is not loaded!" 1>&2 && exit 1; }
test -n "$( declare -f n2st::print_msg )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
test -d "${DNA_ROOT:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }
test -d "${DNA_LIB_PATH:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dna::version_command() {
    local remaining_args=()

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dna::command_help_menu "${DOCUMENTATION_BUFFER_VERSION:?err}"
                exit 0
                ;;
            *)
                remaining_args+=("$@")
                break
                ;;
        esac
    done

    # Read version from version.txt
    if [[ -f "${DNA_ROOT}/version.txt" ]]; then
        local VERSION="$(cat "${DNA_ROOT}/version.txt")"
        echo "Dockerized-NorLab Project version: ${VERSION}"
    else
        echo "Error: version.txt not found." >&2
        echo "Could not determine Dockerized-NorLab Project version." >&2
        exit 1
    fi

    return 0
}

