#!/bin/bash
# lib/commands/version.bash

DOCUMENTATION_BUFFER_VERSION=$( cat <<'EOF'
# =================================================================================================
# Show Dockerized-NorLab project application version
#
# Usage:
#   $ dna version [OPTIONS]
#
# Options:
#   --short, -s            Show version number only
#   --all, -a              Show detailed version information
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
    local verbose=default # options: 'default', 'short' or 'all'

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dna::command_help_menu "${DOCUMENTATION_BUFFER_VERSION:?err}"
                exit 0
                ;;
            -s|--short)
                verbose="short"
                shift
                ;;
            -a|--all)
                verbose="all"
                shift
                ;;
            *)
                dna::unknown_option_msg "version" "$*"
                return 1
                ;;
        esac
    done

    local current_branch
    current_branch=$(cd "${DNA_ROOT:?err}" && git branch --show-current 2>/dev/null || echo "unknown")

    local current_commit
    current_commit=$(cd "${DNA_ROOT}" && git rev-parse HEAD 2>/dev/null || echo "unknown")

    {
        if [[ "${verbose}" == "default"  ]]; then
          echo "${DNA_HUMAN_NAME:?err} version ${DNA_VERSION:?err}"
        elif [[ "${verbose}" == "short"  ]]; then
          echo "${DNA_VERSION:?err}"
        elif [[ "${verbose}" == "all"  ]]; then
          n2st::set_which_architecture_and_os
          echo -en "${DNA_HUMAN_NAME:?err}:
  Version: ${DNA_VERSION:?err}
  Config scheme version: ${DNA_CONFIG_SCHEME_VERSION:?err}
  Submodule version:
    norlab-shell-script-tools: ${N2ST_VERSION:?err}
    norlab-build-system: ${NBS_VERSION:?err}
  Local repository:
    Current branch: ${current_branch}
    Current commit: ${current_commit}
  Host architecture and OS: ${IMAGE_ARCH_AND_OS:?err}
"
        else
          n2st::print_msg_error_and_exit "Could not determine dna version."
        fi
    } || n2st::print_msg_error_and_exit "Could not determine dna version."

    return 0
}
