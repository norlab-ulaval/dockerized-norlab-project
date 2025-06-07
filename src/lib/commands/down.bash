#!/bin/bash
# lib/commands/down.bash

DOCUMENTATION_BUFFER_DOWN=$( cat <<'EOF'
# =================================================================================================
# Stop containers for the project
#
# Usage:
#   $ dnp down [OPTIONS] [<any-docker-compose-down-flags>]
#
# Options:
#   --slurm                Stop project related slurm containers
#   --help, -h             Show this help message
#
# =================================================================================================
EOF
)

test -d "${DNP_ROOT:?err}" || { echo "The DNP lib load error!" ; exit 1 ; }
test -d "${DNP_LIB_PATH:?err}" || { echo "The DNP lib load error!" ; exit 1 ; }


function dnp::down() {
    local slurm=false
    local help=false
    local remaining_args=()

    # Parse options
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --slurm)
                slurm=true
                shift
                ;;
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_DOWN}"
                ;;
            *)
                remaining_args+=("$@")
                break
                ;;
        esac
    done

    # Load super project configuration
    source "${DNP_LIB_PATH}/core/utils/load_super_project_config.bash"

    # Determine which down script to execute
    if [[ "${slurm}" == true ]]; then
        echo "Stopping slurm containers..."
        source "${DNP_LIB_PATH}/core/execute/down.slurm.bash" "${remaining_args[@]}"
    else
        echo "Stopping containers..."
        source "${DNP_LIB_PATH}/core/execute/down.bash" "${remaining_args[@]}"
    fi

    return 0
}
