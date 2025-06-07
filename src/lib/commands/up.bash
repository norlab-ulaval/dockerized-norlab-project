#!/bin/bash
# lib/commands/up.bash

DOCUMENTATION_BUFFER_UP=$( cat <<'EOF'
# =================================================================================================
# Start and attach to a containers for the project
# Notes:
# - spin a specific service from 'docker-compose.project.run.<DEVICE>.yaml'
# - service are started in detach mode
# - architecture specific docker-compose config are automaticaly selected at runtime
#
# Usage:
#   $ dnp up [--service <theService>] [--] [<command&arguments>]
#
# Options:
#   --service service      Specify the service to start and to attach (default: project-develop)
#   --help, -h             Show this help message
#
# Positional argument:
#   <command&arguments>      Any command to be executed inside the docker container (default: bash)
#
# =================================================================================================
EOF
)

test -d "${DNP_ROOT:?err}" || { echo "The DNP lib load error!" ; exit 1 ; }
test -d "${DNP_LIB_PATH:?err}" || { echo "The DNP lib load error!" ; exit 1 ; }

function dnp::up() {
    local remaining_args=()

    # Parse options
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_UP}"
                ;;
            *)
                remaining_args+=("$@")
                break
                ;;
        esac
    done

    # Load super project configuration
    source "${DNP_LIB_PATH}/core/utils/load_super_project_config.bash"

    echo "Starting containers..."
    source "${DNP_LIB_PATH}/core/execute/up_and_attach.bash" "${remaining_args[@]}"

    return 0
}

