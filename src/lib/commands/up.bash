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
                dnp::up_help
                exit 0
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

function dnp::up_help() {
    echo -e "${MSG_DIMMED_FORMAT}"
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "$0 up --help"
    # Strip shell comment char `#` and both lines
    echo -e "${DOCUMENTATION_BUFFER_UP}" | sed 's/\# ====.*//' | sed 's/^\#//'
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "${MSG_END_FORMAT}"
}
