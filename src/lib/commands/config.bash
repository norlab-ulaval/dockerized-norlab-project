#!/bin/bash
# lib/commands/config.bash

DOCUMENTATION_BUFFER_CONFIG=$( cat <<'EOF'
# =================================================================================================
# Show Docker Compose configuration
#
# Usage:
#   $ dnp config mode [platform]
#
# Modes:
#   dev                    Development mode
#   deploy                 Deployment mode
#   ci-tests               CI tests mode
#   slurm                  SLURM mode
#   release                Release mode
#
# Platforms (for dev mode):
#   darwin                 macOS
#   linux                  Linux
#   jetson                 NVIDIA Jetson
#
# Options:
#   --help, -h             Show this help message
#
# =================================================================================================
EOF
)

test -d "${DNP_ROOT:?err}" || { echo "The DNP lib load error!" ; exit 1 ; }
test -d "${DNP_LIB_PATH:?err}" || { echo "The DNP lib load error!" ; exit 1 ; }

function dnp::config() {
    local mode=""
    local platform=""
    local help=false

    declare -a remaining_args

    # Parse options
    while [[ $# -gt 0 ]]; do
        case "$1" in
            dev|deploy|ci-tests|slurm|release)
                mode="$1"
                shift
                ;;
            darwin|linux|jetson)
                platform="$1"
                shift
                ;;
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_CONFIG}"
                ;;
            *)
                remaining_args=("$@")
                break
                ;;
        esac
    done

    if [[ -z "${mode}" ]]; then
        echo "Error: Mode is required." >&2
        dnp::config_help
        exit 1
    fi

#    # Import DNP lib
#    source "${DNP_LIB_PATH:?err}/core/utils/import_dnp_lib.bash"

    # Load super project configuration
    source "${DNP_LIB_PATH}/core/utils/load_super_project_config.bash"


    # Determine which compose file to use
    local compose_file=""

    if [[ "${mode}" == "dev" ]]; then
        if [[ "${platform}" == "darwin" ]]; then
#            compose_file="docker-compose.project.run.darwin.yaml"
            compose_file="docker-compose.project.build.native.yaml"
            # (CRITICAL) ToDo: implement <-- we are here
            docker-compose --file "${DNP_LIB_PATH}/core/docker/${compose_file}" config project-core project-develop
            echo -e "\n${0}: breakpoint\n" && exit 1 # (CRITICAL) ToDo: on task end >> delete this line <--
        elif [[ "${platform}" == "jetson" ]]; then
            compose_file="docker-compose.project.run.jetson.yaml"
        else
            compose_file="docker-compose.project.run.linux-x86.yaml"
        fi
    elif [[ "${mode}" == "deploy" ]]; then
        compose_file="docker-compose.project.build.native.yaml"
    elif [[ "${mode}" == "ci-tests" ]]; then
        compose_file="docker-compose.project.build.native.yaml"
    elif [[ "${mode}" == "slurm" ]]; then
        compose_file="docker-compose.project.run.slurm.yaml"
    elif [[ "${mode}" == "release" ]]; then
        compose_file="docker-compose.project.build.multiarch.yaml"
    fi

    # Execute docker-compose config command
    echo "Showing configuration for ${mode} mode with ${compose_file}..."
    docker-compose --file "${DNP_LIB_PATH}/core/docker/${compose_file}" config "${remaining_args[@]}"

    return 0
}

