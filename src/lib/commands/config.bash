#!/bin/bash
# lib/commands/config.bash

DOCUMENTATION_BUFFER_CONFIG=$( cat <<'EOF'
# =================================================================================================
# Show Docker Compose configuration
#
# Usage:
#   $ dna config MODE
#
# Modes:
#   dev [platform]         Development mode
#   deploy [platform]      Deployment mode
#   ci-tests               CI tests mode
#   slurm                  SLURM mode
#   release                Release mode
#
# Platforms:
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

# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dna_error_prefix="\033[1;31m[DNA error]\033[0m"
test -n "$( declare -f dna::import_lib_and_dependencies )" || { echo -e "${dna_error_prefix} The DNA lib is not loaded!" 1>&2 && exit 1; }
test -n "$( declare -f n2st::print_msg )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
test -d "${DNA_ROOT:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }
test -d "${DNA_LIB_PATH:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dna::config_command() {
    local mode=""
    local platform=""
    local help=false

    declare -a remaining_args=()

    n2st::print_msg "Command ${MSG_DIMMED_FORMAT}dna config MODE${MSG_END_FORMAT} is not released yet, stay tuned!\n" && exit 0 # (CRITICAL) ToDo: on task end >> delete this line <--

    if [[ -z "$1" ]]; then
        dna::command_help_menu "${DOCUMENTATION_BUFFER_CONFIG:?err}"
        exit 1
    fi

    # ....cli......................................................................................
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
                dna::command_help_menu "${DOCUMENTATION_BUFFER_CONFIG:?err}"
                exit 0
                ;;
            *)
                remaining_args=("$@")
                break
                ;;
        esac
    done

    if [[ -z "${mode}" ]]; then
#        n2st::print_msg_error "Unknown mode!"
#        dna::command_help_menu "${DOCUMENTATION_BUFFER_CONFIG}"
        dna::unknown_subcommand_msg "config" "$remaining_args"
        exit 1
    fi


    # ....Load dependencies........................................................................
    # Load super project configuration
    source "${DNA_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1

    # ....Begin....................................................................................
    # Determine which compose file to use
    local compose_file=""


    if [[ "${mode}" == "dev" ]]; then
        if [[ "${platform}" == "darwin" ]]; then
#            compose_file="docker-compose.project.run.darwin.yaml"
            compose_file="docker-compose.project.build.native.yaml"
            # (CRITICAL) ToDo: implement <-- we are here
            docker compose --file "${DNA_LIB_PATH}/core/docker/${compose_file}" config project-core project-develop
        elif [[ "${platform}" == "jetson" ]]; then
            compose_file="docker-compose.project.run.jetson.yaml"
        else
            compose_file="docker-compose.project.run.linux-x86.yaml"
        fi
    elif [[ "${mode}" == "deploy" ]]; then
        compose_file="docker-compose.project.build.native.yaml"
    elif [[ "${mode}" == "ci-tests" ]]; then
        compose_file="docker-compose.project.run.ci-tests.yaml"
    elif [[ "${mode}" == "slurm" ]]; then
        compose_file="docker-compose.project.run.slurm.yaml"
    elif [[ "${mode}" == "release" ]]; then
        compose_file="docker-compose.project.build.multiarch.yaml"
    fi

    # Execute docker-compose config command
    echo "Showing configuration for ${mode} mode with ${compose_file}..."
    docker compose --file "${DNA_LIB_PATH}/core/docker/${compose_file}" config "${remaining_args[@]}"
    fct_exit_code=$?

    return $fct_exit_code
}

