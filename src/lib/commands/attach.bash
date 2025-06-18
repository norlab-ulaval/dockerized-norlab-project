#!/bin/bash
# lib/commands/attach.bash

DOCUMENTATION_BUFFER_Attach=$( cat <<'EOF'
# =================================================================================================
# Attach to a running DNP containers
#
# Usage:
#   $ dnp attach [OPTIONS] [SERVICE]
#
# Options:
#   -h | --help              Show this help message
#
# SERVICE:
#   develop                  Attach to develop service (default)
#   deploy                   Attach to deploy service
#
# =================================================================================================
EOF
)

# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
test -n "$( declare -f dnp::import_lib_and_dependencies )" || { echo -e "${dnp_error_prefix} The DNP lib is not loaded!" ; exit 1 ; }
test -n "$( declare -f n2st::print_msg )" || { echo -e "${dnp_error_prefix} The N2ST lib is not loaded!" ; exit 1 ; }
test -d "${DNP_ROOT:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }
test -d "${DNP_LIB_PATH:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dnp::attach_command() {
    declare -a docker_compose_exec_flag=()
    local service="develop"
    local service_set=false
    local original_command="$*"

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_Attach}"
                exit 0
                ;;
            develop|deploy)
                # If service is already set, it's an error
                if [[ "${service_set}" == true ]]; then
                    dnp::illegal_command_msg "attach" "${original_command}" "Only one SERVICE can be specified.\n"
                    return 1
                fi
                service="$1"
                service_set=true
                shift
                ;;
            --no-attach)
                dnp::illegal_command_msg "attach" "--no-attach" "Its a dnp internal flag"
                exit 1
                ;;
            *)
                # Check if it starts with -- (unknown option)
                if [[ "$1" == --* ]]; then
                    dnp::unknown_subcommand_msg "attach" "$*"
                    exit 1
                fi
                # Otherwise it's an unknown service
                dnp::illegal_command_msg "attach" "${original_command}" "Unknown SERVICE: $1. Valid services are: develop, deploy.\n"
                return 1
                ;;
        esac
    done

    # Add service to docker_compose_exec_flag
    docker_compose_exec_flag+=("--service" "${service}")

    # Splash type: small, negative or big
    n2st::norlab_splash 'Dockerized-NorLab-Project' 'https://github.com/norlab-ulaval/dockerized-norlab-project.git' 'small'

    # ....Load dependencies........................................................................
    source "${DNP_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1
    source "${DNP_LIB_PATH}/core/execute/up_and_attach.bash" || return 1

    # ....Begin....................................................................................
    dnp::up_and_attach --no-up "${docker_compose_exec_flag[@]}"
    fct_exit_code=$?
    if [[ ${fct_exit_code} -eq 0 ]]; then
      n2st::print_msg "Detached. ${MSG_DIMMED_FORMAT}Container ${DN_CONTAINER_NAME} is running in background${MSG_END_FORMAT}"
    fi
    return $fct_exit_code
}
