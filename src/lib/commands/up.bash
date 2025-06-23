#!/bin/bash
# lib/commands/up.bash

DOCUMENTATION_BUFFER_UP=$( cat <<'EOF'
# =================================================================================================
# Start and 'attach to' or' execute cmd in' a DNP containers.
#
# Usage:
#   $ dnp up [OPTIONS] [SERVICE] [-- COMMAND [ARGS...]]
#
# Options:
#   --no-attach              Don't attach to started container
#                             (not compatible with 'execute options')
#   -h | --help              Show this help message
#
# 'Execute' options:
#   -e, --env stringArray    Set container environment variables
#   -w, --workdir string     Override path to workdir directory
#   -T, --no-TTY             Disable pseudo-TTY allocation
#   --detach                 Execute COMMAND in the background
#   --dry-run                (Require --detach flag)
#
# SERVICE:
#   develop                  Start develop service (default)
#   deploy                   Start deploy service
#
# Positional argument:
#   command & arguments    Any command to be executed inside the docker container (default: bash)
#
# Example:
#   $ dnp up --workdir "/" -- bash -c 'tree -L 1 -a $(pwd)'
#
# Notes:
#   • spin a specific service from 'docker-compose.project.run.<DEVICE>.yaml'.
#   • device and architecture specific docker-compose config are automaticaly
#     selected at runtime.
#   • service(s) are started in daemon mode so that when you exit the attached
#     container, it keep running in the bachground.
#     This equivalent to 'docker compose up --detach && docker compose attach'.
#
# =================================================================================================
EOF
)

# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
test -n "$( declare -f dnp::import_lib_and_dependencies )" || { echo -e "${dnp_error_prefix} The DNP lib is not loaded!" ; exit 1 ; }
test -n "$( declare -f n2st::print_msg )" || { echo -e "${dnp_error_prefix} The N2ST lib is not loaded!" ; exit 1 ; }
test -d "${DNP_ROOT:?err}" || { echo -e "${dnp_error_prefix} library load error!" ; exit 1 ; }
test -d "${DNP_LIB_PATH:?err}" || { echo -e "${dnp_error_prefix} library load error!" ; exit 1 ; }

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dnp::up_command() {
    local remaining_args=()
    local service="develop"
    declare -i service_override=0
    local original_command="$*"
    local line_format="${MSG_LINE_CHAR_BUILDER_LVL1}"
    local line_style="${MSG_LINE_STYLE_LVL2}"

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_UP:?err}"
                exit 0
                ;;
            develop|deploy)
                service="$1"
                service_override+=1
                shift
                ;;
            --no-up)
                dnp::illegal_command_msg "up" "--no-up" "Its a dnp internal flag"
                exit 1
                ;;
            *)
                # Check if it starts with -- (unknown option) or allow other arguments to pass through as commands
                remaining_args+=("$@")
                break
                ;;
        esac
    done

    if [[ ${service_override} -ge 2 ]]; then
        # If service is set twice, it's an error
        dnp::illegal_command_msg "up" "${original_command}" "Only one SERVICE can be specified.\n"
        return 1
    fi

    # Splash type: small, negative or big
    n2st::norlab_splash "${DNP_SPLASH_NAME_SMALL:?err}" "${DNP_GIT_REMOTE_URL}" "small"
    n2st::print_formated_script_header "up procedure" "${line_format}" "${line_style}"

    # ....Load dependencies........................................................................
    source "${DNP_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1
    source "${DNP_LIB_PATH}/core/execute/up_and_attach.bash" || return 1

    # ....Set service..............................................................................
    # Check if a service was specified
    if [[ ${service_override} -eq 0 ]]; then
        # If no service was explicitly specified, check for offline deployment
        local offline_service
        if offline_service=$(dnp::check_offline_deploy_service_discovery 2>/dev/null); then
            service="${offline_service}"
            n2st::print_msg "Using offline deployment service: ${service}"
        fi
    fi

    # Add service to remaining_args
    remaining_args=("--service" "${service}" "${remaining_args[@]}")

    # ....Begin....................................................................................
    dnp::up_and_attach "${remaining_args[@]}"
    fct_exit_code=$?
    if [[ ${fct_exit_code} -eq 0 ]]; then
      n2st::print_msg "Detached. ${MSG_DIMMED_FORMAT}Container ${DN_CONTAINER_NAME} is running in background${MSG_END_FORMAT}"
    fi
    return $fct_exit_code
}
