#!/bin/bash
# lib/commands/up.bash

DOCUMENTATION_BUFFER_UP=$( cat <<'EOF'
# =================================================================================================
# 'Start' and 'attach to' or' execute cmd in' a DNP containers.
#
# Usage:
#   $ dnp up [OPTIONS] [-- COMMAND [ARGS...]]
#
# Example:
#   $ dnp up --workdir "/" -- bash -c 'tree -L 1 -a $(pwd)'
#
# 'Start' options:
#   --service SERVICE        The service to attach once up (Default: develop)
#                            Service: develop, deploy, ...
#   --no-attach              Don't attach to started containeror
#                             (not compatible with 'execute options')
#   -h | --help
#
# 'Execute' options:
#   -e, --env stringArray    Set container environment variables
#   -w, --workdir string     Override path to workdir directory
#   -T, --no-TTY             Disable pseudo-TTY allocation
#   --detach                 Execute COMMAND in the background
#   --dry-run                (Require --detach flag)
#
# Positional argument:
#   command & arguments    Any command to be executed inside the docker container (default: bash)
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
test -d "${DNP_ROOT:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }
test -d "${DNP_LIB_PATH:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dnp::up_command() {
    local remaining_args=()

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_UP}"
                exit 0
                ;;
            --no-up)
                dnp::illegal_command_msg "up" "--no-up" "Its a dnp internal flag"
                exit 1
                ;;
            *)
                remaining_args+=("$@")
                break
                ;;
        esac
    done

    # Splash type: small, negative or big
    n2st::norlab_splash 'Dockerized-NorLab-Project' 'https://github.com/norlab-ulaval/dockerized-norlab-project.git' 'small'

    # ....Load dependencies........................................................................
    source "${DNP_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1
    source "${DNP_LIB_PATH}/core/execute/up_and_attach.bash" || return 1

    # ....Begin....................................................................................
    dnp::up_and_attach "${remaining_args[@]}"
    fct_exit_code=$?
    if [[ ${fct_exit_code} -eq 0 ]]; then
      n2st::print_msg "Detached. ${MSG_DIMMED_FORMAT}Container ${DN_CONTAINER_NAME} is running in background${MSG_END_FORMAT}"
    fi
    return $fct_exit_code
}

