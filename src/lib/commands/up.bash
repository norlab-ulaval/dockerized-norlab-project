#!/bin/bash
# lib/commands/up.bash

DOCUMENTATION_BUFFER_UP=$( cat <<'EOF'
# =================================================================================================
# Start and attach to a DNP containers
#
# Notes:
#   • spin a specific service from 'docker-compose.project.run.<DEVICE>.yaml'
#   • service(s) are started in detach mode
#   • device and architecture specific docker-compose config are automaticaly selected at runtime
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
            *)
                remaining_args+=("$@")
                break
                ;;
        esac
    done

    # Splash type: small, negative or big
    n2st::norlab_splash 'Dockerized-NorLab-Project' 'https://github.com/norlab-ulaval/dockerized-norlab-project.git' 'small'
#    header_footer_name="up and attach procedure"
#    n2st::print_formated_script_header "${header_footer_name}" "${MSG_LINE_CHAR_BUILDER_LVL1}"

    # ....Load dependencies........................................................................
    source "${DNP_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1
    source "${DNP_LIB_PATH}/core/execute/up_and_attach.bash" || return 1

    # ....Begin....................................................................................
    dnp::up_and_attach "${remaining_args[@]}"
    fct_exit_code=$?
    n2st::print_msg "Detached. ${MSG_DIMMED_FORMAT}Container is running in background${MSG_END_FORMAT}."
    return $fct_exit_code
}

