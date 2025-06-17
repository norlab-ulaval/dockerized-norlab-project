#!/bin/bash
# lib/commands/attach.bash

DOCUMENTATION_BUFFER_Attach=$( cat <<'EOF'
# =================================================================================================
# Attach to a running DNP containers
#
# Usage:
#   $ dnp attach [OPTIONS]
#
# Options:
#   --service SERVICE        The service to attach once up (Default: develop)
#                            Service: develop, deploy, ...
#   -h | --help
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

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_Attach}"
                exit 0
                ;;
            --service)
                docker_compose_exec_flag+=("$1" "$2")
                shift
                shift
                ;;
            --no-attach)
                dnp::illegal_command_msg "attach" "--no-attach" "Its a dnp internal flag"
                exit 1
                ;;
            *)
                dnp::illegal_command_msg "attach" "$*"
                exit 1
                ;;
        esac
    done

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

