#!/bin/bash
# lib/commands/down.bash

DOCUMENTATION_BUFFER_DOWN=$( cat <<'EOF'
# =================================================================================================
# Stop DNP containers
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

# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
test -n "$( declare -f dnp::import_lib_and_dependencies )" || { echo -e "${dnp_error_prefix} The DNP lib is not loaded!" ; exit 1 ; }
test -n "$( declare -f n2st::print_msg )" || { echo -e "${dnp_error_prefix} The N2ST lib is not loaded!" ; exit 1 ; }
test -d "${DNP_ROOT:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }
test -d "${DNP_LIB_PATH:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dnp::down_command() {
    local slurm=false
    local help=false
    local remaining_args=()

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --slurm)
                slurm=true
                shift
                ;;
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_DOWN}"
                exit 0
                ;;
            *)
                remaining_args+=("$@")
                break
                ;;
        esac
    done

    header_footer_name="down procedure"
    n2st::print_formated_script_header "${header_footer_name}" "${MSG_LINE_CHAR_BUILDER_LVL2}"
#    n2st::print_msg "Starting ${header_footer_name}"

    # ....Load dependencies........................................................................
    source "${DNP_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1

    # ....Begin....................................................................................
    # Determine which down script to execute
    if [[ "${slurm}" == true ]]; then
        source "${DNP_LIB_PATH}/core/execute/down.slurm.bash"
        dnp::down_slurm "${remaining_args[@]}" || return 1
        n2st::print_msg_done "slurm container down"
    else
        source "${DNP_LIB_PATH}/core/execute/down.bash"
        dnp::down_command "${remaining_args[@]}" || return 1
    fi

#    n2st::print_msg_done "Completed ${header_footer_name}"
    n2st::print_formated_script_footer "${header_footer_name}" "${MSG_LINE_CHAR_BUILDER_LVL2}"

    return 0
}
