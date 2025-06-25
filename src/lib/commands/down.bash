#!/bin/bash
# lib/commands/down.bash

DOCUMENTATION_BUFFER_DOWN=$( cat <<'EOF'
# =================================================================================================
# Stop DNA containers
#
# Usage:
#   $ dna down [OPTIONS] [<any-docker-compose-down-flags>]
#
# Options:
#   --slurm                Stop project related slurm containers
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
function dna::down_command() {
    local slurm=false
    local help=false
    local remaining_args=()
    local line_format="${MSG_LINE_CHAR_BUILDER_LVL2}"
    local line_style="${MSG_LINE_STYLE_LVL2}"

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --slurm)
                slurm=true
                shift
                ;;
            --help|-h)
                dna::command_help_menu "${DOCUMENTATION_BUFFER_DOWN:?err}"
                exit 0
                ;;
            *)
                remaining_args+=("$@")
                break
                ;;
        esac
    done

    n2st::print_formated_script_header "down procedure" "${line_format}" "${line_style}"
#    n2st::print_msg "Starting down procedure"

    # ....Load dependencies........................................................................
    source "${DNA_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1

    # ....Begin....................................................................................
    # Determine which down script to execute
    if [[ "${slurm}" == true ]]; then
        source "${DNA_LIB_PATH}/core/execute/down.slurm.bash"
        dna::down_slurm "${remaining_args[@]}" || return 1
        n2st::print_msg_done "slurm container down"
    else
        source "${DNA_LIB_PATH}/core/execute/down.bash"
        dna::down_command "${remaining_args[@]}" || return 1
    fi

#    n2st::print_msg_done "Completed down procedure"
    n2st::print_formated_script_footer "down procedure" "${line_format}" "${line_style}"

    return 0
}
