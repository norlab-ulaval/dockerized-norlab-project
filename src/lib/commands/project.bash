#!/bin/bash
# lib/commands/project.bash

DOCUMENTATION_BUFFER_PROJECT=$( cat <<'EOF'
# =================================================================================================
# Super project DNP configuration related command
#
# Usage:
#   $ dnp project [validate|sanity|dotenv] [OPTIONS]
#
# Commands:
#   validate               Validate super project setup
#   sanity                 Validate super project setup
#   dotenv                 Show consolidated and interpolated dotenv config files
#
# Options:
#   --help, -h             Show this help message
#
# =================================================================================================
EOF
)

DOCUMENTATION_BUFFER_PROJECT_VALIDATE=$( cat <<'EOF'
# =================================================================================================
# Validate DNP project docker configurations: docker-compose, dotenv and shell variable substitution
# In short, iterate over each services of all docker-compose files (or just the slurm ones):
#   1. first in config mode with variable interpolation;
#   2. and after, execute build and/or run in dry-run mode.
#
# Usage:
#   $ dnp project validate [OPTIONS]
#
# Options:
#   --slurm ["<slurm/job/dir/path>"]   Validate only the slurm configuration
#   --help, -h                         Show this help message
#
# Slurm flag positional argument:
#   <slurm/job/dir/path>     (Optional) The path to the directory containing the slurm job scripts.
#                            Default to "slurm_jobs/"
#
# =================================================================================================
EOF
)

DOCUMENTATION_BUFFER_PROJECT_SANITY=$( cat <<'EOF'
# =================================================================================================
# Validate super project setup
#
# Usage:
#   $ dnp project validate [OPTIONS]
#
# Options:
#   --help, -h             Show this help message
#
# =================================================================================================
EOF
)

DOCUMENTATION_BUFFER_PROJECT_DOTENV=$( cat <<'EOF'
# =================================================================================================
# Show consolidated and interpolated dotenv config files
#
# Usage:
#   $ dnp project dotenv [OPTIONS]
#
# Options:
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
function dnp::project_validate_command() {
    local slurm=false
    local remaining_args=()


    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --slurm)
                slurm=true
                shift
                ;;
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_PROJECT_VALIDATE:?err}"
                exit 0
                ;;
            *)
                remaining_args+=("$@")
                break
                ;;
        esac
    done

    # ....Load dependencies........................................................................
    source "${DNP_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1
    source "${DNP_LIB_EXEC_PATH}/build.all.bash" || return 1
    source "${DNP_LIB_EXEC_PATH}/build.all.multiarch.bash" || return 1

    # ....Begin....................................................................................
    header_footer_name="project validate procedure"
    n2st::print_formated_script_header "${header_footer_name}" "${MSG_LINE_CHAR_BUILDER_LVL2}"

    # Determine which validate script to execute
    if [[ "${slurm}" == true ]]; then
        echo "Validating slurm configuration..."
        source "${DNP_LIB_PATH}/core/execute/project_validate.slurm.bash"
        dnp::project_validate_slurm "${remaining_args[@]}"
        fct_exit_code=$?
    else
        echo "Validating configuration..."
        source "${DNP_LIB_PATH}/core/execute/project_validate.all.bash"
        dnp::project_validate_all "${remaining_args[@]}"
        fct_exit_code=$?
    fi

    n2st::print_formated_script_footer "${header_footer_name}" "${MSG_LINE_CHAR_BUILDER_LVL2}"
    return $fct_exit_code
}


function dnp::project_sanity_command() {
    local remaining_args=()


    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_PROJECT_SANITY:?err}"
                exit 0
                ;;
            *)
                remaining_args+=("$@")
                break
                ;;
        esac
    done

    # ....Load dependencies........................................................................
    source "${DNP_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1
    source "${DNP_LIB_PATH}/core/utils/super_project_dnp_sanity_check.bash" || return 1

    # ....Begin....................................................................................
    header_footer_name="project sanity procedure"
    n2st::print_formated_script_header "${header_footer_name}" "${MSG_LINE_CHAR_BUILDER_LVL2}"

    # Execute super_project_dnp_sanity_check.bash
    echo "Validating super project setup..."
    dnp::super_project_dnp_sanity_check "${remaining_args[@]}"
    fct_exit_code=$?

    n2st::print_formated_script_footer "${header_footer_name}" "${MSG_LINE_CHAR_BUILDER_LVL2}"
    return $fct_exit_code
}

function dnp::project_dotenv_command() {
    local remaining_args=()


    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_PROJECT_DOTENV:?err}"
                exit 0
                ;;
            *)
                remaining_args+=("$@")
                break
                ;;
        esac
    done

    # ....Load dependencies........................................................................
    source "${DNP_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1

    # ....Begin....................................................................................
    header_footer_name="project dotenv procedure"
    n2st::print_formated_script_header "${header_footer_name}" "${MSG_LINE_CHAR_BUILDER_LVL2}"

    # Show consolidated and interpolated dotenv config files
    n2st::print_msg "Showing consolidated and interpolated dotenv config files...\n"

    # Show DNP environment variables
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "${MSG_DIMMED_FORMAT}DNP Environment Variables${MSG_END_FORMAT}"
    echo
    env | grep "^DN_" | sort
    env | grep "^DNP_" | sort

    # Show project environment variables
    echo ""
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "${MSG_DIMMED_FORMAT}Project Environment Variables${MSG_END_FORMAT}"
    echo
    env | grep "^PROJECT_" | sort
    env | grep "^SUPER_" | sort

    # Show NBS environment variables
    echo ""
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "${MSG_DIMMED_FORMAT}NBS Environment Variables${MSG_END_FORMAT}"
    echo
    env | grep "^NBS_" | sort

    # Show N2ST environment variables
    echo ""
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "${MSG_DIMMED_FORMAT}N2ST Environment Variables${MSG_END_FORMAT}"
    echo
    env | grep "^N2ST_" | sort

    n2st::print_formated_script_footer "${header_footer_name}" "${MSG_LINE_CHAR_BUILDER_LVL2}"
    return 0
}
