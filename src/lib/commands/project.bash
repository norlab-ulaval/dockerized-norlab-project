#!/bin/bash
# lib/commands/project.bash

DOCUMENTATION_BUFFER_PROJECT=$( cat <<'EOF'
# =================================================================================================
# Super project DNA configuration related command
#
# Usage:
#   $ dna project [validate|sanity|dotenv] [OPTIONS]
#
# Commands:
#   validate               Validate super project '.dockerized_norlab' configurations
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
# Validate super project '.dockerized_norlab' configurations including docker-compose files, dotenv
# files environment variables and shell variable substitutions.
#
# In short, iterate over each services of all docker-compose files (or just the slurm ones):
#   1. first in config mode with variable interpolation;
#   2. and after, execute build and/or run in dry-run mode.
#
# Usage:
#   $ dna project validate [OPTIONS]
#
# Options:
#   --slurm ["<slurm/job/dir/path>"]   Validate only the slurm configuration
#   --include-multiarch                Include multi-architecture image validation
#   --help, -h                         Show this help message
#
# Slurm flag positional argument:
#   <slurm/job/dir/path>               (Optional) The slurm job scripts directory path.
#                                      Default to "slurm_jobs/"
#
# =================================================================================================
EOF
)

DOCUMENTATION_BUFFER_PROJECT_SANITY=$( cat <<'EOF'
# =================================================================================================
# Validate super project DNA required components and dependencies are installed as expected.
#
# Usage:
#   $ dna project sanity [OPTIONS]
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
#   $ dna project dotenv [OPTIONS]
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
function dna::project_validate_command() {
    local slurm=false
    local include_multiarch=false
    local remaining_args=()
    local line_format="${MSG_LINE_CHAR_BUILDER_LVL1}"
    local line_style="${MSG_LINE_STYLE_LVL2}"


    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --slurm)
                slurm=true
                shift
                ;;
            --include-multiarch)
                include_multiarch=true
                shift
                ;;
            --help|-h)
                dna::command_help_menu "${DOCUMENTATION_BUFFER_PROJECT_VALIDATE:?err}"
                exit 0
                ;;
            *)
                remaining_args+=("$@")
                break
                ;;
        esac
    done

    header_footer_name="project validate procedure"
    n2st::print_formated_script_header "${header_footer_name}" "${line_format}" "${line_style}"

    # ....Load dependencies........................................................................
    source "${DNA_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1
    source "${DNA_LIB_EXEC_PATH}/build.all.bash" || return 1
    source "${DNA_LIB_EXEC_PATH}/build.all.multiarch.bash" || return 1

    # ....Begin....................................................................................
    # Add --include-multiarch to remaining_args if set
    if [[ "${include_multiarch}" == true ]]; then
        remaining_args=("--include-multiarch" "${remaining_args[@]}")
    fi

    # Determine which validate script to execute
    if [[ "${slurm}" == true ]]; then
        n2st::print_msg "The message""Validating slurm configuration..."
        source "${DNA_LIB_PATH}/core/execute/project_validate.slurm.bash"
        dna::project_validate_slurm "${remaining_args[@]}"
        fct_exit_code=$?
    else
        n2st::print_msg "The message""Validating configuration..."
        source "${DNA_LIB_PATH}/core/execute/project_validate.all.bash"
        dna::project_validate_all "${remaining_args[@]}"
        fct_exit_code=$?
    fi

    n2st::print_formated_script_footer "${header_footer_name}" "${line_format}" "${line_style}"
    return $fct_exit_code
}


function dna::project_sanity_command() {
    local remaining_args=()
    local line_format="${MSG_LINE_CHAR_BUILDER_LVL2}"
    local line_style="${MSG_LINE_STYLE_LVL2}"


    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dna::command_help_menu "${DOCUMENTATION_BUFFER_PROJECT_SANITY:?err}"
                exit 0
                ;;
            *)
                remaining_args+=("$@")
                break
                ;;
        esac
    done

    header_footer_name="project sanity procedure"
    n2st::print_formated_script_header "${header_footer_name}" "${line_format}" "${line_style}"

    # ....Load dependencies........................................................................
    source "${DNA_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1
    source "${DNA_LIB_PATH}/core/utils/super_project_dna_sanity_check.bash" || return 1

    # ....Begin....................................................................................

    # Execute super_project_dna_sanity_check.bash
    n2st::print_msg "Validating super project setup..."
    dna::super_project_dna_sanity_check "${remaining_args[@]}"
    fct_exit_code=$?

    n2st::print_formated_script_footer "${header_footer_name}" "${line_format}" "${line_style}"
    return $fct_exit_code
}

function dna::project_dotenv_command() {
    local remaining_args=()
    local line_format="${MSG_LINE_CHAR_BUILDER_LVL2}"
    local line_style="${MSG_LINE_STYLE_LVL2}"


    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dna::command_help_menu "${DOCUMENTATION_BUFFER_PROJECT_DOTENV:?err}"
                exit 0
                ;;
            *)
                remaining_args+=("$@")
                break
                ;;
        esac
    done

    header_footer_name="project dotenv procedure"
    n2st::print_formated_script_header "${header_footer_name}" "${line_format}" "${line_style}"

    # ....Load dependencies........................................................................
    source "${DNA_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1

    # ....Begin....................................................................................
    # Show consolidated and interpolated dotenv config files
    n2st::print_msg "Showing consolidated and interpolated dotenv config files...\n"

    # Show DNA-specific environment variables
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "${MSG_DIMMED_FORMAT}DNA-specific Environment Variables${MSG_END_FORMAT}"
    echo
    env | grep "^DN_" | sort
    env | grep "^DNA_" | sort

    # Show project-specific environment variables
    echo ""
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "${MSG_DIMMED_FORMAT}Project-specific Environment Variables${MSG_END_FORMAT}"
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

    n2st::print_formated_script_footer "${header_footer_name}" "${line_format}" "${line_style}"
    return 0
}
