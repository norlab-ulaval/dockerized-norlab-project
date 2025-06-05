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
#                            Default to ".dockerized_norlab_project/slurm_jobs"
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

# ....Command functions............................................................................
test -d "${DNP_ROOT:?err}" || { echo "The DNP lib load error!" ; exit 1 ; }
test -d "${DNP_LIB_PATH:?err}" || { echo "The DNP lib load error!" ; exit 1 ; }

# Load super project configuration
source "${DNP_LIB_PATH}/core/utils/load_super_project_config.bash"

function dnp::project_validate() {
    local slurm=false
    local remaining_args=()

    # Parse options
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --slurm)
                slurm=true
                shift
                ;;
            --help|-h)
                dnp::project_validate_help
                exit 0
                ;;
            *)
                remaining_args+=("$@")
                break
                ;;
        esac
    done

    # Determine which validate script to execute
    if [[ "${slurm}" == true ]]; then
        echo "Validating slurm configuration..."
        source "${DNP_LIB_PATH}/core/execute/project_validate.slurm.bash" "${remaining_args[@]}"
    else
        echo "Validating configuration..."
        source "${DNP_LIB_PATH}/core/execute/project_validate.all.bash"
    fi

    return 0
}


function dnp::project_sanity() {
    local remaining_args=()

    # Parse options
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dnp::project_sanity_help
                exit 0
                ;;
            *)
                remaining_args+=("$@")
                break
                ;;
        esac
    done


    # Execute super_project_dnp_sanity_check.bash
    echo "Validating super project setup..."
    source "${DNP_LIB_PATH}/core/utils/super_project_dnp_sanity_check.bash"

    return 0
}

function dnp::project_dotenv() {
    local remaining_args=()

    # Parse options
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dnp::project_dotenv_help
                exit 0
                ;;
            *)
                remaining_args+=("$@")
                break
                ;;
        esac
    done

    # Show consolidated and interpolated dotenv config files
    echo "Showing consolidated and interpolated dotenv config files..."

    # Show DNP environment variables
    echo "=== DNP Environment Variables ==="
    env | grep "^DN_" | sort
    env | grep "^DNP_" | sort

    # Show project environment variables
    echo ""
    echo "=== Project Environment Variables ==="
    env | grep "^PROJECT_" | sort
    env | grep "^SUPER_" | sort

    # Show NBS environment variables
    echo ""
    echo "=== NBS Environment Variables ==="
    env | grep "^NBS_" | sort

    # Show N2ST environment variables
    echo ""
    echo "=== N2ST Environment Variables ==="
    env | grep "^N2ST_" | sort

    return 0
}


# ....Help commands................................................................................

function dnp::project_validate_help() {
    echo -e "${MSG_DIMMED_FORMAT}"
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "dnp validate --help"
    # Strip shell comment char `#` and both lines
    echo -e "${DOCUMENTATION_BUFFER_PROJECT_VALIDATE}" | sed 's/\# ====.*//' | sed 's/^\#//'
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "${MSG_END_FORMAT}"
}

function dnp::project_sanity_help() {
    echo -e "${MSG_DIMMED_FORMAT}"
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "dnp super validate --help"
    # Strip shell comment char `#` and both lines
    echo -e "${DOCUMENTATION_BUFFER_PROJECT_SANITY}" | sed 's/\# ====.*//' | sed 's/^\#//'
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "${MSG_END_FORMAT}"
}

function dnp::project_dotenv_help() {
    echo -e "${MSG_DIMMED_FORMAT}"
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "dnp super show --help"
    # Strip shell comment char `#` and both lines
    echo -e "${DOCUMENTATION_BUFFER_PROJECT_DOTENV}" | sed 's/\# ====.*//' | sed 's/^\#//'
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "${MSG_END_FORMAT}"
}

# Main help use in dnp::show_entrypoint_help()
function dnp::project_help() {
    echo -e "${MSG_DIMMED_FORMAT}"
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "dnp super --help"
    # Strip shell comment char `#` and both lines
    echo -e "${DOCUMENTATION_BUFFER_PROJECT}" | sed 's/\# ====.*//' | sed 's/^\#//'
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "${MSG_END_FORMAT}"
}
