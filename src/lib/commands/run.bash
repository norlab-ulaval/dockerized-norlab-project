#!/bin/bash
# lib/commands/run.bash

DOCUMENTATION_BUFFER_RUN=$( cat <<'EOF'
# =================================================================================================
# Run commands in containers
#
# Usage:
#   $ dnp run [OPTIONS] SERVICE [<specialized-option>]
#
# Service
#   ci-tests               Run CI tests container.
#   slurm                  Run slurm container. Run dnp run slurm --help to see available options
#
# Specialized options
#   For details, execute $ dnp run SERVICE --help
#
# Options:
#   --help, -h             Show this help message
#
# =================================================================================================
EOF
)

test -d "${DNP_ROOT:?err}" || { echo "The DNP lib load error!" ; exit 1 ; }
test -d "${DNP_LIB_PATH:?err}" || { echo "The DNP lib load error!" ; exit 1 ; }

function dnp::run_command() {
    local ci_tests=false
    local slurm=false
    local remaining_args=()

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            ci-tests)
                ci_tests=true
                shift
                ;;
            slurm)
                slurm=true
                shift
                ;;
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_RUN}"
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
    # Determine which run script to execute
    if [[ "${ci_tests}" == true ]]; then
        echo "Running CI tests..."
        source "${DNP_LIB_PATH}/core/execute/run.ci_tests.bash" "${remaining_args[@]}"
        fct_exit_code=$?
    elif [[ "${slurm}" == true ]]; then
        echo "Running slurm containers..."
        source "${DNP_LIB_PATH}/core/execute/run.slurm.bash" "${remaining_args[@]}"
        fct_exit_code=$?
    else
        echo "Error: No run mode specified."
        dnp::run_help
        exit 1
    fi

    return $fct_exit_code
}
