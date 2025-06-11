#!/bin/bash
# lib/commands/run.bash

DOCUMENTATION_BUFFER_RUN=$( cat <<'EOF'
# =================================================================================================
# Run commands in containers
#
# Usage:
#   $ dnp run [OPTIONS] SERVICE [--] [<specialized-option>]
#
# Service
#   ci-tests [<any-docker-argument>]       Run CI tests container.
#   slurm <sjob-id>                        Run slurm job in container.
#
# Specialized options
#   For details, execute $ dnp run SERVICE -- --help
#
# Options:
#   --help, -h             Show this help message
#
# Notes about slurm run:
#   To launch job on slurm/mamba server, prefer directly executing custom slurm script with
#   enviroment variable header. See example 'slurm_job.template.bash' and 'slurm_job.dryrun.bash'
#   in '.dockerized_norlab_project/slurm_jobs/' directory.
#
#
# =================================================================================================
EOF
)

# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
test -n "$( declare -F dnp::import_lib_and_dependencies )" || { echo -e "${dnp_error_prefix} The DNP lib is not loaded!" ; exit 1 ; }
test -n "$( declare -F n2st::print_msg )" || { echo -e "${dnp_error_prefix} The N2ST lib is not loaded!" ; exit 1 ; }
test -d "${DNP_ROOT:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }
test -d "${DNP_LIB_PATH:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
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
            --) # no more option
                shift
                remaining_args+=("$@")
                break
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
        source "${DNP_LIB_PATH}/core/execute/run.ci_tests.bash"
        # (temporary hack) ToDo: NMO-692 feat: add a build ci-tests option to run.ci_tests.bash
        if [[ "${remaining_args[*]}" =~ .*"--help".* ]]; then
          dnp::run_ci_tests "--help"
          exit 0
        fi
        n2st::print_msg "Running CI tests..."
        dnp build --ci-tests
        dnp::run_ci_tests "${remaining_args[@]}"
        fct_exit_code=$?
    elif [[ "${slurm}" == true ]]; then
        n2st::print_msg "Running slurm containers..."
        source "${DNP_LIB_PATH}/core/execute/run.slurm.bash"
        dnp::run_slurm "${remaining_args[@]}"
        fct_exit_code=$?
    else
        n2st::print_msg_error "No run service specified."
        dnp::command_help_menu "${DOCUMENTATION_BUFFER_RUN}"
    fi

    return $fct_exit_code
}
