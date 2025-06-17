#!/bin/bash
# lib/commands/run.bash

DOCUMENTATION_BUFFER_RUN=$( cat <<'EOF'
# =================================================================================================
# Run commands in containers non-interactively
#
# Usage:
#   $ dnp run SERVICE [OPTIONS]
#
# Service:
#   Interactive:
#     dnp run develop [OPTIONS] [-- COMMAND [ARGS...]]   Run command in a development container.
#     dnp run deploy [OPTIONS] [-- COMMAND [ARGS...]]    Run command in a deployment container.
#   Non-interactive:
#     dnp run ci-tests [OPTIONS]          Run CI tests container.
#     dnp run slurm <sjob-id> [OPTIONS]   Run slurm job in container.
#
# Options:
#   --help, -h                   Show this help message
#
# Options for interactive services:
#   -e, --env stringArray        Set container environment variables
#   -w, --workdir string         Override path to workdir directory
#   -T, --no-TTY                 Disable pseudo-TTY allocation
#   -v, --volume stringArray     Bind mount a volume
#   --detach                     Execute COMMAND in the background
#   --dry-run                    (Require --detach flag)
#
# Options for non-interactive services:
#   For details, execute $ dnp run [ci-tests|slurm] -- --help
#
# Notes about slurm run:
#   To launch job on slurm/mamba server, prefer directly executing custom slurm script with
#   enviroment variable header. See example 'slurm_job.*template.bash' and 'slurm_job.dryrun.bash'
#   in 'slurm_jobs/' directory.
#
# =================================================================================================
EOF
)

# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
test -n "$( declare -f dnp::import_lib_and_dependencies )" || { echo -e "${dnp_error_prefix} The DNP lib is not loaded!" ; exit 1 ; }
test -n "$( declare -f n2st::print_msg )" || { echo -e "${dnp_error_prefix} The N2ST lib is not loaded!" ; exit 1 ; }
test -n "$( declare -f n2st::norlab_splash )" || { echo -e "${dnp_error_prefix} The N2ST lib is not loaded!" ; exit 1 ; }
test -d "${DNP_ROOT:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }
test -d "${DNP_LIB_PATH:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dnp::run_command() {
    local ci_tests=false
    local slurm=false
    local deploy=false
    local develop=false
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
            deploy)
                deploy=true
                shift
                ;;
            develop)
                develop=true
                shift
                ;;
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_RUN}"
                exit 0
                ;;
            --) # no more option
                remaining_args+=("$@")
                break
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
        dnp build ci-tests -- --no-cache
        dnp::run_ci_tests "${remaining_args[@]}"
        fct_exit_code=$?
    elif [[ "${slurm}" == true ]]; then
        n2st::print_msg "Running slurm containers..."
        source "${DNP_LIB_PATH}/core/execute/run.slurm.bash"
        if [[ "${remaining_args[*]}" =~ .*"--help".* ]]; then
          dnp::run_slurm "--help"
          exit 0
        fi
        dnp::run_slurm "${remaining_args[@]}"
        fct_exit_code=$?
    elif [[ "${develop}" == true ]]; then
        n2st::print_msg "Running develop containers..."
        source "${DNP_LIB_PATH}/core/execute/up_and_attach.bash" || return 1
        source "${DNP_LIB_PATH}/core/execute/run.any.bash" || return 1
        dnp::run_any --service project-develop "${remaining_args[@]}"
        fct_exit_code=$?
    elif [[ "${deploy}" == true ]]; then
        n2st::print_msg "Running deploy containers..."
        source "${DNP_LIB_PATH}/core/execute/up_and_attach.bash" || return 1
        source "${DNP_LIB_PATH}/core/execute/run.any.bash" || return 1
        dnp::run_any --service project-deploy "${remaining_args[@]}"
        fct_exit_code=$?
    else
        n2st::print_msg_error "No run service specified."
        dnp::command_help_menu "${DOCUMENTATION_BUFFER_RUN}"
    fi

    return $fct_exit_code
}
