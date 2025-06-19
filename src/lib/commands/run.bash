#!/bin/bash
# lib/commands/run.bash

DOCUMENTATION_BUFFER_RUN=$( cat <<'EOF'
# =================================================================================================
# Run commands in a uniquely identified containers
#
# Usage:
#   $ dnp run [OPTIONS] SERVICE
#
# Options:
#   --help, -h                   Show this help message
#   --help-develop               Show run develop help message
#   --help-deploy                Show run deploy help message
#   --help-slurm                 Show run slurm help message
#   --help-ci-tests              Show run ci-tests help message
#
# Service:
#
#   Interactive container:
#     $ dnp run [OPTIONS] develop|deploy [--] [COMMAND [ARGS...]]
#
#   Non-interactive container:
#     $ dnp run [OPTIONS] ci-tests [COMMAND [ARG...]]
#     $ dnp run [OPTIONS] slurm <sjob-id> [--] <python-cmd-args>
#
# Note:
#   Can be executed while compose service are up as the run container are assigne a new unique
#   ID at each instanciation i.e. <DN_CONTAINER_NAME>-ID.
#
# =================================================================================================
EOF
)

DOCUMENTATION_BUFFER_RUN_DEVELOP_DEPLOY_CMD=$( cat <<'EOF'
# =================================================================================================
# Run a develop/deploy uniquely identified containers
#
# Usage:
#   $ dnp run [OPTIONS] develop|deploy [--] [COMMAND [ARGS...]]
#
# Options for interactive services:
#   -e, --env stringArray        Set container environment variables
#   -w, --workdir string         Override path to workdir directory
#   -T, --no-TTY                 Disable pseudo-TTY allocation
#   -v, --volume stringArray     Bind mount a volume
#   --detach                     Execute COMMAND in the background
#   --dry-run                    (Require --detach flag)
#
# =================================================================================================
EOF
)

DOCUMENTATION_RUN_SLURM_CMD=$( cat <<'EOF'
# =================================================================================================
# Run slurm job specilaized DNP container.
# - Handle stoping the container in case the slurm command `scancel` is issued.
# - Rebuild images automaticaly
#
# Usage:
#   $ dnp run [OPTIONS] <sjob-id> [--] <any-python-args>
#
# Optional flag:
#   --log-name=<name>                                 The log file name without postfix
#   --log-path=<absolute-path-super-project-root>     The Absolute path to the slurm log directory.
#                                                     Will be created if it does not exist.
#   --skip-core-force-rebuild
#   --hydra-dry-run                                   Dry-run slurm job using registered hydra flag
#   --register-hydra-dry-run-flag                     Hydra flag used by '--hydra-dry-run'
#                                                     e.g., "+dev@_global_=math_env_slurm_job_dryrun"
#   -h | --help                                       Show this help message
#
# Positional argument:
#   <sjob-id>              (required) Used to ID the docker container, slurm job, optuna study ...
#   <any-python-args>      (required) The python command with flags
#
# Notes about slurm run:
#   To launch job on slurm/mamba server, use 'dnp run slurm ...' command in a slurm launch script.
#   See example 'slurm_job.*template.bash' and 'slurm_job.dryrun.bash' in 'slurm_jobs/' directory.
#
# =================================================================================================
EOF
)

DOCUMENTATION_BUFFER_RUN_CI_TESTS_CMD=$( cat <<'EOF'
# =================================================================================================
# Run continuous integration tests container.
#
# Usage:
#   $ dnp build ci-tests
#   $ dnp run ci_tests [COMMAND [ARG...]]
#
# Note: Require executing `dnp build ci-tests` first.
#
# =================================================================================================
EOF
)

DOCUMENTATION_BUFFER_RUN_CI_TESTS_CMD=$( cat <<'EOF'
# =================================================================================================
# Run continuous integration tests container.
#
# Usage:
#   $ dnp build ci-tests
#   $ dnp run ci_tests [COMMAND [ARG...]]
#
# Note: Require executing `dnp build ci-tests` first.
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
    local service=""
    declare -i service_set=0
    local remaining_args=()
    local original_command="$*"

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_RUN}"
                exit 0
                ;;
            --help-slurm)
                dnp::command_help_menu "${DOCUMENTATION_RUN_SLURM_CMD}"
                exit 0
                ;;
            --help-ci-tests)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_RUN_CI_TESTS_CMD}"
                exit 0
                ;;
            --help-develop|--help-deploy)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_RUN_DEVELOP_DEPLOY_CMD}"
                exit 0
                ;;
            --detach|--dry-run|-T|--no-TTY) # Assume its a docker compose flag
                remaining_args+=("$1")
                if [[ ${1} == "--dry-run" ]]; then
                  remaining_args+=("--detach")
                fi
                shift
                ;;
            -e|--env|-w|--workdir|-v|--volume) # Assume its a docker compose flag
                remaining_args+=("$1" "$2")
                shift
                shift
                ;;
            ci-tests)
                service="ci-tests"
                service_set+=1
                shift
                ;;
            slurm)
                service="slurm"
                service_set+=1
                shift
                ;;
            deploy)
                service="deploy"
                service_set+=1
                shift
                ;;
            develop)
                service="develop"
                service_set+=1
                shift
                ;;
            --) # no more option
                remaining_args+=("$@")
                break
                ;;
            *)
                # Otherwise, pass remaining arguments through
                remaining_args+=("$@")
                break
                ;;
        esac
    done

    # Check if a service was specified
    if [[ ${service_set} -eq 0 ]]; then
        n2st::print_msg_error "Service is either unknown or not specified."
        dnp::command_help_menu "${DOCUMENTATION_BUFFER_RUN}"
        return 1
    elif [[ ${service_set} -ge 2 ]]; then
        # If service is already set, it's an error
        n2st::print_msg_error "Only one SERVICE can be specified."
        dnp::command_help_menu "${DOCUMENTATION_BUFFER_RUN}"
        return 1
    fi

        # Splash type: small, negative or big
    n2st::norlab_splash 'Dockerized-NorLab-Project' 'https://github.com/norlab-ulaval/dockerized-norlab-project.git' 'small'

    # ....Load dependencies........................................................................
    source "${DNP_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1
    source "${DNP_LIB_PATH}/core/execute/run.ci_tests.bash"
    source "${DNP_LIB_PATH}/core/execute/up_and_attach.bash" || return 1
    source "${DNP_LIB_PATH}/core/execute/run.any.bash" || return 1
    source "${DNP_LIB_PATH}/core/execute/run.slurm.bash"



    # ....Begin....................................................................................
    # Determine which run script to execute
    if [[ "${service}" == "ci-tests" ]]; then
        n2st::print_msg "Running CI tests..."
        # (temporary hack) ToDo: NMO-692 feat: add a build ci-tests option to run.ci_tests.bash
        dnp build ci-tests -- --no-cache || return 1
        dnp::run_ci_tests "${remaining_args[@]}"
        fct_exit_code=$?
    elif [[ "${service}" == "slurm" ]]; then
        n2st::print_msg "Running slurm containers..."
        dnp::run_slurm "${remaining_args[@]}"
        fct_exit_code=$?
    elif [[ "${service}" == "develop" ]]; then
        n2st::print_msg "Running develop containers..."
        dnp::run_any --service project-develop "${remaining_args[@]}"
        fct_exit_code=$?
    elif [[ "${service}" == "deploy" ]]; then
        n2st::print_msg "Running deploy containers..."
        dnp::run_any --service project-deploy "${remaining_args[@]}"
        fct_exit_code=$?
    fi

    return $fct_exit_code
}
