#!/bin/bash
# lib/commands/run.bash

DOCUMENTATION_BUFFER_RUN=$( cat <<'EOF'
# =================================================================================================
# Run commands in a uniquely identified containers
#
# Usage:
#   $ dna run [OPTIONS] SERVICE
#
# Options:
#   --help, -h                   Show this help message
#   --help-develop               Show run develop help message
#   --help-deploy                Show run deploy help message
#   --help-slurm                 Show run slurm help message
#   --help-slurm-apptainer       Show run slurm apptainer help message
#   --help-ci-tests              Show run ci-tests help message
#
# Service:
#
#   Interactive container:
#     $ dna run [OPTIONS] develop|deploy [--] [COMMAND [ARGS...]]
#
#   Non-interactive container:
#     $ dna run [OPTIONS] ci-tests [COMMAND [ARG...]]
#     $ dna run [OPTIONS] slurm <sjob-id> [--] <python-cmd-args>
#     $ dna run [OPTIONS] slurm <sjob-id> --generate-apptainer <profile> [--] <python-cmd-args>
#
# Note:
#   Can be executed while compose service are up as the run container are assigne a new unique
#   ID at each instanciation i.e. <DN_CONTAINER_NAME>-ID.
#   Use --generate-apptainer <profile> (or --ga) with slurm to generate Apptainer exec
#   scripts for HPC servers using Apptainer (e.g., Valeria, Compute Canada).
#   Does NOT execute apptainer locally. See --help-slurm-apptainer for details.
#
# =================================================================================================
EOF
)

DOCUMENTATION_BUFFER_RUN_DEVELOP_DEPLOY_CMD=$( cat <<'EOF'
# =================================================================================================
# Run a develop/deploy uniquely identified containers
#
# Usage:
#   $ dna run [OPTIONS] develop|deploy [--] [COMMAND [ARGS...]]
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
# Run slurm job specilaized DNA container.
# - Handle stoping the container in case the slurm command `scancel` is issued.
# - Rebuild images automaticaly
#
# Usage:
#   $ dna run [OPTIONS] slurm <sjob-id> [--] <any-python-args>
#
# Optional flag:
#   --generate-apptainer, --ga <profile>                Generate Apptainer artifacts instead of
#                                                     running Docker. See --help-slurm-apptainer.
#   --log-name <name>                                 The log file name without postfix
#   --log-path <absolute-path-super-project-root>     The Absolute path to the slurm log directory.
#                                                     Will be created if it does not exist.
#   --skip-core-force-rebuild
#   --skip-slurm-force-rebuild
#   --hydra-dry-run                                   Dry-run slurm job using registered hydra flag
#   --register-hydra-dry-run-flag                     Hydra flag used by '--hydra-dry-run'
#                                                     e.g., "+dev@_global_=math_env_slurm_job_dryrun"
#   -e, --env stringArray                             Set container environment variables
#   -w, --workdir string                              Override path to workdir directory
#   -T, --no-TTY                                      Disable pseudo-TTY allocation
#   -v, --volume stringArray                          Bind mount a volume
#   -h | --help                                       Show this help message
#
# Positional argument:
#   <sjob-id>              (required) Used to ID the docker container, slurm job, optuna study ...
#   <any-python-args>      (required) The python command with flags
#
# Notes about slurm run:
#   To launch job on slurm/mamba server, use 'dna run slurm ...' command in a slurm launch script.
#   See example 'slurm_job.*template.bash' and 'slurm_job.dryrun.bash' in 'slurm_jobs/' directory.
#
# =================================================================================================
EOF
)

DOCUMENTATION_RUN_SLURM_APPTAINER_CMD=$( cat <<'EOF'
# =================================================================================================
# Generate Apptainer exec command/script for running a slurm job on an HPC server.
#
# IMPORTANT: This command does NOT execute Docker or Apptainer locally.
#   It generates a standalone apptainer exec script meant to be transferred and run on the
#   target HPC server. Apptainer is Linux-only and NOT supported on macOS.
#
# Design constraints:
#   - Apptainer is Linux-only and NOT supported on macOS.
#   - Generated scripts are self-contained (no DNA dependency on the HPC server).
#   - Supported HPC server archetypes:
#       Mamba (NorLab): Docker + Apptainer runtime → use 'dna run slurm' (Docker) or --ga mamba (Apptainer)
#       Valeria:        Apptainer runtime, no DNA on server  → use generated standalone script
#       Compute Canada: Apptainer runtime, no DNA on server  → use generated standalone script
#
# Usage:
#   $ dna run slurm <sjob-id> --generate-apptainer <profile> [OPTIONS] [--] <python-args>
#   $ dna run slurm <sjob-id> --ga <profile> [OPTIONS] [--] <python-args>
#
# Required arguments:
#   <profile>              HPC server profile (e.g., valeria, compute_canada, mamba).
#                          Must match an HPC server dotenv file (e.g., .env.valeria) in the
#                          super project configuration/hpc_server_profile/ directory.
#   <sjob-id>              Used to ID the slurm job and name the generated script.
#   <python-args>          The python command with flags.
#
# Optional flags:
#   --sif-path <path>      Path to the SIF file on the HPC server.
#                          (default: artifact/apptainer/<image>-slurm.sif)
#   --output-dir <path>    Directory for generated scripts.
#                          (default: artifact/apptainer/)
#   --print-only           Print apptainer exec command to stdout only
#                          (do not write script file).
#   --log-name <name>      Log file name (for script header comment).
#   -h | --help            Show this help message.
#
# Examples:
#   # Generate a standalone run script for Valeria:
#   $ dna run slurm NMO-001 --generate-apptainer valeria -- launcher/train.py --epochs=10
#
#   # Print the apptainer exec command to stdout (no file generated):
#   $ dna run slurm NMO-001 --ga valeria --print-only -- launcher/train.py
#
#   # Use a custom SIF path and output directory:
#   $ dna run slurm NMO-002 --ga compute_canada
#       --sif-path /scratch/user/my_image.sif
#       --output-dir /scratch/user/scripts
#       -- launcher/train.py --config=exp1
#
# Workflow:
#   1. dna build slurm --apptainer <profile>   → build Docker image + tar archive + build_sif.sh
#   2. dna save --apptainer <profile> slurm    → save tar archive + Apptainer artifacts
#   3. dna run slurm <sjob-id> --ga <profile>    → generate standalone run script
#   4. Transfer artifacts to HPC server
#   5. On HPC: bash build_sif.sh               → convert tar to SIF
#   6. On HPC: sbatch slurm_job.apptainer.<profile>.template.bash
#
# See also:
#   dna build slurm --apptainer <profile>
#   dna save --apptainer <profile> slurm
#   documentation/command/apptainer.md
#
# =================================================================================================
EOF
)

DOCUMENTATION_BUFFER_RUN_CI_TESTS_CMD=$( cat <<'EOF'
# =================================================================================================
# Run continuous integration tests container.
#
# Usage:
#   $ dna build ci-tests
#   $ dna run ci_tests [OPTIONS] [COMMAND [ARG...]]
#
# Options:
#   -e, --env stringArray        Set container environment variables
#   -w, --workdir string         Override path to workdir directory
#   -T, --no-TTY                 Disable pseudo-TTY allocation
#   -v, --volume stringArray     Bind mount a volume
#
# Note: Require executing `dna build ci-tests` first.
#
# =================================================================================================
EOF
)


# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dna_error_prefix="\033[1;31m[dna error]\033[0m"
test -n "$( declare -f dna::import_lib_and_dependencies )" || { echo -e "${dna_error_prefix} The DNA lib is not loaded!" 1>&2 && exit 1; }
test -n "$( declare -f n2st::print_msg )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
test -n "$( declare -f n2st::norlab_splash )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
test -d "${DNA_ROOT:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }
test -d "${DNA_LIB_PATH:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dna::run_command() {
    local service=""
    declare -i service_set=0
    declare -a remaining_args=()
    local apptainer_profile=""
    local original_command="$*"
    local line_format="${MSG_LINE_CHAR_BUILDER_LVL1}"
    local line_style="${MSG_LINE_STYLE_LVL2}"

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dna::command_help_menu "${DOCUMENTATION_BUFFER_RUN:?err}"
                exit 0
                ;;
            --help-slurm)
                dna::command_help_menu "${DOCUMENTATION_RUN_SLURM_CMD:?err}"
                exit 0
                ;;
            --help-slurm-apptainer)
                dna::command_help_menu "${DOCUMENTATION_RUN_SLURM_APPTAINER_CMD:?err}"
                exit 0
                ;;
            --help-ci-tests)
                dna::command_help_menu "${DOCUMENTATION_BUFFER_RUN_CI_TESTS_CMD:?err}"
                exit 0
                ;;
            --help-develop|--help-deploy)
                dna::command_help_menu "${DOCUMENTATION_BUFFER_RUN_DEVELOP_DEPLOY_CMD:?err}"
                exit 0
                ;;
            --ga|--generate-apptainer)

                if [[ -z "$2" ]]; then
                    dna::illegal_command_msg "run" "${original_command}" "The --generate-apptainer flag requires a <profile> argument (e.g., valeria, compute_canada, mamba).\n"
                    return 1
                fi
                apptainer_profile="$2"
                shift
                shift
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
                # Positional argument: consume next arg as DNA_SJOB_NAME only if it's not a flag or --
                if [[ -n "$1" && "$1" != "--" && "$1" != -* ]]; then
                    local DNA_SJOB_NAME="$1"
                    shift
                else
                    local DNA_SJOB_NAME=""
                fi
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

    # ....Load dependencies (part 1)...............................................................
    source "${DNA_LIB_PATH}/core/utils/load_super_project_config.bash" --no-execute || return 1

    # ....Set service..............................................................................
    # Check if a service was specified
    if [[ ${service_set} -eq 0 ]]; then
        # If no service was specified, check for offline deployment
        local offline_service
        if offline_service=$(dna::check_offline_deploy_service_discovery 2>/dev/null); then
            service="${offline_service}"
        else
            n2st::print_msg_error "Service is either unknown or not specified."
            dna::command_help_menu "${DOCUMENTATION_BUFFER_RUN:?err}"
            return 1
        fi
    elif [[ ${service_set} -ge 2 ]]; then
        # If service is set twice, it's an error
        dna::illegal_command_msg "run" "${original_command}" "Only one SERVICE can be specified.\n"
        return 1
    fi

    # Splash type: small, negative or big
    n2st::norlab_splash "${DNA_SPLASH_NAME_SMALL:?err}" "${DNA_GIT_REMOTE_URL}" "small"
    n2st::print_formated_script_header "run procedure" "${line_format}" "${line_style}"

    if [[ -n ${offline_service} ]]; then
      n2st::print_msg "Using offline deployment service: ${service}"
    fi

    # ....Load dependencies (part 2)...............................................................
    dna::load_super_project_configurations
    source "${DNA_LIB_PATH}/core/execute/run.ci_tests.bash" || return 1
    source "${DNA_LIB_PATH}/core/execute/up_and_attach.bash" || return 1
    source "${DNA_LIB_PATH}/core/execute/run.any.bash" || return 1
    source "${DNA_LIB_PATH}/core/execute/run.slurm.bash" || return 1
    source "${DNA_LIB_PATH}/core/execute/run.slurm.apptainer.bash" || return 1
    source "${DNA_LIB_EXEC_PATH}/build.all.bash" || return 1

    # ....Begin....................................................................................

    # Determine which run script to execute
    declare -i fct_exit_code
    if [[ "${service}" == "ci-tests" ]]; then
        n2st::print_msg "Running CI tests..."
        # (temporary hack) ToDo: NMO-692 feat: add a build ci-tests option to run.ci_tests.bash
        build_all_flag=()
        build_all_flag+=(--service-names "project-core-pre,project-core-user,project-core,project-ci-tests")
        #build_all_flag+=(-- --no-cache)
        dna::build_services "${build_all_flag[@]}"
        dna::run_ci_tests "${remaining_args[@]}"
        fct_exit_code=$?
    elif [[ "${service}" == "slurm" ]]; then
        if [[ -n "${apptainer_profile}" ]]; then
            n2st::print_msg "Generating Apptainer slurm artifacts (profile: ${apptainer_profile})..."
            dna::load_super_project_configurations
            dna::run_slurm_apptainer "$DNA_SJOB_NAME" --apptainer "${apptainer_profile}" "${remaining_args[@]}"
        else
            n2st::print_msg "Running slurm containers..."
            dna::run_slurm "$DNA_SJOB_NAME" "${remaining_args[@]}"
        fi
        fct_exit_code=$?
    elif [[ "${service}" == "develop" ]]; then
        n2st::print_msg "Running develop containers..."
        dna::run_any --service project-develop "${remaining_args[@]}"
        fct_exit_code=$?
    elif [[ "${service}" == "deploy" ]]; then
        n2st::print_msg "Running deploy containers..."
        dna::run_any --service project-deploy "${remaining_args[@]}"
        fct_exit_code=$?
    fi

    return $fct_exit_code
}
