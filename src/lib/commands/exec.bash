#!/bin/bash
# lib/commands/exec.bash

DOCUMENTATION_BUFFER_EXEC=$( cat <<'EOF'
# =================================================================================================
# Execute command and arguments in a running DNA containers
#
# Usage:
#   $ dna exec [OPTIONS] [SERVICE] [--] COMMAND [ARGS...]
#
# Options:
#   -e, --env stringArray    Set container environment variables
#   -w, --workdir string     Override path to workdir directory
#   -T, --no-TTY             Disable pseudo-TTY allocation
#   --detach                 Execute COMMAND in the background
#   --dry-run                (Require --detach flag)
#   -h | --help              Show this help message
#
# SERVICE:
#   develop                  Execute in develop service (default)
#   deploy                   Execute in deploy service
#
# Positional argument:
#   command & arguments    Any command to be executed inside the docker container (default: bash)
#
# Example:
#   $ dna exec --workdir "/" -- bash -c 'tree -L 1 -a $(pwd) && echo \"Hello-world\"'
#
# Notes:
#   • Running 'dna exec' with no COMMAND is equivalent to executing 'docker compose attach'
#     which execute the default container command.
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
function dna::exec_command() {
    declare -a remaining_args=()
    declare -a docker_compose_exec_flag=()
    local service="develop"
    declare -i service_override=0
    local original_command="$*"
    local line_format="${MSG_LINE_CHAR_BUILDER_LVL1}"
    local line_style="${MSG_LINE_STYLE_LVL2}"


    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dna::command_help_menu "${DOCUMENTATION_BUFFER_EXEC:?err}"
                exit 0
                ;;
            --detach|--dry-run|-T|--no-TTY) # Assume its a docker compose flag
                docker_compose_exec_flag+=("$1")
                if [[ ${1} == "--dry-run" ]]; then
                  docker_compose_exec_flag+=("--detach")
                fi
                shift
                ;;
            -e|--env|-w|--workdir) # Assume its a docker compose flag
                docker_compose_exec_flag+=("$1" "$2")
                shift
                shift
                ;;
            develop|deploy)
                service="$1"
                service_override+=1
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

    if [[ ${service_override} -ge 2 ]]; then
        # If service is set twice, it's an error
        dna::illegal_command_msg "exec" "${original_command}" "Only one SERVICE can be specified.\n"
        return 1
    fi

    # Splash type: small, negative or big
    n2st::norlab_splash "${DNA_SPLASH_NAME_SMALL:?err}" "${DNA_GIT_REMOTE_URL}" "small"
    n2st::print_formated_script_header "exec procedure" "${line_format}" "${line_style}"

    # ....Load dependencies........................................................................
    source "${DNA_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1
    source "${DNA_LIB_PATH}/core/execute/up_and_attach.bash" || return 1

    # ....Set service..............................................................................
    # Check if a service was specified
    if [[ ${service_override} -eq 0 ]]; then
        # If no service was explicitly specified, check for offline deployment
        local offline_service
        if offline_service=$(dna::check_offline_deploy_service_discovery 2>/dev/null); then
            service="${offline_service}"
            n2st::print_msg "Using offline deployment service: ${service}"
        fi
    fi

    docker_compose_exec_flag+=("--service" "${service}")

    # ....Begin....................................................................................
    dna::up_and_attach --no-up "${docker_compose_exec_flag[@]}" "${remaining_args[@]}"
    fct_exit_code=$?
    if [[ ${fct_exit_code} -eq 0 ]]; then
      n2st::print_msg "Detached. ${MSG_DIMMED_FORMAT}Container ${DN_CONTAINER_NAME} is running in background${MSG_END_FORMAT}"
    fi
    return $fct_exit_code
}
