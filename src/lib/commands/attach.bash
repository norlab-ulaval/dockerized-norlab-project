#!/bin/bash
# lib/commands/attach.bash

DOCUMENTATION_BUFFER_ATTACH=$( cat <<'EOF'
# =================================================================================================
# Attach to a running DNA containers
#
# Usage:
#   $ dna attach [OPTIONS] [SERVICE]
#
# Options:
#   -h | --help              Show this help message
#
# SERVICE:
#   develop                  Attach to develop service (default)
#   deploy                   Attach to deploy service
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
function dna::attach_command() {
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
                dna::command_help_menu "${DOCUMENTATION_BUFFER_ATTACH:?err}"
                exit 0
                ;;
            develop|deploy)
                service="$1"
                service_override+=1
                shift
                ;;
            *)
                # Check if it starts with -- (unknown option)
                if [[ "$1" == --* ]]; then
                    dna::unknown_subcommand_msg "attach" "$*"
                    exit 1
                fi
                # Otherwise it's an unknown service
                dna::illegal_command_msg "attach" "${original_command}" "Unknown SERVICE: $1. Valid services are: develop, deploy.\n"
                return 1
                ;;
        esac
    done

    if [[ ${service_override} -ge 2 ]]; then
        # If service is set twice, it's an error
        dna::illegal_command_msg "attach" "${original_command}" "Only one SERVICE can be specified.\n"
        return 1
    fi

    # Splash type: small, negative or big
    n2st::norlab_splash "${DNA_SPLASH_NAME_SMALL:?err}" "${DNA_GIT_REMOTE_URL}" "small"
    n2st::print_formated_script_header "attach procedure" "${line_format}" "${line_style}"

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
    dna::up_and_attach --no-up "${docker_compose_exec_flag[@]}"
    fct_exit_code=$?
    if [[ ${fct_exit_code} -eq 0 ]]; then
      n2st::print_msg "Detached. ${MSG_DIMMED_FORMAT}Container ${DN_CONTAINER_NAME} is running in background${MSG_END_FORMAT}"
    fi
    return $fct_exit_code
}
