#!/bin/bash
# lib/commands/config.bash

DOCUMENTATION_BUFFER_CONFIG=$( cat <<'EOF'
# =================================================================================================
# Show Docker Compose configuration file.
#
# Usage:
#   $ dna config [OPTIONS] MODE [--] [DOCKER_CONFIG_FLAGS|DOCKER_BAKE_FLAGS]
#
# Options:
#   --bake                 Use 'docker buildx bake' instead of 'docker compose config'
#   --compose-to-bake      Print the compose file converted to bake format
#   -q | --quiet           Skip dna messages, only print docker command output
#   --help, -h             Show this help message
#
# Modes:
#   build-core             Core only (pre, user, final) native build config
#   build-core-ma          Core only (pre, user, final) multi-architecture build config
#   build                  All native build config
#   build-ma               All multi-architecture build config
#   dev [platform]         Development mode
#   deploy [platform]      Deployment mode
#   ci-tests               CI tests mode
#   slurm                  SLURM mode
#   release [platform]     Release mode
#
# Platforms:
#   darwin                 macOS
#   linux                  Linux
#   jetson                 NVIDIA Jetson
#
# Docker [config|bake] flags options:
#   'dna config' use 'docker compose config' or 'docker buildx bake' command under the hood, so it
#   can consume their respective option flags. Check their respective help documentation for
#   available options: '$ docker [compose config|buildx bake] --help'
#
# =================================================================================================
EOF
)


# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dna_error_prefix="\033[1;31m[dna error]\033[0m"
test -n "$( declare -f dna::import_lib_and_dependencies )" || { echo -e "${dna_error_prefix} The DNA lib is not loaded!" 1>&2 && exit 1; }
test -n "$( declare -f n2st::print_msg )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
test -d "${DNA_ROOT:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }
test -d "${DNA_LIB_PATH:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dna::config_command() {
    local initial_cwd
    initial_cwd=$(pwd)
    local mode=""
    local platform=""
    local remaining_args=()
    local docker_cmd=config
    local dna_quiet=false
    local initial_dna_cmd="$*"

    if [[ -z "$1" ]]; then
        dna::command_help_menu "${DOCUMENTATION_BUFFER_CONFIG:?err}"
        exit 1
    fi

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            build-core|build-core-ma|build|build-ma|dev|deploy|ci-tests|slurm|release)
                mode="$1"
                shift
                ;;
            darwin|linux|jetson)
                platform="$1"
                shift
                ;;
            --bake)
                docker_cmd=bake
                shift
                ;;
            --compose-to-bake)
                docker_cmd=build
                shift
                ;;
            --quiet|-q)
              dna_quiet=true
              shift
              ;;
            --help|-h)
                dna::command_help_menu "${DOCUMENTATION_BUFFER_CONFIG:?err}"
                exit 0
                ;;
            --) # no more option
                shift
                remaining_args=("$@")
                break
                ;;
            *)
                remaining_args=("$@")
                break
                ;;
        esac
    done

    if [[ -z "${mode}" ]]; then
        dna::illegal_command_msg "config" "$initial_dna_cmd" "Unknown mode!"
        exit 1
    fi

    # ....Load dependencies........................................................................
    # Load super project configuration
    if [[ ${dna_quiet} == true ]]; then
      source "${DNA_LIB_PATH}/core/utils/load_super_project_config.bash" >/dev/null || return 1
    else
      source "${DNA_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1
    fi

    # ....Begin....................................................................................
#    n2st::set_is_teamcity_run_environment_variable

    # Determine which compose file to use
    local compose_file=""
    local services=()

    declare -a docker_command


    if [[ "${mode}" =~ ^(build-core|build-core-ma)$ ]]; then
        services+=(project-core-pre project-core-user project-core)
        if [[ "${mode}" == build-core ]]; then
          compose_file="docker-compose.build.native.yaml"
        elif [[ "${mode}" == build-core-ma ]]; then
          compose_file="docker-compose.build.multiarch.yaml"
        fi
    elif [[ "${mode}" =~ ^(build|build-ma)$ ]]; then
        if [[ "${mode}" == build ]]; then
          compose_file="docker-compose.build.native.yaml"
        elif [[ "${mode}" == build-ma ]]; then
          compose_file="docker-compose.build.multiarch.yaml"
        fi
    elif [[ "${mode}" == dev ]]; then
        services+=(project-develop)
        if [[ "${platform}" == darwin ]]; then
            compose_file="docker-compose.run.darwin.yaml"
        elif [[ "${platform}" == jetson ]]; then
            compose_file="docker-compose.run.jetson.yaml"
        else
            compose_file="docker-compose.run.linux-x86.yaml"
        fi
    elif [[ "${mode}" == deploy ]]; then
        services+=(project-deploy)
        if [[ "${platform}" == darwin ]]; then
            compose_file="docker-compose.run.darwin.yaml"
        elif [[ "${platform}" == jetson ]]; then
            compose_file="docker-compose.run.jetson.yaml"
        else
            compose_file="docker-compose.run.linux-x86.yaml"
        fi
    elif [[ "${mode}" == ci-tests ]]; then
        services+=(project-ci-tests)
        compose_file="docker-compose.run.ci-tests.yaml"
    elif [[ "${mode}" == slurm ]]; then
        services+=(project-slurm)
        compose_file="docker-compose.run.slurm.yaml"
    elif [[ "${mode}" == release ]]; then
        n2st::print_msg_warning "Command ${MSG_DIMMED_FORMAT}dna config release${MSG_END_FORMAT} is not released yet, stay tuned!\n" && exit 0 # (CRITICAL) ToDo: on task end >> delete this line <--
        services+=(project-release)
        compose_file="docker-compose.build.multiarch.yaml"
    fi

    # shellcheck disable=SC2207
    compose_override=($( dna::generate_super_project_compose_override_files_flags ".dockerized_norlab/configuration/overrides" "${compose_file}" ) )

    if [[ ${docker_cmd} == bake ]]; then
      if [[ ${mode} =~ ^(dev|deploy|ci-tests|slurm|release) ]]; then
        n2st::print_msg_warning "Using ${MSG_DIMMED_FORMAT}--bake${MSG_END_FORMAT} flag with non-build mode ${MSG_DIMMED_FORMAT}${mode}${MSG_END_FORMAT} is pointless. Bake only handle the ${MSG_DIMMED_FORMAT}build${MSG_END_FORMAT} attribute in compose config file." && return 0
      fi
      cd "${DNA_LIB_PATH}/core/docker/" || return 1
      docker_command=(buildx bake --file "${compose_file}" "${compose_override[@]}" --print)
    elif [[ ${docker_cmd} == config ]]; then
      docker_command=(compose --file "${DNA_LIB_PATH}/core/docker/${compose_file}" "${compose_override[@]}" config)
    elif [[ ${docker_cmd} == build ]]; then
      if [[ ${mode} =~ ^(dev|deploy|ci-tests|slurm|release) ]]; then
        n2st::print_msg_warning "Using ${MSG_DIMMED_FORMAT}--compose-to-bake${MSG_END_FORMAT} flag with non-build mode ${MSG_DIMMED_FORMAT}${mode}${MSG_END_FORMAT} is pointless. Bake only handle the ${MSG_DIMMED_FORMAT}build${MSG_END_FORMAT} attribute in compose config file." && return 0
      fi
      docker_command=(compose --file "${DNA_LIB_PATH}/core/docker/${compose_file}" "${compose_override[@]}" build --print)
    fi

    # Execute docker-compose config command
    if [[ ${dna_quiet} == false ]]; then
      n2st::print_msg "Showing ${MSG_DIMMED_FORMAT}${mode}${MSG_END_FORMAT} mode configuration from ${MSG_DIMMED_FORMAT}${compose_file}${MSG_END_FORMAT}...\n"
    fi
    docker "${docker_command[@]}" "${remaining_args[@]}" "${services[@]}"
    fct_exit_code=$?

    # ....Teardown.................................................................................
    cd "${initial_cwd}" || return 1
    return $fct_exit_code
}

