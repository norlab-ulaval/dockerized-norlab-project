#!/bin/bash
# lib/commands/build.bash

DOCUMENTATION_BUFFER_BUILD=$( cat <<'EOF'
# =================================================================================================
# Build project Docker images
#
# Usage:
#   $ dnp build [OPTIONS] [SERVICE] [-- <any-docker-argument>]
#
# Options:
#   --multiarch                   Build services for multiple architectures (require a configured
#                                  docker buildx multiarch builder)
#   --online-build                Build image sequentialy by pushing/pulling intermediate images
#                                  from Dockerhub
#   --save DIRPATH                Save built image to directory (develop or deploy services only)
#   --push                        Push image to dockerhub (deploy services only)
#   --help, -h                    Show this help message
#
#
# SERVICE:
#   develop                       Build develop images only
#   deploy                        Build deploy images only
#   ci-tests                      Build CI tests images only
#   slurm                         Build slurm images only
#   release                       Build release images only
#
# Notes:
#   - build all services for host native architecture by default
#   - build offline from the local image store by default
#
# =================================================================================================
EOF
)

# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
test -n "$( declare -f dnp::import_lib_and_dependencies )" || { echo -e "${dnp_error_prefix} The DNP lib is not loaded!" 1>&2 && exit 1; }
test -n "$( declare -f n2st::print_msg )" || { echo -e "${dnp_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
test -d "${DNP_ROOT:?err}" || { echo -e "${dnp_error_prefix} library load error!" 1>&2 && exit 1; }
test -d "${DNP_LIB_PATH:?err}" || { echo -e "${dnp_error_prefix} library load error!" 1>&2 && exit 1; }

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dnp::build_command() {

    if ! dnp::is_online; then
      n2st::print_msg_error "Be advised, you are currently, offline. Executing ${MSG_DIMMED_FORMAT}dnp build${MSG_END_FORMAT} require internet connection."
      return 1
    fi

    # ....Set env variables (pre cli)).............................................................
    local multiarch=false
    local force_push_project_core=false
    local service=""
    local push_deploy=false
    local save_dirpath=""
    local remaining_args=()
    local original_command="$*"
    local line_format="${MSG_LINE_CHAR_BUILDER_LVL1}"
    local line_style="${MSG_LINE_STYLE_LVL2}"


    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --multiarch)
                multiarch=true
                shift
                ;;
            --online-build)
                force_push_project_core=true
                shift
                ;;
            --push)
                push_deploy=true
                shift
                ;;
            --save)
                if [[ -z "$2" ]]; then
                    dnp::illegal_command_msg "build" "${original_command}" "The --save flag requires a DIRPATH argument.\n"
                    return 1
                fi
                save_dirpath="$2"
                shift 2
                ;;
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_BUILD:?err}"
                exit 0
                ;;
            --) # no more option
                shift
                remaining_args+=("$@")
                break
                ;;
            develop|deploy|ci-tests|slurm|release)
                # If service is already set, it's an error
                if [[ -n "${service}" ]]; then
                    dnp::illegal_command_msg "build" "${original_command}" "Only one SERVICE can be specified.\n"
                    return 1
                fi
                service="$1"
                if [[ "${service}" == "release" ]]; then
                    n2st::print_msg "Command ${MSG_DIMMED_FORMAT}dnp build release${MSG_END_FORMAT} is not released yet, stay tuned!\n ... yeah I know, an un-released release function" && exit 0 # (CRITICAL) ToDo: on task end >> delete this line <--
                    return 1
                fi
                shift
                ;;
            *)
                # Check if it starts with -- (unknown option)
                if [[ "$1" == --* ]]; then
                    dnp::unknown_subcommand_msg "build" "$*"
                    exit 1
                fi
                # Otherwise it's an unknown service
                dnp::illegal_command_msg "build" "${original_command}" "Unknown SERVICE: $1. Valid services are: ci-tests, deploy, develop, slurm.\n"
                return 1
                ;;
        esac
    done

    # ....Set env variables (post cli)...............................................................
    declare -a build_flag=()
    declare -a deploy_flag=()

    build_flag+=("--msg-line-level" "${line_format}")

    local architecture="native"
    if [[ "${multiarch}" == true ]]; then
      architecture="multiarch"
      if [[ "${force_push_project_core}" == false ]]; then
        build_flag+=("--no-force-push-project-core")
      fi
    else
      if [[ "${force_push_project_core}" == true ]]; then
        build_flag+=("--force-push-project-core")
      fi
    fi

    if [[ "${service}" == "deploy" ]]; then
        # Case: Deploy
        header_footer_name="deploy images ${architecture} build procedure"
        if [[ "${multiarch}" == true ]]; then
          deploy_flag+=("--multiarch")
        fi
        if [[ "${push_deploy}" == true ]]; then
          deploy_flag+=("--push")
        fi
    else
      # Case: general
      if [[ "${service}" == "ci-tests" ]]; then
          header_footer_name="CI tests images ${architecture} build procedure"
          build_flag+=("--service-names" "project-core,project-ci-tests,project-ci-tests-no-gpu")
      elif [[ "${service}" == "slurm" ]]; then
          header_footer_name="slurm images ${architecture} build procedure"
          build_flag+=("--service-names" "project-core,project-slurm,project-slurm-no-gpu")
      elif [[ "${service}" == "develop" ]]; then
          header_footer_name="develop images ${architecture} build procedure"
          build_flag+=("--service-names" "project-core,project-develop")
      else
          header_footer_name="all images ${architecture} build procedure"
      fi
    fi

    # Splash type: small, negative or big
    n2st::norlab_splash "${DNP_SPLASH_NAME_SMALL}" "${DNP_GIT_REMOTE_URL}" "small"
    n2st::print_formated_script_header "${header_footer_name}" "${line_format}" "${line_style}"

    # ....Flag check...............................................................................
    if [[ "${service}" != "deploy" ]] && [[ "${push_deploy}" == true ]]; then
      dnp::illegal_command_msg "build" "${original_command}" "The ${MSG_DIMMED_FORMAT}--push${MSG_END_FORMAT} flag can only be used with SERVICE=deploy.\n"
      return 1
    fi

    if [[ -n "${save_dirpath}" ]]; then
      if [[ "${service}" != "develop" && "${service}" != "deploy" ]]; then
        dnp::illegal_command_msg "build" "${original_command}" "The ${MSG_DIMMED_FORMAT}--save${MSG_END_FORMAT} flag can only be used with SERVICE=develop or SERVICE=deploy.\n"
        return 1
      fi
      if [[ ! -d "${save_dirpath}" ]]; then
        n2st::print_msg_error "The DIRPATH does not exist: ${MSG_DIMMED_FORMAT}${save_dirpath}${MSG_END_FORMAT}\n"
        return 1
      fi
    fi

    # ....Load dependencies........................................................................
    source "${DNP_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1
    source "${DNP_LIB_EXEC_PATH}/build.all.bash" || return 1
    source "${DNP_LIB_EXEC_PATH}/build.all.multiarch.bash" || return 1
    source "${DNP_LIB_EXEC_PATH}/build.deploy.bash" || return 1

    # ....Begin....................................................................................
    if [[ "${service}" == "deploy" ]]; then
        dnp::build_project_deploy_service "${deploy_flag[@]}" "${build_flag[@]}" "${remaining_args[@]}"
        fct_exit_code=$?
    else
      if [[ "${multiarch}" == true ]]; then
          dnp::build_services_multiarch "${build_flag[@]}" "${remaining_args[@]}"
          fct_exit_code=$?
      else
          dnp::build_services  "${build_flag[@]}" "${remaining_args[@]}"
          fct_exit_code=$?
      fi
    fi

    # ....Post-build save if requested.............................................................
    if [[ -n "${save_dirpath}" && $fct_exit_code -eq 0 ]]; then
        n2st::print_msg "Executing save command as requested"
        source "${DNP_LIB_PATH}/commands/save.bash" || {
            n2st::print_msg_error "Failed to load save command"
            return 1
        }
        dnp::save_command "${save_dirpath}" "${service}" || {
            n2st::print_msg_error "Failed to save image"
            return 1
        }
    fi

    # ....Teardown.................................................................................
    n2st::print_formated_script_footer "${header_footer_name}" "${line_format}" "${line_style}"
    return $fct_exit_code
}
