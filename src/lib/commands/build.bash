#!/bin/bash
# lib/commands/build.bash

DOCUMENTATION_BUFFER_BUILD=$( cat <<'EOF'
# =================================================================================================
# Build project Docker images
#
# Notes: build all services for host native architecture by default
#
# Usage:
#   $ dnp build [OPTIONS] [SERVICE] [-- <any-docker-argument>]
#
# Options:
#   --multiarch                           Build all service for multiple architectures
#                                          (require a configured docker buildx multiarch builder)
#   --force-push-project-core             Pull/push from/to Dockerhub sequentialy
#                                          (instead of building images from the local image store).
#   --push                                Push deploy image (only valid with SERVICE=deploy)
#   --help, -h                            Show this help message
#
# SERVICE:
#   develop                               Build develop images only
#   deploy [--push]                       Build deploy images only
#   ci-tests                              Build CI tests images only
#   slurm                                 Build slurm images only
#
# =================================================================================================
EOF
)

# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
test -n "$( declare -f dnp::import_lib_and_dependencies )" || { echo -e "${dnp_error_prefix} The DNP lib is not loaded!" ; exit 1 ; }
test -n "$( declare -f n2st::print_msg )" || { echo -e "${dnp_error_prefix} The N2ST lib is not loaded!" ; exit 1 ; }
test -d "${DNP_ROOT:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }
test -d "${DNP_LIB_PATH:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dnp::build_command() {

    # ....Set env variables (pre cli)).............................................................
    local multiarch=false
    local force_push_project_core=false
    local service=""
    local push_deploy=false
    local remaining_args=()
    local original_command="$*"

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --multiarch)
                multiarch=true
                shift
                ;;
            --force-push-project-core)
                force_push_project_core=true
                shift
                ;;
            --push)
                push_deploy=true
                shift
                ;;
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_BUILD}"
                exit 0
                ;;
            --) # no more option
                shift
                remaining_args+=("$@")
                break
                ;;
            ci-tests|deploy|develop|slurm)
                # If service is already set, it's an error
                if [[ -n "${service}" ]]; then
                    dnp::illegal_command_msg "build" "${original_command}" "Only one SERVICE can be specified.\n"
                    return 1
                fi
                service="$1"
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

    # ....Load dependencies........................................................................
    source "${DNP_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1
    source "${DNP_LIB_EXEC_PATH}/build.all.bash" || return 1
    source "${DNP_LIB_EXEC_PATH}/build.all.multiarch.bash" || return 1
    source "${DNP_LIB_EXEC_PATH}/build.deploy.bash" || return 1

    # ....Flag check...............................................................................
    if [[ "${service}" != "deploy" ]] && [[ "${push_deploy}" == true ]]; then
      dnp::illegal_command_msg "build" "${original_command}" "The --push flag can only be used with SERVICE=deploy.\n"
      return 1
    fi

    if [[ "${service}" == "deploy" ]] && [[ "${multiarch}" == true ]] && [[ "${push_deploy}" == true ]]; then
      # (Priority) ToDo: NMO-680 feat: improve project-deploy logic
      dnp::illegal_command_msg "build" "${original_command}" "The build and push multiarch deploy image feature is not released yet!\nIt only affect ${MSG_DIMMED_FORMAT}dnp build --multiarch --push deploy${MSG_END_FORMAT} flags combinaison.\nIssue NMO-680 feat: improve project-deploy logic\n"
      return 1
    fi

    # ....Set env variables (post cli).............................................................
    declare -a build_flag
    declare -a deploy_flag
    # build_flag+=("--msg-line-level" " ")

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

    # ....Begin....................................................................................
    if [[ "${service}" == "deploy" ]] && [[ "${multiarch}" == false ]]; then
        # Case: Deploy
        header_footer_name="deploy images (${architecture}) build procedure"
        if [[ "${push_deploy}" == true ]]; then
          deploy_flag+=("--push")
        fi
        n2st::print_formated_script_header "${header_footer_name}" "${MSG_LINE_CHAR_BUILDER_LVL1}"
        dnp::build_project_deploy_service "${deploy_flag[@]}" "${build_flag[@]}" "${remaining_args[@]}"
        fct_exit_code=$?
        n2st::print_formated_script_footer "${header_footer_name}" "${MSG_LINE_CHAR_BUILDER_LVL1}"
    else
      # Case: general
      if [[ "${service}" == "ci-tests" ]]; then
          header_footer_name="CI tests images (${architecture}) build procedure"
          build_flag+=("--service-names" "project-core,project-ci-tests,project-ci-tests-no-gpu")
      elif [[ "${service}" == "slurm" ]]; then
          header_footer_name="slurm images (${architecture}) build procedure"
          build_flag+=("--service-names" "project-core,project-slurm,project-slurm-no-gpu")
      elif [[ "${service}" == "develop" ]]; then
          header_footer_name="develop images (${architecture}) build procedure"
          build_flag+=("--service-names" "project-core,project-develop")
      elif [[ "${service}" == "deploy" ]]; then
          header_footer_name="develop images (${architecture}) build procedure"
          build_flag+=("--service-names" "project-core,project-deploy")
      else
          header_footer_name="all images (${architecture}) build procedure"
      fi

      n2st::print_formated_script_header "${header_footer_name}" "${MSG_LINE_CHAR_BUILDER_LVL1}"
      if [[ "${multiarch}" == true ]]; then
          dnp::build_services_multiarch "${build_flag[@]}" "${remaining_args[@]}"
          fct_exit_code=$?
      else
          dnp::build_services  "${build_flag[@]}" "${remaining_args[@]}"
          fct_exit_code=$?
      fi
      n2st::print_formated_script_footer "${header_footer_name}" "${MSG_LINE_CHAR_BUILDER_LVL1}"
    fi

    return $fct_exit_code
}
