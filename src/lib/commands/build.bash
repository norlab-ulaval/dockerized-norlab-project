#!/bin/bash
# lib/commands/build.bash

DOCUMENTATION_BUFFER_BUILD=$( cat <<'EOF'
# =================================================================================================
# Build Docker images for the project
# Notes: build all services for host native architecture by default
#
# Usage:
#   $ dnp build [OPTIONS] [-- <any-docker-argument>]
#
# Options:
#   --multiarch                           Build all service for multiple architectures
#                                          (require a configured docker buildx multiarch builder)
#   --force-push-project-core             Build images from the local image store
#   --develop                             Build develop images only
#   --ci-tests                            Build CI tests images only
#   --slurm                               Build slurm images only
#   --deploy [--push-deploy-image]        Build deploy images only
#   --help, -h                            Show this help message
#
# =================================================================================================
EOF
)

test -d "${DNP_ROOT:?err}" || { echo "The DNP lib load error!" ; exit 1 ; }
test -d "${DNP_LIB_PATH:?err}" || { echo "The DNP lib load error!" ; exit 1 ; }

function dnp::build_command() {

    # ....Set env variables (pre cli)).............................................................
    local multiarch=false
    local force_push_project_core=false
    local ci_tests=false
    local deploy=false
    local develop=false
    local slurm=false
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
            --ci-tests)
                ci_tests=true
                shift
                ;;
            --deploy)
                deploy=true
                shift
                ;;
            --push-deploy-image)
                push_deploy=true
                shift
                ;;
            --develop)
                develop=true
                shift
                ;;
            --slurm)
                slurm=true
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
            *)
                dnp::unknown_subcommand_msg "build" "$*"
                exit 1
                ;;
        esac
    done

    # ....Load dependencies........................................................................
    source "${DNP_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1
    source "${DNP_LIB_EXEC_PATH}/build.all.bash" || return 1
    source "${DNP_LIB_EXEC_PATH}/build.all.multiarch.bash" || return 1
    source "${DNP_LIB_EXEC_PATH}/build.deploy.bash" || return 1

    # ....Flag check...............................................................................
    if [[ "${deploy}" == false ]] && [[ "${push_deploy}" == true ]]; then
      dnp::illegal_command_msg "build" "${original_command}" "The --push-deploy-image flag can only be used in combination with --deploy.\n"
      return 1

    fi
    if [[ "${deploy}" == true ]] && [[ "${multiarch}" == true ]]; then
      # (Priority) ToDo: NMO-680 feat: improve project-deploy logic
      dnp::illegal_command_msg "build" "${original_command}" "The build multiarch deploy image feature is not released yet!\nUse ${MSG_DIMMED_FORMAT}dnp build --multiarch${MSG_END_FORMAT} in the mean time to build multiarch project-deploy images.\nIssue NMO-680 feat: improve project-deploy logic\n"
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
    if [[ "${deploy}" == true ]]; then
        # Case: Deploy
        header_footer_name="deploy images (${architecture}) build procedure"
        if [[ "${push_deploy}" == true ]]; then
          deploy_flag+=("--push-deploy-image")
        fi

        n2st::print_formated_script_header "${header_footer_name}" "${MSG_LINE_CHAR_BUILDER_LVL1}"
        dnp::build_project_deploy_service "${deploy_flag[@]}" "${build_flag[@]}" "${remaining_args[@]}"
        fct_exit_code=$?
        n2st::print_formated_script_footer "${header_footer_name}" "${MSG_LINE_CHAR_BUILDER_LVL1}"
    else
      # Case: general
      if [[ "${ci_tests}" == true ]]; then
          header_footer_name="CI tests images (${architecture}) build procedure"
          build_flag+=("--service-names" "project-core,project-ci-tests,project-ci-tests-no-gpu")
      elif [[ "${slurm}" == true ]]; then
          header_footer_name="slurm images (${architecture}) build procedure"
          build_flag+=("--service-names" "project-core,project-slurm,project-slurm-no-gpu")
      elif [[ "${develop}" == true ]]; then
          header_footer_name="develop images (${architecture}) build procedure"
          build_flag+=("--service-names" "project-core,project-develop")
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

