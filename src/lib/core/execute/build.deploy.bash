#!/bin/bash
DOCUMENTATION_BUILD_DEPLOY=$(
                              cat << 'EOF'
# =================================================================================================
# Build deploy images specified in docker-compose.project.build.native.yaml
#
# Usage:
#   $ bash build.deploy.bash [--push] [<any-build.all-argument>] [--] [<any-docker-flag>]
#
# Arguments:
#   --push
#   -h | --help
#
# Positional argument:
#   <any-build.all-argument>               (Optional) Any build.all.bash arguments
#   <any-docker-flag>                      (Optional) Any docker flag
#
# =================================================================================================
EOF
)
# (Priority) ToDo: unit-test of flag option

# ToDo: move the help fct near the script/fct menu
function show_help() {
  # (NICE TO HAVE) ToDo: refactor as a n2st fct (ref NMO-583)
  echo -e "${MSG_DIMMED_FORMAT}"
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo -e "$0 --help"
  # Strip shell comment char `#` and both lines
  echo -e "${DOCUMENTATION_BUILD_DEPLOY}" | sed '/\# ====.*/d' | sed 's/^\# //' | sed 's/^\#//'
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo -e "${MSG_END_FORMAT}"
}

function dnp::build_project_deploy_service() {
  local tmp_cwd
  tmp_cwd=$(pwd)

  # ....Set env variables (pre cli)................................................................
  local push_deploy_image=false
  local multiarch=false
  local build_core_exit_code
  local build_deploy_exit_code
  declare -a build_flag=()
  declare -a push_flag=()
  declare -a remaining_args=()

  # ....cli........................................................................................
  while [ $# -gt 0 ]; do

    case $1 in
      --push)
        push_deploy_image=true
        shift
        ;;
      --multiarch)
        multiarch=true
        shift
        ;;
      -h | --help)
        clear
        show_help
        exit
        ;;
      *) # Default case
        remaining_args=("$@")
        break
        ;;
    esac

  done

  # ....Set env variables (post cli)...............................................................
  if [[ ${push_deploy_image} != true ]]; then
    n2st::print_msg "Won't push the deploy image to hub, use flag ${MSG_DIMMED_FORMAT}--push${MSG_END_FORMAT} to push it."
  fi

  # ....Build stage..................................................................................
  {
    build_flag+=("--service-names" "project-core,project-deploy")
    if [[ ${push_deploy_image} == true ]]; then
      build_flag+=("--force-push-project-core")
    fi
    if [[ "${multiarch}" == true ]]; then
      dnp::build_services_multiarch "${build_flag[@]}" "${remaining_args[@]}"
      build_core_exit_code=$?
    else
      dnp::build_services "${build_flag[@]}" "${remaining_args[@]}"
      build_core_exit_code=$?
    fi
  }

  # ....Push deploy stage............................................................................
  build_flag+=("--service-names" "project-deploy")
  if [[ ${push_deploy_image} == true ]]; then
    remaining_args+=("--push")
  fi
  if [[ "${multiarch}" == true ]]; then
    dnp::build_services_multiarch "${build_flag[@]}" "${remaining_args[@]}"
    build_deploy_exit_code=$?
  else
    dnp::build_services "${build_flag[@]}" "${remaining_args[@]}"
    build_deploy_exit_code=$?
  fi

  # ....Teardown...................................................................................
  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return $((build_core_exit_code + build_deploy_exit_code))
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  # ....Source project shell-scripts dependencies..................................................
  script_path="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  script_path_parent="$(dirname "${script_path}")"
  if [[ -z $( declare -f dnp::import_lib_and_dependencies)  ]]; then
    source "${script_path_parent}/../utils/import_dnp_lib.bash" || exit 1
    source "${script_path_parent}/../utils/execute_compose.bash" || exit 1
  fi
  if [[ -z ${SUPER_PROJECT_ROOT} ]]; then
    source "${script_path_parent}/../utils/load_super_project_config.bash" || exit 1
  fi
  source "${script_path_parent}/build.all.bash" || exit 1
  source "${DNP_LIB_EXEC_PATH}/build.all.multiarch.bash" || exit 1

  # ....Execute....................................................................................
  if [[ "${DNP_CLEAR_CONSOLE_ACTIVATED}" == "true" ]]; then
    clear
  fi
  n2st::norlab_splash "${DNP_GIT_NAME} (${DNP_PROMPT_NAME})" "${DNP_GIT_REMOTE_URL}"
  n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  dnp::build_project_deploy_service "$@"
  fct_exit_code=$?
  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  exit "${fct_exit_code}"
else
  # This script is being sourced, ie: __name__="__source__"

  # ....Pre-condition..............................................................................
  dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
  test -n "$( declare -f dnp::import_lib_and_dependencies)"  || {
                                                                  echo -e "${dnp_error_prefix} The DNP lib is not loaded!"
                                                                                                                             exit 1
  }
  test -n "$( declare -f n2st::print_msg)"  || {
                                                 echo -e "${dnp_error_prefix} The N2ST lib is not loaded!"
                                                                                                             exit 1
  }
  test -n "${SUPER_PROJECT_ROOT}" || {
                                       echo -e "${dnp_error_prefix} The super project DNP configuration is not loaded!"
                                                                                                                          exit 1
  }
fi
