#!/bin/bash
DOCUMENTATION_BUILD_DEPLOY=$( cat <<'EOF'
# =================================================================================================
# Build deploy images specified in docker-compose.project.build.native.yaml
#
# Usage:
#   $ bash build.deploy.bash [--push-deploy-image] [<any-build.all-argument>] [--] [<any-docker-flag>]
#
# Arguments:
#   --push-deploy-image
#   -h | --help
#
# Positional argument:
#   <any-build.all-argument>               (Optional) Any build.all.bash arguments
#   <any-docker-flag>                      (Optional) Any docker flag
#
# =================================================================================================
EOF
)

# (Priority) ToDo: unit-test for flag option

if [[ "${DNP_CLEAR_CONSOLE_ACTIVATED}" == "true" ]]; then
  clear
fi
pushd "$(pwd)" >/dev/null || exit 1

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


# ....Set env variables (pre cli)................................................................
declare -a remaining_args
push_deploy_image=false

# ....cli..........................................................................................
while [ $# -gt 0 ]; do

  case $1 in
    --push-deploy-image)
      push_deploy_image=true
      shift # Remove argument (--push-deploy-image)
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
# Add env var


# ....Source project shell-scripts dependencies..................................................
script_path="$(realpath "${BASH_SOURCE[0]:-'.'}")"
script_path_parent="$(dirname "${script_path}")"
if [[ ! $( dnp::is_lib_loaded 2>/dev/null >/dev/null )  ]]; then
  source "${script_path_parent}/../utils/import_dnp_lib.bash" || exit 1
  source "${script_path_parent}/../utils/execute_compose.bash" || exit 1
fi
if [[ -z ${SUPER_PROJECT_ROOT} ]]; then
  source "${script_path_parent}/../utils/load_super_project_config.bash" || exit 1
fi

source "${script_path_parent}/build.all.bash" || exit 1

# ....Execute....................................................................................
if [[ "${DNP_CLEAR_CONSOLE_ACTIVATED}" == "true" ]]; then
  clear
fi

# ====Begin========================================================================================
n2st::norlab_splash "${PROJECT_GIT_NAME} (${DNP_PROMPT_NAME})" "${DNP_GIT_REMOTE_URL}"
n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"

if [[ ${push_deploy_image} == true ]]; then
  echo "Won't push the deploy image, use falg ${MSG_DIMMED_FORMAT}--push-deploy-image${MSG_END_FORMAT} to push it."
fi

# ....Build stage..................................................................................
{
  build_docker_flag=("--service-names" "project-core,project-deploy")
  build_docker_flag+=("--force-push-project-core")
  dnp::build_dn_project_services "${build_docker_flag[@]}" "${remaining_args[@]}"
  build_exit_code=$?
}

# ....Push deploy stage............................................................................
if [[ ${push_deploy_image} == true ]]; then
  source "${script_path_parent}/../utils/execute_compose.bash" || exit 1
  {
    push_docker_flag=("--push" "project-deploy")
    # push_docker_flag=("--override-build-cmd" "push" "project-deploy")
    dnp::excute_compose_on_dn_project_image "${push_docker_flag[@]}"
    push_exit_code=$?
  }
else
  push_exit_code=0
fi

# ====Teardown=====================================================================================
n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
popd >/dev/null || exit 1

exit $((build_exit_code + push_exit_code))
