#!/bin/bash
DOCUMENTATION_BUILD_DEPLOY=$( cat <<'EOF'
# =================================================================================================
# Build deploy images specified in docker-compose.project.build.native.yaml
#
# Usage:
#   $ bash build.deploy.bash [<any-arguments>] [--] [<any-build.all-argument>]
#
# Arguments:
#   --push-deploy-image
#   -h | --help
#
# Positional argument:
#   <any-docker-flag>                      Any docker flag (optional)
#
# =================================================================================================
EOF
)

# (Priority) ToDo: unit-test for flag option

clear
pushd "$(pwd)" >/dev/null || exit 1

# ToDo: move the help fct near the script/fct menu
function show_help() {
  # (NICE TO HAVE) ToDo: refactor as a n2st fct (ref NMO-583)
  echo -e "${MSG_DIMMED_FORMAT}"
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo -e "$0 --help\n"
  # Strip shell comment char `#` and both lines
  echo -e "${DOCUMENTATION_BUILD_DEPLOY}" | sed 's/\# ====.*//' | sed 's/^\#//'
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo -e "${MSG_END_FORMAT}"
}


# ....Set env variables (pre cli)................................................................
declare -a REMAINING_ARGS
PUSH_DEPLOY_IMAGE=false

# ....cli..........................................................................................
while [ $# -gt 0 ]; do

  case $1 in
    --push-deploy-image)
      PUSH_DEPLOY_IMAGE=true
      shift # Remove argument (--push-deploy-image)
      ;;
    -h | --help)
      clear
      show_help
      exit
      ;;
    --) # no more option
      shift
      REMAINING_ARGS=( "$@" )
      break
      ;;
    *) # Default case
      REMAINING_ARGS=("$@")
      break
      ;;
  esac

done

# ....Set env variables (post cli)...............................................................
# Add env var


# ....Source project shell-scripts dependencies..................................................
SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]:-'.'}")"
SCRIPT_PATH_PARENT="$(dirname "${SCRIPT_PATH}")"
source "${SCRIPT_PATH_PARENT}/../utils/import_dnp_lib.bash" || exit 1
source "${SCRIPT_PATH_PARENT}/../utils/load_super_project_config.bash" || exit 1
source "${SCRIPT_PATH_PARENT}/execute_compose.bash" || exit 1
source "${SCRIPT_PATH_PARENT}/build.all.bash" || exit 1

# ....Execute....................................................................................
clear

# ====Begin========================================================================================
n2st::norlab_splash "${PROJECT_GIT_NAME} (${DNP_PROMPT_NAME})" "${DNP_GIT_REMOTE_URL}"
n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"

if [[ ${PUSH_DEPLOY_IMAGE} == true ]]; then
  echo "Won't push the deploy image, use falg ${MSG_DIMMED_FORMAT}--push-deploy-image${MSG_END_FORMAT} to push it."
fi

{
  BUILD_DOCKER_FLAG=("--service-names" "project-core,project-deploy")
  BUILD_DOCKER_FLAG+=("--force-push-project-core")
  # BUILD_DOCKER_FLAG+=("--dry-run")
  dnp::build_dn_project_services "${BUILD_DOCKER_FLAG[@]}" "${REMAINING_ARGS[@]}"
  BUILD_EXIT_CODE=$?

}

if [[ ${PUSH_DEPLOY_IMAGE} == true ]]; then
  source "${SCRIPT_PATH_PARENT}/execute_compose.bash" || exit 1
  {
    PUSH_DOCKER_FLAG=("--push" "project-deploy")
    # PUSH_DOCKER_FLAG=("--override-build-cmd" "push" "project-deploy")
    # PUSH_DOCKER_FLAG+=("--dry-run")
    dnp::excute_compose_on_dn_project_image "${PUSH_DOCKER_FLAG[@]}"
    PUSH_EXIT_CODE=$?
  }
else
  PUSH_EXIT_CODE=0
fi

# ====Teardown=====================================================================================
n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
popd >/dev/null || exit 1

exit $((BUILD_EXIT_CODE + PUSH_EXIT_CODE))
