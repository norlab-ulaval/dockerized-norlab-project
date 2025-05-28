#!/bin/bash
DOCUMENTATION_BUILD_ALL=$(
  cat <<'EOF'
# =================================================================================================
# Build all images specified in a compose file.
#
# Usage as a function:
#   $ source build.all.bash
#   $ dnp::build_dn_project_services [<any-arguments>] [--] [<any-docker-flag>]
#
# Usage as a script:
#   $ bash build.all.bash [<any-arguments>] [--] [<any-docker-flag>]
#
# Arguments:
#   --force-push-project-core            To build pull/push from/to dockerhub sequentialy.
#   --service-names "<name1>,<name2>"    To override the list of build services.
#                                        Must be a comma separated string of service name.
#   -f | --file "compose.yaml"           To override the docker compose file
#                                        (default: "docker-compose.project.build.native.yaml")
#   --msg-line-level                     Set consol horizontal line character when used as a fct
#   -h | --help
#
# Positional argument:
#   <any-docker-flag>                      Any docker flag (optional)
#
# Global
#   none
#
# =================================================================================================
EOF
)

# (Priority) ToDo: unit-test for flag option

# ....Function.....................................................................................
function dnp::build_dn_project_services() {
  local TMP_CWD
  TMP_CWD=$(pwd)

  # ....Set env variables (pre cli))...............................................................
  local REMAINING_ARGS=()
  local BUILD_EXIT_CODE=()
  local SERVICES_NAMES=("none")
  local FORCE_PUSH_PROJECT_CORE=false
  local COMPOSE_PATH="${SUPER_PROJECT_ROOT:?err}/.dockerized_norlab_project/configuration"
  local THE_COMPOSE_FILE="docker-compose.project.build.native.yaml"
  local BUILD_DOCKER_FLAG=()
  local MSG_LINE_LEVEL=$MSG_LINE_CHAR_BUILDER_LVL1

  # ....cli........................................................................................
  function show_help() {
    # (NICE TO HAVE) ToDo: refactor as a n2st fct (ref NMO-583)
    echo -e "${MSG_DIMMED_FORMAT}"
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "$0 --help\n"
    # Strip shell comment char `#` and both lines
    echo -e "${DOCUMENTATION_BUILD_ALL}" | sed 's/\# ====.*//' | sed 's/^\#//'
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "${MSG_END_FORMAT}"
  }

  while [ $# -gt 0 ]; do

    case $1 in
    --service-names)
      # ToDo: refactor (ref task NMO-574)
      # shellcheck disable=SC2207
      # Override SERVICES_NAMES
      SERVICES_NAMES=($(echo "${2}" | tr "," "\n"))
      shift # Remove argument (--service-names)
      shift # Remove argument value
      ;;
    --force-push-project-core)
      FORCE_PUSH_PROJECT_CORE=true
      shift # Remove argument (--force-push-project-core)
      ;;
    -f | --file)
      THE_COMPOSE_FILE="${2}"
      shift # Remove argument (-f | --file)
      shift # Remove argument value
      ;;
    --msg-line-level)
      MSG_LINE_LEVEL="${2}"
      shift # Remove argument (--msg-line-level)
      shift # Remove argument value
      ;;
    -h | --help)
      clear
      show_help
      exit
      ;;
    --multiarch)
      BUILD_DOCKER_FLAG+=("--multiarch")
      shift # Remove argument (--multiarch)
      ;;
    --) # no more option
      shift
      REMAINING_ARGS=("$@")
      break
      ;;
    *) # Default case
      REMAINING_ARGS=("$@")
      break
      ;;
    esac

  done

  # ....Set env variables (post cli)...............................................................
  BUILD_DOCKER_FLAG+=("${REMAINING_ARGS[@]}")

  # ====Begin======================================================================================

  # ....Fetch service list.........................................................................
  if [[ "${SERVICES_NAMES[0]}" == "none" ]]; then
    # shellcheck disable=SC2207
    SERVICES_NAMES=($(docker compose -f "${COMPOSE_PATH}/${THE_COMPOSE_FILE}" config --services --no-interpolate))
  fi

  n2st::print_msg "Building the following services"
  for idx in "${!SERVICES_NAMES[@]}"; do
    echo "              [$idx] ${SERVICES_NAMES[idx]}"
  done

  # ....Execute build..............................................................................
  n2st::print_msg "FORCE_PUSH_PROJECT_CORE: ${FORCE_PUSH_PROJECT_CORE}"
  if [[ ${FORCE_PUSH_PROJECT_CORE} == false ]]; then
    n2st::print_msg "Building from the local image store. ${MSG_DIMMED_FORMAT}To pull/push from/to Dockerhub sequentialy, execute script with flag '--force-push-project-core'.${MSG_END_FORMAT}"

    dnp::excute_compose_on_dn_project_image --file "${THE_COMPOSE_FILE}" "${BUILD_DOCKER_FLAG[@]}" --with-dependencies "${SERVICES_NAMES[@]}"
    BUILD_EXIT_CODE=("$?")

    # ....On faillure, re-run build.all one service at the time....................................
    if [[ ${BUILD_EXIT_CODE[0]} != 0 ]]; then
      n2st::print_msg_error "Build error, re-running build.all one service at the time"
      # Reset exit code buffer
      BUILD_EXIT_CODE=()
      # Execute on all remaining service
      for each in "${SERVICES_NAMES[@]}"; do
        dnp::excute_compose_on_dn_project_image --file "${THE_COMPOSE_FILE}" "${BUILD_DOCKER_FLAG[@]}" "${each}"
        BUILD_EXIT_CODE+=("$?")
      done

      # Show build faillure summary
      n2st::draw_horizontal_line_across_the_terminal_window "${MSG_LINE_LEVEL}"
      n2st::print_msg "Build faillure test summary"
      for idx in "${!BUILD_EXIT_CODE[@]}"; do
        if [[ ${BUILD_EXIT_CODE[idx]} != 0 ]]; then
          echo -e "    ${MSG_ERROR_FORMAT}Service ${SERVICES_NAMES[idx]} completed build with error${MSG_END_FORMAT}"
        else
          echo -e "    ${MSG_DONE_FORMAT}Service ${SERVICES_NAMES[idx]} completed build${MSG_END_FORMAT}"
        fi
      done
    else
      # Show build succes
      n2st::draw_horizontal_line_across_the_terminal_window "${MSG_LINE_LEVEL}"
      n2st::print_msg_done "Building services ${MSG_DIMMED_FORMAT}${SERVICES_NAMES[*]}${MSG_END_FORMAT} completed succesfully"
    fi

  else

    # Rebuild and push the core image prior to building any other images
    for each in "${SERVICES_NAMES[@]}"; do
      if [[ "${each}" == "project-core" ]]; then
        # Note:
        #   - THIS WORK ON MacOs with buildx builder "docker-container:local-builder-multiarch-virtual"
        #   - THIS WORK on TC server as its the same setup use in DN
        #   - ⚠️ If you experience problem:
        #       1. check that project-core image on Dockerhub has been pushed for both arm64 and amd64
        #       2. if not, consider building and pushing manualy each arm64 and amd64 images and
        #          then merge as in DN l4t base images
        dnp::excute_compose_on_dn_project_image --file "${THE_COMPOSE_FILE}" "${BUILD_DOCKER_FLAG[@]}" --push project-core
        BUILD_EXIT_CODE=("$?")
      fi
    done

    # Execute docker cmd on all remaining service
    for each in "${SERVICES_NAMES[@]}"; do
      if [[ "${each}" != "project-core" ]]; then
        dnp::excute_compose_on_dn_project_image --file "${THE_COMPOSE_FILE}" "${BUILD_DOCKER_FLAG[@]}" "${each}"
        BUILD_EXIT_CODE+=("$?")
      fi
    done

    # ....Show build summary.......................................................................
    n2st::draw_horizontal_line_across_the_terminal_window "${MSG_LINE_LEVEL}"
    n2st::print_msg "Build summary"
    for idx in "${!BUILD_EXIT_CODE[@]}"; do
      if [[ ${BUILD_EXIT_CODE[idx]} != 0 ]]; then
        echo -e "    ${MSG_ERROR_FORMAT}Service ${SERVICES_NAMES[idx]} completed build with error${MSG_END_FORMAT}"
      else
        echo -e "    ${MSG_DONE_FORMAT}Service ${SERVICES_NAMES[idx]} completed build${MSG_END_FORMAT}"
      fi
    done

  fi

  # Check build faillure
  BUILD_EXIT=0
  for each_build_exit_code in "${BUILD_EXIT_CODE[@]}"; do
    BUILD_EXIT=$((BUILD_EXIT + each_build_exit_code))
  done

  # ....Teardown...................................................................................
  cd "${TMP_CWD}" || { echo "Return to original dir error" 1>&2 && exit 1; }

  if [[ ${BUILD_EXIT} != 0 ]]; then
    return 1
  else
    return 0
  fi
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  # ....Source project shell-scripts dependencies..................................................
  SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  SCRIPT_PATH_PARENT="$(dirname "${SCRIPT_PATH}")"
  source "${SCRIPT_PATH_PARENT}/../utils/import_dnp_lib.bash" || exit 1
  source "${SCRIPT_PATH_PARENT}/../utils/load_super_project_config.bash" || exit 1
  source "${SCRIPT_PATH_PARENT}/../utils/execute_compose.bash" || exit 1

  # ....Execute....................................................................................
  clear
  n2st::norlab_splash "${PROJECT_GIT_NAME} (${DNP_PROMPT_NAME})" "${DNP_GIT_REMOTE_URL}"
  n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  dnp::build_dn_project_services "$@"
  FCT_EXIT_CODE=$?
  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  exit "${FCT_EXIT_CODE}"
else
  # This script is being sourced, ie: __name__="__source__"
  :
fi
