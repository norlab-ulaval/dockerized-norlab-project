#!/bin/bash
DOCUMENTATION_DNP_EXECUTE_COMPOSE=$(
  cat <<'EOF'
# =================================================================================================
# Convenient script for executin a compose command on a single images specified in a compose file.
#
# Note: Prefer using $(build.all.bash) instead of using $(execute_compose.bash) directly
#
# Usage as a function:
#   $ source execute_compose.bash
#   $ dnp::excute_compose_on_dn_project_image [<any-arguments>] [--] [<any-docker-flag>]
#
# Usage as a script:
#   $ bash execute_compose.bash [<any-arguments>] [--] [<any-docker-flag>]
#
# Arguments:
#   --override-build-cmd <docker_cmd>      To override the docker command
#                                           (defaul: 'build')
#   -f | --file "compose.yaml"             To override the docker compose file
#                                           (default: "docker-compose.project.build.native.yaml")
#   --multiarch
#   --buildx-builder-name "name"         Default to "local-builder-multiarch-virtual"
#   -h | --help
#
# Positional argument:
#   <any-docker-flag>                      (Optional) Any docker flag
#
# Global
#   none
#
# =================================================================================================
EOF
)

# (CRITICAL) ToDo: unit-test

# ....Functions....................................................................................
function dnp::excute_compose_on_dn_project_image() {
  local TMP_CWD
  TMP_CWD=$(pwd)

  cd "${SUPER_PROJECT_ROOT:?err}" || exit 1

  # ....Set env variables (pre cli)................................................................
  local REMAINING_ARGS=()
  local DOCKER_COMMAND_W_FLAGS=()
  local COMPOSE_PATH="${SUPER_PROJECT_ROOT:?err}/.dockerized_norlab_project/configuration"
  local THE_COMPOSE_FILE="docker-compose.project.build.native.yaml"
  local MULTIARCH=false
  local LOCAL_BUILDX_BUILDER_NAME="local-builder-multiarch-virtual"

  # Note: The env var DOCKER_CMD is required for using the script with 'push' or 'config' docker cmd
  local DOCKER_CMD=build

  # ....cli........................................................................................
  function show_help() {
    # (NICE TO HAVE) ToDo: refactor as a n2st fct (ref NMO-583)
    echo -e "${MSG_DIMMED_FORMAT}"
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "$0 --help\n"
    # Strip shell comment char `#` and both lines
    echo -e "${DOCUMENTATION_DNP_EXECUTE_COMPOSE}" | sed 's/\# ====.*//' | sed 's/^\#//'
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "${MSG_END_FORMAT}"
  }

  while [ $# -gt 0 ]; do

    case $1 in
    --override-build-cmd)
      DOCKER_CMD="${2}"
      shift # Remove argument (--override-build-cmd)
      shift # Remove argument value
      ;;
    -f | --file)
      THE_COMPOSE_FILE="${2}"
      shift # Remove argument (-f | --file)
      shift # Remove argument value
      ;;
    --multiarch)
      MULTIARCH=true
      shift # Remove argument (--multiarch)
      ;;
    --buildx-builder-name)
      LOCAL_BUILDX_BUILDER_NAME="${2}"
      shift # Remove argument (--buildx-builder-name)
      shift # Remove argument value
      ;;
    -h | --help)
      clear
      show_help
      exit
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
  DOCKER_COMMAND_W_FLAGS=("${DOCKER_CMD}" "${REMAINING_ARGS[@]}")

  # ====Begin======================================================================================
  n2st::print_formated_script_header "execute_compose.bash ${MSG_END_FORMAT}on device ${MSG_DIMMED_FORMAT}$(hostname -s)" "${MSG_LINE_CHAR_BUILDER_LVL2}"

  n2st::set_is_teamcity_run_environment_variable
  n2st::print_msg "IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN} ${TC_VERSION}"
  n2st::set_which_architecture_and_os
  n2st::print_msg "Current image architecture and os: $IMAGE_ARCH_AND_OS"
  n2st::print_msg "MULTIARCH: ${MULTIARCH}"

  if [[ -z ${BUILDX_BUILDER} ]]; then
    if [[ ${IS_TEAMCITY_RUN} == false ]] && [[ ${MULTIARCH} == false ]]; then
      # Note: Default to default buildx builder (ie native host architecture) so that the build img
      # be available in the local image store and that run executed via `up_and_attach.bash` doesn't
      # require pulling built img from dockerhub.
      if [[ $IMAGE_ARCH_AND_OS == 'darwin/arm64' ]]; then
        export BUILDX_BUILDER=desktop-linux
      elif [[ $IMAGE_ARCH_AND_OS == 'l4t/arm64' ]] || [[ $IMAGE_ARCH_AND_OS == 'linux/arm64' ]] || [[ $IMAGE_ARCH_AND_OS == 'linux/x86' ]]; then
        export BUILDX_BUILDER=default
      fi
    elif [[ ${IS_TEAMCITY_RUN} == false ]] && [[ ${MULTIARCH} == true ]]; then
      CURRENT_BUILDX_BUILDER=$(docker buildx inspect | grep -i -m 1 -e Name: | sed "s/^Name:[[:space:]]*//")
      n2st::print_msg "Current buildx builder: ${CURRENT_BUILDX_BUILDER}"
      BUILDER_PLATFORM=$(docker buildx inspect --builder "${CURRENT_BUILDX_BUILDER}" | grep -i -e Platforms)
      if [[ ! "${BUILDER_PLATFORM}" =~ "Platforms:".*"linux/amd64*".* ]] && [[ ! "${BUILDER_PLATFORM}" =~ "Platforms:".*"linux/arm64*".* ]]; then
        n2st::print_msg_warning "Exporting ${MSG_DIMMED_FORMAT}BUILDX_BUILDER=${LOCAL_BUILDX_BUILDER_NAME:?err}${MSG_END_FORMAT} for $(basename $0) execution."
        # Set builder for local execution
        export BUILDX_BUILDER="${LOCAL_BUILDX_BUILDER_NAME}"
        # ToDo: logic to warn and provide instructions to user if the builder does not exist and need to be setup.
      fi
    elif [[ ${IS_TEAMCITY_RUN} == true ]] && [[ ${MULTIARCH} == false ]]; then
      # Case TC native-architecture: Run build on native single arch builder. Required for build.all.bash
      export BUILDX_BUILDER=default
    elif [[ ${IS_TEAMCITY_RUN} == true ]] && [[ ${MULTIARCH} == true ]]; then
      # Case TC multi-architecture: Run build on muti-arch builder. Required for build.all.multiarch.bash
      # Pass as TC is responsible for setting the buildx builder
      :
    fi
    # Force builder initialisation
    docker buildx inspect --bootstrap "${BUILDX_BUILDER}" >/dev/null
  fi
  n2st::print_msg "BUILDX_BUILDER=${BUILDX_BUILDER}"

  # ....Execute....................................................................................
  n2st::teamcity_service_msg_blockOpened "Execute docker compose with argument ${DOCKER_COMMAND_W_FLAGS[*]}"
  DOCKER_CMD_STR="${MSG_DIMMED_FORMAT}docker compose --file ${COMPOSE_PATH}/${THE_COMPOSE_FILE} ${DOCKER_COMMAND_W_FLAGS[*]}${MSG_END_FORMAT}"
  n2st::print_msg "Execute ${DOCKER_CMD_STR}\n"

  # Refactor using "n2st::show_and_execute_docker" (See ref NMO-575)
  docker compose --file "${COMPOSE_PATH}/${THE_COMPOSE_FILE}" "${DOCKER_COMMAND_W_FLAGS[@]}"
  local DOCKER_EXIT_CODE=$?
  # Ref on docker exit codes: https://komodor.com/learn/exit-codes-in-containers-and-kubernetes-the-complete-guide/

  n2st::teamcity_service_msg_blockClosed

  if [[ ${DOCKER_EXIT_CODE} == 0 ]]; then
    n2st::print_msg_done "Completed ${DOCKER_CMD_STR} succesfuly 👍"
  else
    n2st::print_msg_error "Completed ${DOCKER_CMD_STR} with error ⚠️ "
  fi

  # ....Teardown...................................................................................
  n2st::print_formated_script_footer "execute_compose.bash" "${MSG_LINE_CHAR_BUILDER_LVL2}"
  cd "${TMP_CWD}" || { echo "Return to original dir error" 1>&2 && exit 1; }
  return ${DOCKER_EXIT_CODE}
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  # ....Source project shell-scripts dependencies..................................................
  SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  SCRIPT_PATH_PARENT="$(dirname "${SCRIPT_PATH}")"
  source "${SCRIPT_PATH_PARENT}/../utils/import_dnp_lib.bash" || exit 1
  source "${SCRIPT_PATH_PARENT}/../utils/load_super_project_config.bash" || exit 1

  # ....Execute....................................................................................
  n2st::norlab_splash "${PROJECT_GIT_NAME} (${DNP_PROMPT_NAME})" "${DNP_GIT_REMOTE_URL}"
  n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  dnp::excute_compose_on_dn_project_image "$@"
  FCT_EXIT_CODE=$?
  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  exit "${FCT_EXIT_CODE}"
else
  # This script is being sourced, ie: __name__="__source__"
  :
fi
