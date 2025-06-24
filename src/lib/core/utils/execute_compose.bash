#!/bin/bash
DOCUMENTATION_DNA_EXECUTE_COMPOSE=$(
  cat <<'EOF'
# =================================================================================================
# Convenient script for executin a compose command on a single images specified in a compose file.
#
# Note: Prefer using $(build.all.bash) instead of using $(execute_compose.bash) directly
#
# Usage as a function:
#   $ source execute_compose.bash
#   $ dna::excute_compose [OPTIONS] [--] [<any-docker-flag>]
#
# Usage as a script:
#   $ bash execute_compose.bash [OPTIONS] [--] [<any-docker-flag>]
#
# Options:
#   --override-build-cmd <docker_cmd>      To override the docker command
#                                           (defaul: 'build')
#   -f | --file "compose.yaml"             To override the docker compose file
#                                           (default: "docker-compose.project.build.native.yaml")
#   --compose-path "/path/to/compose/dir"  To override the compose file directory
#   --multiarch
#   --buildx-builder-name "name"           Default to "local-builder-multiarch-virtual"
#   --msg-line-level CHAR                  Set consol horizontal line character when used as a fct
#   -h | --help
#
# Positional argument:
#   <any-docker-flag>                      (Optional) Any docker flag
#
# Global
#   read SUPER_PROJECT_ROOT
#   read DNA_ROOT
#
# =================================================================================================
EOF
)

# (CRITICAL) ToDo: unit-test

# ....Functions....................................................................................
function dna::excute_compose() {
  local tmp_cwd
  tmp_cwd=$(pwd)

  cd "${SUPER_PROJECT_ROOT:?err}" || exit 1

  # ....Set env variables (pre cli)................................................................
  local remaining_args=()
  local docker_command_w_flags=()
  local compose_path="${DNA_ROOT:?err}/src/lib/core/docker"
  local the_compose_file="docker-compose.project.build.native.yaml"
  local multiarch=false
  local local_buildx_builder_name="local-builder-multiarch-virtual"
  local msg_line_level="${MSG_LINE_CHAR_BUILDER_LVL2}"
  local line_style="${MSG_LINE_STYLE_LVL2}"

  # Note: The env var docker_cmd is required for using the script with 'push' or 'config' docker cmd
  local docker_cmd=build

  # ....cli........................................................................................
  function show_help() {
    # (NICE TO HAVE) ToDo: refactor as a n2st fct (ref NMO-583)
    echo -e "${MSG_DIMMED_FORMAT}"
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "$0 --help"
    # Strip shell comment char `#` and both lines
    echo -e "${DOCUMENTATION_DNA_EXECUTE_COMPOSE}" | sed '/\# ====.*/d' | sed 's/^\# //' | sed 's/^\#//'
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "${MSG_END_FORMAT}"
  }

  while [ $# -gt 0 ]; do

    case $1 in
    --override-build-cmd)
      docker_cmd="${2}"
      shift # Remove argument (--override-build-cmd)
      shift # Remove argument value
      ;;
    -f | --file)
      the_compose_file="${2}"
      shift # Remove argument (-f | --file)
      shift # Remove argument value
      ;;
    --compose-path)
      compose_path="${2}"
      shift # Remove argument (--compose-path)
      shift # Remove argument value
      ;;
    --multiarch)
      multiarch=true
      shift # Remove argument (--multiarch)
      ;;
    --buildx-builder-name)
      local_buildx_builder_name="${2}"
      shift # Remove argument (--buildx-builder-name)
      shift # Remove argument value
      ;;
    --msg-line-level)
      msg_line_level="${2}"
      shift # Remove argument (--msg-line-level)
      shift # Remove argument value CHAR
      ;;
    -h | --help)
      clear
      show_help
      exit
      ;;
    --) # no more option
      shift
      remaining_args=("$@")
      break
      ;;
    *) # Default case
      remaining_args=("$@")
      break
      ;;
    esac

  done

  # ....Set env variables (post cli)...............................................................
  docker_command_w_flags=("${docker_cmd}" "${remaining_args[@]}")

  # ====Begin======================================================================================
  n2st::print_formated_script_header "dna::excute_compose ${MSG_END_FORMAT}on device ${MSG_DIMMED_FORMAT}$(hostname -s)" "${msg_line_level}" "${line_style}"

  n2st::set_is_teamcity_run_environment_variable
  n2st::print_msg "IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN} ${TC_VERSION}"
  n2st::set_which_architecture_and_os
  n2st::print_msg "Current image architecture and os: ${IMAGE_ARCH_AND_OS:?err}"
  n2st::print_msg "multiarch: ${multiarch}"

  if [[ -z ${BUILDX_BUILDER} ]]; then
    if [[ ${IS_TEAMCITY_RUN} == false ]] && [[ ${multiarch} == false ]]; then
      # Note: Default to default buildx builder (ie native host architecture) so that the build img
      # be available in the local image store and that run executed via `up_and_attach.bash` doesn't
      # require pulling built img from dockerhub.
      if [[ $IMAGE_ARCH_AND_OS == 'darwin/arm64' ]]; then
        export BUILDX_BUILDER=desktop-linux
      elif [[ $IMAGE_ARCH_AND_OS == 'l4t/arm64' ]] || [[ $IMAGE_ARCH_AND_OS == 'linux/arm64' ]] || [[ $IMAGE_ARCH_AND_OS == 'linux/x86' ]]; then
        export BUILDX_BUILDER=default
      fi
    elif [[ ${IS_TEAMCITY_RUN} == false ]] && [[ ${multiarch} == true ]]; then
      CURRENT_BUILDX_BUILDER=$(docker buildx inspect | grep -i -m 1 -e Name: | sed "s/^Name:[[:space:]]*//")
      n2st::print_msg "Current buildx builder: ${CURRENT_BUILDX_BUILDER}"
      BUILDER_PLATFORM=$(docker buildx inspect --builder "${CURRENT_BUILDX_BUILDER}" | grep -i -e Platforms)
      if [[ ! "${BUILDER_PLATFORM}" =~ "Platforms:".*"linux/amd64*".* ]] && [[ ! "${BUILDER_PLATFORM}" =~ "Platforms:".*"linux/arm64*".* ]]; then
        n2st::print_msg_warning "Setting env var ${MSG_DIMMED_FORMAT}BUILDX_BUILDER=${local_buildx_builder_name:?err}${MSG_END_FORMAT} for $(basename $0) execution."
        # Set builder for local execution
        export BUILDX_BUILDER="${local_buildx_builder_name}"
        # ToDo: logic to warn and provide instructions to user if the builder does not exist and need to be setup.
      fi
    elif [[ ${IS_TEAMCITY_RUN} == true ]] && [[ ${multiarch} == false ]]; then
      # Case TC native-architecture: Run build on native single arch builder. Required for build.all.bash
      export BUILDX_BUILDER=default
    elif [[ ${IS_TEAMCITY_RUN} == true ]] && [[ ${multiarch} == true ]]; then
      # Case TC multi-architecture: Run build on muti-arch builder. Required for build.all.multiarch.bash
      # Pass as TC is responsible for setting the buildx builder
      :
    fi
    # Force builder initialisation
    docker buildx inspect --bootstrap "${BUILDX_BUILDER}" >/dev/null
  fi
  n2st::print_msg "BUILDX_BUILDER=${BUILDX_BUILDER}"

  # ....Execute....................................................................................
  n2st::teamcity_service_msg_blockOpened "Execute docker compose with argument ${docker_command_w_flags[*]}"
  docker_cmd_str="${MSG_DIMMED_FORMAT}docker compose --file ${compose_path}/${the_compose_file} ${docker_command_w_flags[*]}${MSG_END_FORMAT}"
  n2st::print_msg "Execute ${docker_cmd_str}\n"

  # Refactor using "n2st::show_and_execute_docker" (See ref NMO-575)
  docker compose --file "${compose_path}/${the_compose_file}" "${docker_command_w_flags[@]}"
  local docker_exit_code=$?
  # Ref on docker exit codes: https://komodor.com/learn/exit-codes-in-containers-and-kubernetes-the-complete-guide/

  n2st::teamcity_service_msg_blockClosed
  echo
  if [[ ${docker_exit_code} == 0 ]]; then
    n2st::print_msg_done "Completed ${docker_cmd_str} succesfuly 👍"
  else
    n2st::print_msg_error "Completed ${docker_cmd_str} with error ⚠️ "
  fi

  # ....Teardown...................................................................................
  n2st::print_formated_script_footer "dna::excute_compose" "${msg_line_level}" "${line_style}"
  cd "${tmp_cwd}" || { n2st::print_msg_error "Return to original dir error" && exit 1; }
  return ${docker_exit_code}
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  # ....Source project shell-scripts dependencies..................................................
  script_path="$(realpath -q "${BASH_SOURCE[0]:-.}")"
  script_path_parent="$(dirname "${script_path}")"
  source "${script_path_parent}/import_dna_lib.bash" || exit 1
  source "${script_path_parent}/load_super_project_config.bash" || exit 1

  # ....Execute....................................................................................
  n2st::norlab_splash "${DNA_SPLASH_NAME_FULL:?err}" "${DNA_GIT_REMOTE_URL:?err}" "negative"
  n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  dna::excute_compose "$@"
  fct_exit_code=$?
  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  exit "${fct_exit_code}"
else
  # This script is being sourced, ie: __name__="__source__"
  :
fi
