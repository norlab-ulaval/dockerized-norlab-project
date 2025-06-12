#!/bin/bash
# =================================================================================================
# Execute docker compose down using the compose file matching the host
#
# Usage:
#   $ bash down.bash [<any-docker-compose-down-flags>]
#
# =================================================================================================

function dnp::down_command() {
  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)
  declare -a remaining_args=("$@")

  cd "${SUPER_PROJECT_ROOT:?err}" || return 1

  # ....Begin......................................................................................
  n2st::print_msg "Stoping container on device ${MSG_DIMMED_FORMAT}$(hostname -s)${MSG_END_FORMAT}"
  # n2st::print_formated_script_header "$(basename $0) ${MSG_END_FORMAT}on device ${MSG_DIMMED_FORMAT}$(hostname -s)" "${MSG_LINE_CHAR_BUILDER_LVL2}"

  n2st::set_which_architecture_and_os
  n2st::print_msg "Current image architecture and os: ${IMAGE_ARCH_AND_OS:?err}"

  n2st::set_is_teamcity_run_environment_variable

  # ....Device specific config.......................................................................
  compose_path=".dockerized_norlab_project/configuration"

  if [[ ${IMAGE_ARCH_AND_OS:?err} == 'l4t/arm64' ]]; then
    the_compose_file=docker-compose.project.run.jetson.yaml
  elif [[ $IMAGE_ARCH_AND_OS == 'darwin/arm64' ]]; then
    the_compose_file=docker-compose.project.run.darwin.yaml
  elif [[ $IMAGE_ARCH_AND_OS == 'linux/x86' ]]; then
    the_compose_file=docker-compose.project.run.linux-x86.yaml
  else
    n2st::print_msg_error_and_exit "Support for current host not implemented yet!"
  fi

  docker compose -f "${compose_path}/${the_compose_file}" down "${remaining_args[@]}"
  exit_code=$?

  # ....Teardown...................................................................................
  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return $exit_code
}


# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  # ....Source project shell-scripts dependencies..................................................
  script_path="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  script_path_parent="$(dirname "${script_path}")"
  if [[ -z $( declare -f dnp::import_lib_and_dependencies ) ]]; then
    source "${script_path_parent}/../utils/import_dnp_lib.bash" || exit 1
    source "${script_path_parent}/../utils/execute_compose.bash" || exit 1
  fi
  if [[ -z ${SUPER_PROJECT_ROOT} ]]; then
    source "${script_path_parent}/../utils/load_super_project_config.bash" || exit 1
  fi

  # ....Execute....................................................................................
  if [[ "${DNP_CLEAR_CONSOLE_ACTIVATED}" == "true" ]]; then
    clear
  fi
  n2st::norlab_splash "${DNP_GIT_NAME} (${DNP_PROMPT_NAME})" "${DNP_GIT_REMOTE_URL}"
  n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  dnp::down_command "$@"
  fct_exit_code=$?
  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  exit "${fct_exit_code}"
else
  # This script is being sourced, ie: __name__="__source__"
  :
fi
