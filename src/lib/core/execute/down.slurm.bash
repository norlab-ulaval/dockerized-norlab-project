#!/bin/bash
# =================================================================================================
# Kill docker compose ressources alocated by run.slurm.bash
#
# Usage:
#   $ bash down.slurm.bash [<any-docker-compose-down-flags>]
#
# =================================================================================================

function dna::down_slurm() {
  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)
  declare -a remaining_args=("$@")

  cd "${SUPER_PROJECT_ROOT:?err}" || return 1

  # ....Begin......................................................................................
  n2st::print_msg "Stoping container on device ${MSG_DIMMED_FORMAT}$(hostname -s)${MSG_END_FORMAT}"
  # n2st::print_formated_script_header "$(basename $0) ${MSG_END_FORMAT}on device ${MSG_DIMMED_FORMAT}$(hostname -s)" "${MSG_LINE_CHAR_BUILDER_LVL2}"

  n2st::set_which_architecture_and_os
  n2st::print_msg "Current os/architecture: ${IMAGE_ARCH_AND_OS:?err}"

  n2st::set_is_teamcity_run_environment_variable

  # ....Device specific config.......................................................................
  compose_path="${DNA_ROOT:?err}/src/lib/core/docker"
  the_compose_file=docker-compose.project.run.slurm.yaml

  container_id=$(docker compose -f "${compose_path}/${the_compose_file}" ps --quiet --all --orphans=false)
  if [[ -n ${container_id} ]]; then
    echo
    echo "Compose project project-slurm running service(s) container_id:"
    echo "${container_id[*]}"
    echo
    echo "Stop the following running container(s):"
    # shellcheck disable=SC2068
    docker stop ${container_id[@]} "${remaining_args[@]}" 2>/dev/null
    exit_code=$?
  else
    n2st::print_msg "No running container"
    exit_code=0
  fi

  # ....Teardown...................................................................................
  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return $exit_code
}


# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  # ....Source project shell-scripts dependencies..................................................
  script_path="$(realpath -q "${BASH_SOURCE[0]:-.}")"
  script_path_parent="$(dirname "${script_path}")"
  if [[ -z $( declare -f dna::import_lib_and_dependencies ) ]]; then
    source "${script_path_parent}/../utils/import_dna_lib.bash" || exit 1
    source "${script_path_parent}/../utils/execute_compose.bash" || exit 1
  fi
  if [[ -z ${SUPER_PROJECT_ROOT} ]]; then
    source "${script_path_parent}/../utils/load_super_project_config.bash" || exit 1
  fi

  # ....Execute....................................................................................
  if [[ "${DNA_CLEAR_CONSOLE_ACTIVATED}" == "true" ]]; then
    clear
  fi
  n2st::norlab_splash "${DNA_SPLASH_NAME_FULL:?err}" "${DNA_GIT_REMOTE_URL}" "negative"
  n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  dna::down_slurm "$@"
  fct_exit_code=$?
  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  exit "${fct_exit_code}"
else
  # This script is being sourced, ie: __name__="__source__"
  :
fi
