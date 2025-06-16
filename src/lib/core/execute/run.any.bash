#!/bin/bash

DOCUMENTATION_BUFFER_RUN_ANY=$( cat <<'EOF'
# =================================================================================================
# Run a command in a unique DNP container.
# Create a new container with a unique ID, to prevent name collision with those created by dnp up.
#
# Usage:
#   $ bash build.all.bash
#   $ bash run.any.bash [OPTIONS] [-- COMMAND [ARGS...]]
#
# Options:
#   --service SERVICE            The service to attach once up (Default: develop)
#                                Service: develop, deploy, ...
#   -e, --env stringArray        Set container environment variables
#   -w, --workdir string         Override path to workdir directory
#   -T, --no-TTY                 Disable pseudo-TTY allocation
#   -v, --volume stringArray     Bind mount a volume
#   --detach                     Execute COMMAND in the background
#   --dry-run                    (Require --detach flag)
#
# Positional argument:
#   command & arguments    Any command to be executed inside the docker container (default: bash)
#
# =================================================================================================
EOF
)

# (NICE TO HAVE) ToDo: investigate, not working properly
#   --no-rm                      Dont remove container on exits (default to --rm)


function dnp::run_any() {
  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)

  declare -a remaining_args=()
  declare -a docker_compose_run_flag=()
  local the_service=develop
  local no_rm=false

  # ....cli......................................................................................
  while [[ $# -gt 0 ]]; do
      case "$1" in
          --service)
            the_service="${2}"
            shift
            shift
            ;;
#          --no-rm)
#            # (NICE TO HAVE) ToDo: investigate, not working properly
#            no_rm=true
#            shift
#            ;;
          --help|-h)
              dnp::command_help_menu "${DOCUMENTATION_BUFFER_RUN_ANY}"
              exit 0
              ;;
          --detach|--dry-run|-T|--no-TTY) # Assume its a docker compose flag
              docker_compose_run_flag+=("$1")
              if [[ ${1} == "--dry-run" ]]; then
                docker_compose_run_flag+=("--detach")
              fi
              shift
              ;;
          -e|--env|-w|--workdir|-v|--volume) # Assume its a docker compose flag
              docker_compose_run_flag+=("$1" "$2")
              shift
              shift
              ;;
          --) # no more option
              shift
              remaining_args=("$@")
              break
              ;;
          *)
              dnp::illegal_command_msg "run" "$*"
              exit 1
              ;;
      esac
  done

  # ....Set env variables (post cli)...............................................................
  declare -a docker_run_cmd_and_args=("${remaining_args[@]:-"bash"}")
  local compose_path="${DNP_ROOT:?err}/src/lib/core/docker"

  # ....post-cli setup.............................................................................
  dnp::up_and_attach --no-up --no-attach --service "${the_service}" || return 1

  test -n "${_THE_COMPOSE_FILE:?err}" || n2st::print_msg_error_and_exit "Env var _THE_COMPOSE_FILE is empty!"
  test -n "${_THE_SERVICE:?err}" || n2st::print_msg_error_and_exit "Env var _THE_SERVICE is empty!"
  local compose_file="${_THE_COMPOSE_FILE}"
  local the_service="${_THE_SERVICE}"

  # ....Begin......................................................................................
  DN_CONTAINER_NAME="${DN_CONTAINER_NAME:?err}-${BASHPID:-$$}"
  echo
  n2st::print_msg "Be advised that ${MSG_DIMMED_FORMAT}dnp run${MSG_END_FORMAT} extend the container_name attribute
  of the docker compose file, so that you can spin the same service
  multiple time and all spawned containers will have a unique name,
  i.e. ${MSG_DIMMED_FORMAT}${DN_CONTAINER_NAME}${MSG_END_FORMAT}"
  echo

  declare -a docker_run_flag=(--no-deps)
  if [[ ${no_rm} != true ]]; then
    # (NICE TO HAVE) ToDo: investigate, not working properly
    docker_run_flag+=(--rm)
  fi
  docker_run_flag+=(--name "${DN_CONTAINER_NAME}" --env "DN_CONTAINER_NAME=${DN_CONTAINER_NAME}")
  docker_run_flag+=(--build --quiet-build)
  docker_run_flag+=("${docker_compose_run_flag[@]}")
  docker_run_flag+=("${the_service}")
  docker_run_flag+=("/dockerized-norlab/project/${the_service}/dn_entrypoint.init.bash")
  docker_run_flag+=("${docker_run_cmd_and_args[@]}")
#  dnp::excute_compose "--override-build-cmd" "run" "-f" "${compose_file}" "${docker_run_flag[@]}"
  n2st::print_msg "Execute ${MSG_DIMMED_FORMAT}docker compose -f ${compose_path}/${the_compose_file} run ${docker_run_flag[*]}${MSG_END_FORMAT}"
  docker compose "-f" "${compose_path}/${compose_file}" run "${docker_run_flag[@]}"
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
    source "${script_path_parent}/up_and_attach.bash" || exit 1
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
  dnp::run_any "$@"
  fct_exit_code=$?
  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  exit "${fct_exit_code}"
else
  # This script is being sourced, ie: __name__="__source__"

  # ....Pre-condition..............................................................................
  dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
  test -n "$( declare -f dnp::import_lib_and_dependencies )" || { echo -e "${dnp_error_prefix} The DNP lib is not loaded!" ; exit 1 ; }
  test -n "$( declare -f dnp::up_and_attach )" || { echo -e "${dnp_error_prefix} The dnp::up_and_attach is not loaded!" ; exit 1 ; }
  test -n "$( declare -f n2st::print_msg )" || { echo -e "${dnp_error_prefix} The N2ST lib is not loaded!" ; exit 1 ; }
  test -n "${SUPER_PROJECT_ROOT}" || { echo -e "${dnp_error_prefix} The super project DNP configuration is not loaded!" ; exit 1 ; }
fi
