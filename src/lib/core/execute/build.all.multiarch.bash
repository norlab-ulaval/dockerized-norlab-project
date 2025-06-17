#!/bin/bash
DOCUMENTATION_BUILD_ALL_MULTIARCH=$(
  cat <<'EOF'
# =================================================================================================
# Build all images specified in docker-compose.project.build.multiarch.yaml
# i.e. pull/push from/to dockerhub sequentialy.
#
# Requirement:
#   - Multiarch build require docker buildx be installed and a multi architecture builder be
#     configured using docker-container buildx driver with 'linux/arm64' and 'linux/amd64'.
#   - The buildx builder name 
#
# Usage:
#   $ bash build.all.multiarch.bash [<any-arguments>] [--] [<any-docker-flag>]
#
# Arguments:
#   --service-names "<name1>,<name2>"    To override the list of build services.
#                                        Must be a comma separated string of service name.
#   --no-force-push-project-core         To build DN images dependencies using local image store
#   -f | --file "compose.yaml"           To override the docker compose file
#                                         (default: "docker-compose.project.build.multiarch.yaml")
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

# (Priority) ToDo: unit-test of flag option

# ....Function.....................................................................................
function dnp::build_services_multiarch() {
  local tmp_cwd
  tmp_cwd=$(pwd)

  # ....Set env variables (pre cli)................................................................
  declare -a dnp_build_all_args=()
  declare -a remaining_args=()
  local force_push_project_core=true
  local the_compose_file="docker-compose.project.build.multiarch.yaml"

  # ....cli........................................................................................
  function show_help() {
    # (NICE TO HAVE) ToDo: refactor as a n2st fct (ref NMO-583)
    echo -e "${MSG_DIMMED_FORMAT}"
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "$0 --help"
    # Strip shell comment char `#` and both lines
    echo -e "${DOCUMENTATION_BUILD_ALL_MULTIARCH}" | sed '/\# ====.*/d' | sed 's/^\# //' | sed 's/^\#//'
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "${MSG_END_FORMAT}"
  }

  while [ $# -gt 0 ]; do

    case $1 in
    --no-force-push-project-core)
      force_push_project_core=false
      shift # Remove argument (--my-new-flag-name)
      ;;
    --service-names)
      # ToDo: refactor (ref task NMO-574)
      # shellcheck disable=SC2207
      dnp_build_all_args+=("--service-names" "${2}")
      shift # Remove argument (--service-names)
      shift # Remove argument value
      ;;
    -f | --file)
      the_compose_file="${2}"
      shift # Remove argument (-f | --file)
      shift # Remove argument value
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
  dnp_build_all_args+=("--file" "${the_compose_file}")
  if [[ ${force_push_project_core} == true ]]; then
    dnp_build_all_args+=("--force-push-project-core")
  fi
  dnp_build_all_args+=("--multiarch")
  dnp_build_all_args+=("${remaining_args[@]}")

  # ====Begin========================================================================================
  dnp::build_services "${dnp_build_all_args[@]}"
  build_exit_code=$?

  # ....Teardown...................................................................................
  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && exit 1; }
  return $build_exit_code
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  # ....Source project shell-scripts dependencies..................................................
  script_path="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  script_path_parent="$(dirname "${script_path}")"
  source "${script_path_parent}/../utils/import_dnp_lib.bash" || exit 1
  source "${script_path_parent}/../utils/load_super_project_config.bash" || exit 1
  source "${script_path_parent}/build.all.bash" || exit 1

  # ....Execute....................................................................................
  if [[ "${DNP_CLEAR_CONSOLE_ACTIVATED}" == "true" ]]; then
    clear
  fi
  n2st::norlab_splash "${DNP_GIT_NAME} (${DNP_PROMPT_NAME})" "${DNP_GIT_REMOTE_URL}"
  n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  dnp::build_services_multiarch "$@"
  fct_exit_code=$?
  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  exit "${fct_exit_code}"
else
  # This script is being sourced, ie: __name__="__source__"

  # ....Pre-condition..............................................................................
  dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
  test -n "$( declare -f dnp::import_lib_and_dependencies )" || { echo -e "${dnp_error_prefix} The DNP lib is not loaded!" ; exit 1 ; }
  test -n "$( declare -f n2st::print_msg )" || { echo -e "${dnp_error_prefix} The N2ST lib is not loaded!" ; exit 1 ; }
  test -n "${SUPER_PROJECT_ROOT}" || { echo -e "${dnp_error_prefix} The super project DNP configuration is not loaded!" ; exit 1 ; }
  test -n "$( declare -f dnp::build_services )" || { echo -e "${dnp_error_prefix} The DNP build native lib is not loaded!" ; exit 1 ; }
fi
