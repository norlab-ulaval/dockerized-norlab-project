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

# (Priority) ToDo: unit-test for flag option

# ....Function.....................................................................................
function dnp::build_dn_project_multiarch_services() {
  local TMP_CWD
  TMP_CWD=$(pwd)

  # ....Set env variables (pre cli)................................................................
  declare -a DNP_BUILD_ALL_ARGS
  declare -a REMAINING_ARGS
  local FORCE_PUSH_PROJECT_CORE=true
  local THE_COMPOSE_FILE="docker-compose.project.build.multiarch.yaml"

  # ....cli........................................................................................
  function show_help() {
    # (NICE TO HAVE) ToDo: refactor as a n2st fct (ref NMO-583)
    echo -e "${MSG_DIMMED_FORMAT}"
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "$0 --help\n"
    # Strip shell comment char `#` and both lines
    echo -e "${DOCUMENTATION_BUILD_ALL_MULTIARCH}" | sed 's/\# ====.*//' | sed 's/^\#//'
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "${MSG_END_FORMAT}"
  }

  while [ $# -gt 0 ]; do

    case $1 in
    --no-force-push-project-core)
      FORCE_PUSH_PROJECT_CORE=false
      shift # Remove argument (--my-new-flag-name)
      ;;
    --service-names)
      # ToDo: refactor (ref task NMO-574)
      # shellcheck disable=SC2207
      DNP_BUILD_ALL_ARGS+=("--service-names" "${2}")
      shift # Remove argument (--service-names)
      shift # Remove argument value
      ;;
    -f | --file)
      THE_COMPOSE_FILE="${2}"
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
  DNP_BUILD_ALL_ARGS+=("--file" "${THE_COMPOSE_FILE[@]}")
  if [[ ${FORCE_PUSH_PROJECT_CORE} == true ]]; then
    DNP_BUILD_ALL_ARGS+=("--force-push-project-core")
  fi
  DNP_BUILD_ALL_ARGS+=("--multiarch")
  DNP_BUILD_ALL_ARGS+=("${REMAINING_ARGS[@]}")

  # ====Begin========================================================================================
  dnp::build_dn_project_services "${DNP_BUILD_ALL_ARGS[@]}"
  BUILD_EXIT_CODE=$?

  # ....Teardown...................................................................................
  cd "${TMP_CWD}" || { echo "Return to original dir error" 1>&2 && exit 1; }
  return $BUILD_EXIT_CODE
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
  source "${SCRIPT_PATH_PARENT}/build.all.bash" || exit 1

  # ....Execute....................................................................................
  clear
  n2st::norlab_splash "${PROJECT_GIT_NAME} (${DNP_PROMPT_NAME})" "${DNP_GIT_REMOTE_URL}"
  n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  dnp::build_dn_project_multiarch_services "$@"
  FCT_EXIT_CODE=$?
  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  exit "${FCT_EXIT_CODE}"
else
  # This script is being sourced, ie: __name__="__source__"
  :
fi
