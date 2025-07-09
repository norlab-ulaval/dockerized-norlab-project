#!/bin/bash
DOCUMENTATION_BUILD_ALL_MULTIARCH=$(
  cat <<'EOF'
# =================================================================================================
# Build all images specified in docker-compose.project.build.multiarch.yaml
# i.e. pull/push from/to dockerhub sequentialy.
# Usage:
#   $ bash build.all.multiarch.bash [<any-arguments>] [--] [<any-docker-flag>]
#
# Arguments:
#   --service-names "<name1>,<name2>"     To override the list of build services.
#                                         Must be a comma separated string of service name.
#   --force-push-project-core             Pull/push from/to Dockerhub sequentialy
#                                          (instead of building images from the local image store).
#                                         Require a docker hub account.
#   -f | --file "compose.yaml"            To override the docker compose file
#                                          (default: "docker-compose.project.build.multiarch.yaml")
#   -h | --help
#
# Positional argument:
#   <any-docker-flag>                      (Optional) Any docker flag
#
# Global
#   none
#
# Requirement:
#   - Multiarch build require docker buildx be installed and a multi architecture builder be
#     configured using docker-container buildx driver with 'linux/arm64' and 'linux/amd64'.
#   - The buildx builder name 
#   - Using --force-push-project-core require a docker hub account
#
# =================================================================================================
EOF
)

# (Priority) ToDo: unit-test of flag option

# ....Function.....................................................................................
function dna::build_services_multiarch() {
  local tmp_cwd
  tmp_cwd=$(pwd)

  # ....Set env variables (pre cli)................................................................
  declare -a dna_build_all_args=()
  declare -a remaining_args=()
  local force_push_project_core=false
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
    --force-push-project-core)
      force_push_project_core=true
      shift
      ;;
    --service-names)
      # ToDo: refactor (ref task NMO-574)
      # shellcheck disable=SC2207
      dna_build_all_args+=("--service-names" "${2}")
      shift
      shift
      ;;
    -f | --file)
      the_compose_file="${2}"
      shift
      shift
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
  dna_build_all_args+=("--file" "${the_compose_file}")
  if [[ ${force_push_project_core} == true ]]; then
    dna_build_all_args+=("--force-push-project-core")
  fi
  dna_build_all_args+=("--multiarch")
  dna_build_all_args+=("${remaining_args[@]}")

  # ====Begin========================================================================================
  dna::build_services "${dna_build_all_args[@]}"
  build_exit_code=$?

  # ....Teardown...................................................................................
  cd "${tmp_cwd}" || { n2st::print_msg_error "Return to original dir error" && exit 1; }
  return $build_exit_code
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  # ....Source project shell-scripts dependencies..................................................
  script_path="$(realpath -q "${BASH_SOURCE[0]:-.}")"
  script_path_parent="$(dirname "${script_path}")"
  source "${script_path_parent}/../utils/import_dna_lib.bash" || exit 1
  source "${script_path_parent}/../utils/load_super_project_config.bash" || exit 1
  source "${script_path_parent}/build.all.bash" || exit 1

  # ....Execute....................................................................................
  if [[ "${DNA_CLEAR_CONSOLE_ACTIVATED}" == "true" ]]; then
    clear
  fi
  n2st::norlab_splash "${DNA_SPLASH_NAME_FULL:?err}" "${DNA_GIT_REMOTE_URL}" "negative"
  n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  dna::build_services_multiarch "$@"
  fct_exit_code=$?
  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  exit "${fct_exit_code}"
else
  # This script is being sourced, ie: __name__="__source__"

  # ....Pre-condition..............................................................................
  dna_error_prefix="\033[1;31m[DNA error]\033[0m"
  test -n "$( declare -f dna::import_lib_and_dependencies )" || { echo -e "${dna_error_prefix} The DNA lib is not loaded!" 1>&2 && exit 1; }
  test -n "$( declare -f n2st::print_msg )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
  test -n "${SUPER_PROJECT_ROOT}" || { echo -e "${dna_error_prefix} The super project DNA configuration is not loaded!" 1>&2 && exit 1; }
  test -n "$( declare -f dna::build_services )" || { echo -e "${dna_error_prefix} The DNA build native lib is not loaded!" 1>&2 && exit 1; }
fi
