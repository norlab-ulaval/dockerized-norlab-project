#!/bin/bash
DOCUMENTATION_BUILD_ALL=$(
  cat <<'EOF'
# =================================================================================================
# Build all images specified in a compose file.
#
# Usage as a function:
#   $ source build.all.bash
#   $ dnp::build_services [OPTIONS] [--] [<any-docker-flag>]
#
# Usage as a script:
#   $ bash build.all.bash [OPTIONS] [--] [<any-docker-flag>]
#
# Options:
#   --service-names "<name1>,<name2>"     Override the list of build services.
#                                         Must be a comma separated string of service name.
#   -f | --file "compose.yaml"            Override the docker compose file
#                                         (default: "docker-compose.project.build.native.yaml")
#   --multiarch                           Build in multi-architecture mode
#   --force-push-project-core             Pull/push from/to Dockerhub sequentialy
#                                          (instead of building images from the local image store).
#   --msg-line-level CHAR                 Set consol horizontal line character when used as a fct
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
function dnp::build_services() {
  local tmp_cwd
  tmp_cwd=$(pwd)

  # ....Set env variables (pre cli))...............................................................
  declare -a remaining_args=()
  declare -a build_exit_code=()
  declare -a build_docker_flag=()
  declare -a services_names=("none")
  local force_push_project_core=false
  local compose_path="${DNP_ROOT:?err}/src/lib/core/docker"
  local the_compose_file="docker-compose.project.build.native.yaml"
  local msg_line_level="${MSG_LINE_CHAR_BUILDER_LVL1}"
#  local line_style="${MSG_LINE_STYLE_LVL1}"
  local line_style="${MSG_LINE_STYLE_LVL2}"

  # ....cli........................................................................................
  function show_help() {
    # (NICE TO HAVE) ToDo: refactor as a n2st fct (ref NMO-583)
    echo -e "${MSG_DIMMED_FORMAT}"
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "$0 --help"
    # Strip shell comment char `#` and both lines
    echo -e "${DOCUMENTATION_BUILD_ALL}" | sed '/\# ====.*/d' | sed 's/^\# //' | sed 's/^\#//'
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "${MSG_END_FORMAT}"
  }

  while [ $# -gt 0 ]; do

    case $1 in
    --service-names)
      # ToDo: refactor (ref task NMO-574)
      # shellcheck disable=SC2207
      # Override services_names
      services_names=($(echo "${2}" | tr "," "\n"))
      shift # Remove argument (--service-names)
      shift # Remove argument value
      ;;
    --force-push-project-core)
      force_push_project_core=true
      shift # Remove argument (--force-push-project-core)
      ;;
    -f | --file)
      the_compose_file="${2}"
      shift # Remove argument (-f | --file)
      shift # Remove argument value
      ;;
    --msg-line-level)
      msg_line_level="${2}"
      shift # Remove argument (--msg-line-level)
      shift # Remove argument value
      ;;
    --multiarch)
      build_docker_flag+=("--multiarch")
      shift # Remove argument (--multiarch)
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
  build_docker_flag+=("${remaining_args[@]}")

  # ====Begin======================================================================================

  # ....Fetch service list.........................................................................
  if [[ "${services_names[0]}" == "none" ]]; then
    # shellcheck disable=SC2207
    services_names=($(docker compose -f "${compose_path}/${the_compose_file}" config --services --no-interpolate))
  fi

  n2st::print_msg "Building the following services"
  for idx in "${!services_names[@]}"; do
    echo -e "$(dnp::show_indexed_prefix "$idx") ${services_names[idx]}"
  done

  # ....Execute build..............................................................................
  n2st::print_msg "force_push_project_core: ${force_push_project_core}"
  if [[ ${force_push_project_core} == false ]]; then
    n2st::print_msg "Building from the local image store."

    dnp::excute_compose --file "${the_compose_file}" "${build_docker_flag[@]}" --with-dependencies "${services_names[@]}"
    build_exit_code=("$?")

    # ....On faillure, re-run build.all one service at the time....................................
    if [[ ${build_exit_code[0]} != 0 ]]; then
      n2st::print_msg_error "Build error, re-running ${MSG_DIMMED_FORMAT}dnp::build_services${MSG_END_FORMAT} one service at the time"
      # Reset exit code buffer
      build_exit_code=()
      # Execute on all remaining service
      for each in "${services_names[@]}"; do
        dnp::excute_compose --file "${the_compose_file}" "${build_docker_flag[@]}" "${each}"
        build_exit_code+=("$?")
      done

      # Show build faillure summary
      n2st::draw_horizontal_line_across_the_terminal_window "${msg_line_level}" "${line_style}"
      n2st::print_msg "Build faillure summary\n"
      for idx in "${!build_exit_code[@]}"; do
        if [[ ${build_exit_code[idx]} != 0 ]]; then
          echo -e "$(dnp::show_indexed_prefix "$idx") ${MSG_ERROR_FORMAT}${services_names[idx]} completed build with error${MSG_END_FORMAT}"
        else
          echo -e "$(dnp::show_indexed_prefix "$idx") ${MSG_DONE_FORMAT}${services_names[idx]} completed build succesfully${MSG_END_FORMAT}"
        fi
      done
    else
      # Show build succes
      n2st::draw_horizontal_line_across_the_terminal_window "${msg_line_level}" "${line_style}"
      n2st::print_msg "Build summary\n"
      for idx in "${!services_names[@]}"; do
        echo -e "$(dnp::show_indexed_prefix "$idx") ${MSG_DONE_FORMAT}${services_names[idx]} completed build succesfully${MSG_END_FORMAT}"
      done
    fi

  else

    # Rebuild and push the core image prior to building any other images
    for idx in "${!services_names[@]}"; do
      if [[ "${services_names[idx]}" == "project-core" ]]; then
        project_core_build_idx=$idx
        # Note:
        #   - THIS WORK ON MacOs with buildx builder "docker-container:local-builder-multiarch-virtual"
        #   - THIS WORK on TC server as its the same setup use in DN
        #   - ⚠️ If you experience problem:
        #       1. check that project-core image on Dockerhub has been pushed for both arm64 and amd64
        #       2. if not, consider building and pushing manualy each arm64 and amd64 images and
        #          then merge as in DN l4t base images
        dnp::excute_compose --file "${the_compose_file}" "${build_docker_flag[@]}" --push project-core
        project_core_build_exit_code=$?
      fi
    done

    # Reset exit code buffer
    build_exit_code=()
    # Execute docker cmd on all remaining service
    for each in "${services_names[@]}"; do
      if [[ "${each}" != "project-core" ]]; then
        dnp::excute_compose --file "${the_compose_file}" "${build_docker_flag[@]}" "${each}"
        build_exit_code+=("$?")
      else
        build_exit_code+=("$project_core_build_exit_code")
      fi
    done

    # ....Show build summary.......................................................................
    n2st::print_formated_script_header "Build summary" "${msg_line_level}" "${line_style}"
    local action="build"
    for idx in "${!build_exit_code[@]}"; do
      if [[ "${services_names[idx]}" == "project-core" ]]; then
        action="build+push"
      else
        action="build"
      fi
      if [[ ${build_exit_code[idx]} != 0 ]]; then
        echo -e "$(dnp::show_indexed_prefix "$idx") ${MSG_ERROR_FORMAT}${services_names[idx]} completed ${action} with error${MSG_END_FORMAT}"
      else
        echo -e "$(dnp::show_indexed_prefix "$idx") ${MSG_DONE_FORMAT}${services_names[idx]} completed ${action} succesfully${MSG_END_FORMAT}"
      fi
    done

  fi

  # Check build faillure
  build_exit=0
  for each_build_exit_code in "${build_exit_code[@]}"; do
    build_exit=$((build_exit + each_build_exit_code))
  done

  # ....Teardown...................................................................................
  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && exit 1; }

  if [[ ${build_exit} != 0 ]]; then
    return 1
  else
    return 0
  fi
}

function dnp::show_indexed_prefix() {
  echo -e "      ${MSG_DIMMED_FORMAT}$1)${MSG_END_FORMAT}"
  return 0
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  # ....Source project shell-scripts dependencies..................................................
  script_path="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  script_path_parent="$(dirname "${script_path}")"
  source "${script_path_parent}/../utils/import_dnp_lib.bash" || exit 1
  source "${script_path_parent}/../utils/load_super_project_config.bash" || exit 1

  # ....Execute....................................................................................
  if [[ "${DNP_CLEAR_CONSOLE_ACTIVATED}" == "true" ]]; then
    clear
  fi
  n2st::norlab_splash "${DNP_SPLASH_NAME_FULL:?err}" "${DNP_GIT_REMOTE_URL}" "negative"
  n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  dnp::build_services "$@"
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
fi
