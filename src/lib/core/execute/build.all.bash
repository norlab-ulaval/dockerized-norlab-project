#!/bin/bash
DOCUMENTATION_BUILD_ALL=$(
  cat <<'EOF'
# =================================================================================================
# Build all images specified in a compose file.
#
# Usage as a function:
#   $ source build.all.bash
#   $ dna::build_services [OPTIONS] [--] [<any-docker-flag>]
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
#                                         Require a docker hub account.
#   --msg-line-level CHAR                 Set consol horizontal line character when used as a fct
#   --target-release-branch BRANCH        Traget release branch for release image
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
# (Priority) ToDo: unit-test "--target-release-branch" logic (ref task NMO-681)

# ....Function.....................................................................................
function dna::build_services() {
  local tmp_cwd
  tmp_cwd=$(pwd)

  cd "${SUPER_PROJECT_ROOT:?err}" || exit 1
  local original_branch
  original_branch="$(git branch --show-current)"

  # ....Set env variables (pre cli))...............................................................
  declare -a remaining_args=()
  declare -a build_docker_flag=()
  declare -a services_names=("none")
  declare -a project_core_services=()
  declare -a non_project_core_services=()
  declare -a non_project_core_target_branch_services=()
  declare -a build_exit_code=()
  declare -i build_exit
  declare -a build_core_exit_codes=()
  declare -a build_non_core_exit_codes=()
  local target_release_branch
  local force_push_project_core=false
  local compose_path="${DNA_ROOT:?err}/src/lib/core/docker"
  local the_compose_file="docker-compose.project.build.native.yaml"
  local msg_line_level="${MSG_LINE_CHAR_BUILDER_LVL1}"
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
      shift
      shift
      ;;
    --force-push-project-core)
      force_push_project_core=true
      shift
      ;;
    -f | --file)
      the_compose_file="${2}"
      shift
      shift
      ;;
    --msg-line-level)
      msg_line_level="${2}"
      shift
      shift
      ;;
    --target-release-branch)
      target_release_branch="${2}"
      shift
      shift
      ;;
    --multiarch)
      build_docker_flag+=("--multiarch")
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
  build_docker_flag+=( --with-dependencies "${remaining_args[@]}")

  # ====Begin======================================================================================

  # ....Fetch service list.........................................................................
  if [[ "${services_names[0]}" == "none" ]]; then
    # shellcheck disable=SC2207
    services_names=($(dna::excute_compose --verbosity 0 --compose-path "${compose_path}" -f "${the_compose_file}" --docker-cmd config -- --services --no-interpolate))
  fi

  n2st::print_msg "Building the following services"
  for idx in "${!services_names[@]}"; do
    echo -e "$(dna::show_indexed_prefix "$idx") ${services_names[idx]}"
  done

  # ....Split service list.........................................................................
  local each_service
  for each_service in "${services_names[@]}"; do
      if [[ "$each_service" == project-core* ]]; then
          project_core_services+=("$each_service")
      elif [[ "$each_service" == project-release* ]]; then
          non_project_core_target_branch_services+=("$each_service")
      else
          non_project_core_services+=("$each_service")
      fi
  done
  if [[ "${DNA_DEBUG}" == "true" ]]; then
    n2st::print_msg "Services with project-core prefix: ${project_core_services[*]}"
    n2st::print_msg "Services without project-core prefix: ${non_project_core_services[*]}"
  fi

  # ....Execute build..............................................................................
  n2st::print_msg "force_push_project_core: ${force_push_project_core}"
  if [[ ${force_push_project_core} == false ]]; then
    n2st::print_msg "Building from the local image store.."

    build_exit_code=()
    if [[ "${project_core_services[*]}" =~ "project-core-pre" ]]; then
      dna::excute_compose --file "${the_compose_file}" "${build_docker_flag[@]}" project-core-pre
      build_exit_code+=($?)
    fi
    if [[ "${project_core_services[*]}" =~ "project-core-user" ]]; then
      dna::excute_compose --file "${the_compose_file}" "${build_docker_flag[@]}" project-core-user
      build_exit_code+=($?)
    fi
    if [[ "${project_core_services[*]}" =~ "project-core" ]]; then
      dna::excute_compose --file "${the_compose_file}" "${build_docker_flag[@]}" project-core
      build_exit_code+=($?)
    fi

    # Build develope, deploy, ci-tests, slurm images
    if [[ "${#non_project_core_services[@]}" -gt 0 ]]; then
      dna::excute_compose --file "${the_compose_file}" "${build_docker_flag[@]}" "${non_project_core_services[@]}"
      build_exit_code+=("$?")
    fi

    # Build release images
    if [[ "${#non_project_core_target_branch_services[@]}" -gt 0 ]]; then
      dna::checkout_target_branch  "${target_release_branch}" "${original_branch}"
      dna::excute_compose --file "${the_compose_file}" "${build_docker_flag[@]}" "${non_project_core_target_branch_services[@]}"
      build_exit_code+=("$?")
      dna::checkout_original_branch "${original_branch}"
    fi

    build_exit=0
    declare -i each_build_exit_code
    for each_build_exit_code in "${build_exit_code[@]}"; do
      build_exit=$((build_exit + each_build_exit_code))
    done

    # ....On faillure, re-run build.all one service at the time....................................
    if [[ $build_exit -ne 0 ]]; then
      n2st::print_msg_error "Build error, re-running ${MSG_DIMMED_FORMAT}dna::build_services${MSG_END_FORMAT} one service at the time"
      build_core_exit_codes=()
      for each_core in "${project_core_services[@]}"; do
        n2st::print_msg "Building ${each_core}..."
        dna::excute_compose --file "${the_compose_file}" "${build_docker_flag[@]}" "${each_core}"
        build_core_exit_codes+=($?)
      done

      # Reset exit code buffer
      build_non_core_exit_codes=()
      # Execute docker cmd on all remaining service except release
      for each in "${non_project_core_services[@]}"; do
        n2st::print_msg "Building ${each}..."
        dna::excute_compose --file "${the_compose_file}" "${build_docker_flag[@]}" "${each}"
        build_non_core_exit_codes+=("$?")
      done

      # Reset exit code buffer
      build_non_core_target_release_branch_exit_codes=()
      # Execute docker cmd on all release images
      if [[ "${#non_project_core_target_branch_services[@]}" -gt 0 ]]; then
        dna::checkout_target_branch  "${target_release_branch}" "${original_branch}"
        for each in "${non_project_core_target_branch_services[@]}"; do
          n2st::print_msg "Building ${each}..."
          dna::excute_compose --file "${the_compose_file}" "${build_docker_flag[@]}" "${each}"
          build_non_core_target_release_branch_exit_codes+=("$?")
        done
        dna::checkout_original_branch "${original_branch}"
      fi


      # Show build faillure summary
      n2st::draw_horizontal_line_across_the_terminal_window "${msg_line_level}" "${line_style}"
      n2st::print_msg "Build faillure summary\n"
      build_exit_code=( "${build_core_exit_codes[@]}" "${build_non_core_exit_codes[@]}"  "${build_non_core_target_release_branch_exit_codes[@]}" )
      merged_services_names=( "${project_core_services[@]}" "${non_project_core_services[@]}" )
      for idx in "${!build_exit_code[@]}"; do
        if [[ ${build_exit_code[idx]} != 0 ]]; then
          echo -e "$(dna::show_indexed_prefix "$idx") ${MSG_ERROR_FORMAT}${merged_services_names[idx]} completed build with error${MSG_END_FORMAT}"
        else
          echo -e "$(dna::show_indexed_prefix "$idx") ${MSG_DONE_FORMAT}${merged_services_names[idx]} completed build succesfully${MSG_END_FORMAT}"
        fi
      done
    else
      # Show build success
      n2st::draw_horizontal_line_across_the_terminal_window "${msg_line_level}" "${line_style}"
      n2st::print_msg "Build summary\n"
      for idx in "${!services_names[@]}"; do
        echo -e "$(dna::show_indexed_prefix "$idx") ${MSG_DONE_FORMAT}${services_names[idx]} completed build succesfully${MSG_END_FORMAT}"
      done
    fi

  else

    n2st::print_msg "Begin online build"
    n2st::print_msg "Building project-core..."
    # Rebuild and push the core image prior to building any other images
    # Note:
    #   - THIS WORK ON MacOs with buildx builder "docker-container:local-builder-multiarch-virtual"
    #   - THIS WORK on TC server as its the same setup use in DN
    #   - ⚠️ If you experience problem:
    #       1. check that project-core image on Dockerhub has been pushed for both arm64 and amd64
    #       2. if not, consider building and pushing manualy each arm64 and amd64 images and
    #          then merge as in DN l4t base images
    build_core_exit_codes=()
    if [[ "${project_core_services[*]}" =~ "project-core-pre" ]]; then
      dna::excute_compose --file "${the_compose_file}" "${build_docker_flag[@]}" --push project-core-pre
      build_core_exit_codes+=($?)
    fi
    if [[ "${project_core_services[*]}" =~ "project-core-user" ]]; then
      dna::excute_compose --file "${the_compose_file}" "${build_docker_flag[@]}" --push project-core-user
      build_core_exit_codes+=($?)
    fi
    if [[ "${project_core_services[*]}" =~ "project-core" ]]; then
      dna::excute_compose --file "${the_compose_file}" "${build_docker_flag[@]}" --push project-core
      build_core_exit_codes+=($?)
    fi
    n2st::print_msg "Completed project-core build and push."

    # Reset exit code buffer
    build_non_core_exit_codes=()
    # Execute docker cmd on all remaining service except release
    for each in "${non_project_core_services[@]}"; do
      n2st::print_msg "Building ${each}..."
      dna::excute_compose --file "${the_compose_file}" "${build_docker_flag[@]}" "${each}"
      build_non_core_exit_codes+=("$?")
      n2st::print_msg "Completed ${each} build."
    done

    # Reset exit code buffer
    build_non_core_target_release_branch_exit_codes=()
    # Execute docker cmd on all release images
    if [[ "${#non_project_core_target_branch_services[@]}" -gt 0 ]]; then
      dna::checkout_target_branch  "${target_release_branch}" "${original_branch}"
      for each in "${non_project_core_target_branch_services[@]}"; do
        n2st::print_msg "Building ${each}..."
        dna::excute_compose --file "${the_compose_file}" "${build_docker_flag[@]}" "${each}"
        build_non_core_target_release_branch_exit_codes+=("$?")
      done
      dna::checkout_original_branch "${original_branch}"
    fi


    build_exit_code=( "${build_core_exit_codes[@]}" "${build_non_core_exit_codes[@]}" "${build_non_core_target_release_branch_exit_codes[@]}" )
    merged_services_names=( "${project_core_services[@]}" "${non_project_core_services[@]}" )

    # ....Show build summary.......................................................................
    n2st::print_formated_script_header "Build summary" "${msg_line_level}" "${line_style}"
    local action="build"
    for idx in "${!build_exit_code[@]}"; do
      if [[ "${merged_services_names[idx]}" =~ "project-core".* ]]; then
        action="build+push"
      else
        action="build"
      fi
      if [[ ${build_exit_code[idx]} != 0 ]]; then
        echo -e "$(dna::show_indexed_prefix "$idx") ${MSG_ERROR_FORMAT}${merged_services_names[idx]} completed ${action} with error${MSG_END_FORMAT}"
      else
        echo -e "$(dna::show_indexed_prefix "$idx") ${MSG_DONE_FORMAT}${merged_services_names[idx]} completed ${action} succesfully${MSG_END_FORMAT}"
      fi
    done

  fi

  # Check build faillure
  build_exit=0
  declare -i each_build_exit_code
  for each_build_exit_code in "${build_exit_code[@]}"; do
    build_exit=$((build_exit + each_build_exit_code))
  done

  # ....Teardown...................................................................................
  cd "${tmp_cwd}" || { n2st::print_msg_error "Return to original dir error" && exit 1; }

  if [[ ${build_exit} != 0 ]]; then
    return 1
  else
    return 0
  fi
}

function dna::show_indexed_prefix() {
  echo -e "      ${MSG_DIMMED_FORMAT}$1)${MSG_END_FORMAT}"
  return 0
}

function dna::checkout_target_branch() {
  local original_branch
  original_branch=$1
  target_branch=$2
  local tmp_cwd
  tmp_cwd=$(pwd)

  cd "${SUPER_PROJECT_ROOT:?err}" || exit 1

  n2st::print_msg "Check git status..."
  git status || return 1

  if [[ "${original_branch}" != "${target_branch}" ]]; then
    n2st::print_msg "Checking out target branch ${target_branch}..."

    # Fetch all remote branches and tags
    git fetch --tags origin >/dev/null 2>&1 || {
        n2st::print_msg_error "Failed to fetch remote branches and tags from origin";
        cd "${tmp_cwd}";
        return 1;
    }

    # Checkout the target branch
    if ! git checkout "${target_branch}" >/dev/null 2>&1; then
        n2st::print_msg_error "Failed to checkout branch ${target_branch}"
        n2st::print_msg_error "Note on git checkout faillure: If you experience problem checking out a tag, use prefix 'tags/<my-tags-name>' e.g.: target_branch=\"tags/v0.0.1\""
        cd "${tmp_cwd}" || { n2st::print_msg_error "Return to original dir error"; return 1; }
        return 1
    fi

    # Pull the latest changes
    n2st::print_msg "Pull latest version from '${target_branch}' branch..."
    if ! git pull --recurse-submodules origin "${target_branch}" >/dev/null 2>&1; then
        n2st::print_msg_error "Failed to pull branch latest commits from ${target_branch}"
        cd "${tmp_cwd}" || { n2st::print_msg_error "Return to original dir error"; return 1; }
        return 1
    fi

  fi
  return 0
}

function  dna::checkout_original_branch() {
  local original_branch
  original_branch=$1
  local tmp_cwd
  tmp_cwd=$(pwd)

  cd "${SUPER_PROJECT_ROOT:?err}" || exit 1
  if [[ "$(git branch --show-current)" != "${original_branch}" ]]; then
    echo
    n2st::print_msg "Checkout original branch ${original_branch}..."
    if ! git checkout "${original_branch}" >/dev/null 2>&1; then
        n2st::print_msg_error "Failed to checkout branch ${target_branch}"
        cd "${tmp_cwd}" || { n2st::print_msg_error "Return to original dir error"; return 1; }
        return 1
    fi
  fi
  return 0
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  # ....Source project shell-scripts dependencies..................................................
  script_path="$(realpath -q "${BASH_SOURCE[0]:-.}")"
  script_path_parent="$(dirname "${script_path}")"
  source "${script_path_parent}/../utils/import_dna_lib.bash" || exit 1
  source "${script_path_parent}/../utils/load_super_project_config.bash" || exit 1

  # ....Execute....................................................................................
  if [[ "${DNA_CLEAR_CONSOLE_ACTIVATED}" == "true" ]]; then
    clear
  fi
  n2st::norlab_splash "${DNA_SPLASH_NAME_FULL:?err}" "${DNA_GIT_REMOTE_URL}" "negative"
  n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  dna::build_services "$@"
  fct_exit_code=$?
  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  exit "${fct_exit_code}"
else
  # This script is being sourced, ie: __name__="__source__"

  # ....Pre-condition..............................................................................
  dna_error_prefix="\033[1;31m[dna error]\033[0m"
  test -n "$( declare -f dna::import_lib_and_dependencies )" || { echo -e "${dna_error_prefix} The DNA lib is not loaded!" 1>&2 && exit 1; }
  test -n "$( declare -f n2st::print_msg )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
  test -n "${SUPER_PROJECT_ROOT}" || { echo -e "${dna_error_prefix} The super project DNA configuration is not loaded!" 1>&2 && exit 1; }
fi
