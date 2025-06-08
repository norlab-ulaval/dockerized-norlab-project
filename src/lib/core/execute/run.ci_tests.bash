#!/bin/bash
# =================================================================================================
# Run continuous integration tests container specified in docker-compose.project.build.native.yaml.
# Require executing `build.ci_tests.bash` first.
#
# Usage:
#   $ bash build.ci_tests.bash  [<any-build.all-argument>]
#   $ bash run.ci_tests.bash [<any-docker-argument>]
#
# Notes:
#   The difference with `build.ci_tests.multiarch.bash` is that tests are only executed for the
#   host architecture, either x86 or arm64 and are executed at runtime instead of at build time.
#
# =================================================================================================


function dnp::run_ci_tests() {
  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)
  declare -a any_docker_arg=("$@")

  # ....Begin......................................................................................
  compose_file="docker-compose.project.build.native.yaml"

  if [[ $(uname -s) == "Darwin" ]] || [[ $(nvcc -V | grep 'nvcc: NVIDIA (R) Cuda compiler driver') != "nvcc: NVIDIA (R) Cuda compiler driver" ]]; then
    n2st::print_msg_warning "Host computer does not support nvidia gpu, changing container runtime to docker default."
    the_service="project-ci-tests-no-gpu"
  else
    the_service="project-ci-tests"
  fi

  docker_run_flag=("--rm")
  docker_run_flag+=("${the_service}")
  docker_run_flag+=("${any_docker_arg[@]}")
  dnp::excute_compose "--override-build-cmd" "run" "-f" "${compose_file}" "--" "${docker_run_flag[@]}"
  exit_code=$?

  if [[ ${exit_code} != 0 ]]; then
    n2st::print_msg_error "Test scripts completed with error! 👎"
  else
    n2st::print_msg_done "All test scripts have passed 👍"
  fi

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
  if [[ -z $( declare -F dnp::import_lib_and_dependencies ) ]]; then
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
  dnp::run_ci_tests "$@"
  fct_exit_code=$?
  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  exit "${fct_exit_code}"
else
  # This script is being sourced, ie: __name__="__source__"

  # ....Pre-condition..............................................................................
  dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
  test -n "$( declare -F dnp::import_lib_and_dependencies )" || { echo -e "${dnp_error_prefix} The DNP lib is not loaded!" ; exit 1 ; }
  test -n "$( declare -F n2st::print_msg )" || { echo -e "${dnp_error_prefix} The N2ST lib is not loaded!" ; exit 1 ; }
  test -n "${SUPER_PROJECT_ROOT}" || { echo -e "${dnp_error_prefix} The super project DNP configuration is not loaded!" ; exit 1 ; }
fi
