#!/bin/bash

DOCUMENTATION_BUFFER_RUN_CI_TESTS=$( cat <<'EOF'
# =================================================================================================
# Run continuous integration tests container specified in docker-compose.project.build.native.yaml.
# Require executing `build.ci_tests.bash` first.
#
# Usage:
#   $ bash build.ci_tests.bash [<any-build.all-argument>]
#   $ bash run.ci_tests.bash [<command>]
#
# Notes:
#   The difference with `build.ci_tests.multiarch.bash` is that tests are only executed for the
#   host architecture, either x86 or arm64 and are executed at runtime instead of at build time.
#
# =================================================================================================
EOF
)


function dna::run_ci_tests() {
  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)
  declare -a in_docker_command=()

  # ....cli......................................................................................
  while [[ $# -gt 0 ]]; do
      case "$1" in
          --help|-h)
              dna::command_help_menu "${DOCUMENTATION_BUFFER_RUN_CI_TESTS:?err}"
              exit 0
              ;;
          *)
              in_docker_command+=("$@")
              break
              ;;
      esac
  done

    # ....Set env variables (post cli)...............................................................
  compose_file="docker-compose.project.run.ci-tests.yaml"

  # ....Begin......................................................................................

  if [[ $(uname -s) == "Darwin" ]] || [[ $(nvcc -V 2>/dev/null | grep 'nvcc: NVIDIA (R) Cuda compiler driver') != "nvcc: NVIDIA (R) Cuda compiler driver" ]]; then
    n2st::print_msg_warning "Host computer does not support nvidia gpu, changing container runtime to docker default."
    the_service="project-ci-tests-no-gpu"
  else
    the_service="project-ci-tests"
  fi

  docker_run_flag=("--rm")
  docker_run_flag+=("${the_service}")
  docker_run_flag+=("${in_docker_command[@]}")
  dna::excute_compose "--override-build-cmd" "run" "-f" "${compose_file}" "--" "${docker_run_flag[@]}"
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
  dna::run_ci_tests "$@"
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
fi
