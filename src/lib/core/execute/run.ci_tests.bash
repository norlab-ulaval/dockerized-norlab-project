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


# ....Source project shell-scripts dependencies..................................................
script_path="$(realpath "${BASH_SOURCE[0]:-'.'}")"
script_path_parent="$(dirname "${script_path}")"
if [[ ! $( dnp::is_lib_loaded 2>/dev/null >/dev/null )  ]]; then
  source "${script_path_parent}/../utils/import_dnp_lib.bash" || exit 1
  source "${script_path_parent}/../utils/execute_compose.bash" || exit 1
fi
if [[ -z ${SUPER_PROJECT_ROOT} ]]; then
  source "${script_path_parent}/../utils/load_super_project_config.bash" || exit 1
fi



# ====Execute tests================================================================================
n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"

# ====Begin========================================================================================
compose_file="docker-compose.project.build.native.yaml"

if [[ $(uname -s) == "Darwin" ]] || [[ $(nvcc -V | grep 'nvcc: NVIDIA (R) Cuda compiler driver') != "nvcc: NVIDIA (R) Cuda compiler driver" ]]; then
  n2st::print_msg_warning "Host computer does not support nvidia gpu, changing container runtime to docker default."
  the_service="project-ci-tests-no-gpu"
else
  the_service="project-ci-tests"
fi

docker_run_flag=("--rm")
docker_run_flag+=("${the_service}")
docker_run_flag+=("$@")
dnp::excute_compose_on_dn_project_image "--override-build-cmd" "run" "-f" "${compose_file}" "--" "${docker_run_flag[@]}"
exit_code=$?

# ====Teardown=====================================================================================
n2st::norlab_splash "${PROJECT_GIT_NAME} (${DNP_PROMPT_NAME})" "${DNP_GIT_REMOTE_URL}"

if [[ ${exit_code} != 0 ]]; then
  n2st::print_msg_error "Test scripts completed with error! 👎"
  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  exit 1
else
  n2st::print_msg_done "All test scripts have passed 👍"
  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  exit 0
fi

