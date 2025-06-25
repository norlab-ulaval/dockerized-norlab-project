#!/bin/bash
# =================================================================================================
# This is the Dockerfile.ci-tests.native entrypoint and the test script run by Dockerfile.ci-tests.multiarch
#
# Will execute every tests in
# `.dockerized_norlab/configuration/project_entrypoints/project-ci-tests/test_jobs`
# directory that follow the patern `run_ci_tests.*.bash`.
#
# Usage:
#   bash dn_entrypoint.ci_test.bash
#
# Globals:
#   Read DN_PROJECT_PATH
#   Read PYTHONPATH
#   Read DN_ENTRYPOINT_TRACE_EXECUTION
#   Read DN_CONTAINER_NAME
#   Read DN_PROJECT_SERVICE_DIR
#
# =================================================================================================
MSG_ERROR_FORMAT="\033[1;31m"
MSG_DIMMED_FORMAT="\033[1;2m"
MSG_END_FORMAT="\033[0m"
dna_error_prefix="${MSG_ERROR_FORMAT}[DNA error]${MSG_END_FORMAT}"

# ====Setup========================================================================================
if [[ ! -d "${DN_PROJECT_PATH:?'Required DN environment variable is set and not empty'}/src" ]]; then
  echo -e "${dna_error_prefix} '${DN_PROJECT_PATH}/src' directory unreachable! Current working directory is '$(pwd)'" 1>&2
  exit 1
else
  cd "${DN_PROJECT_PATH}/src" || exit 1
fi

# Add the DN-project path to python path (see header Notes).
export PYTHONPATH="${DN_PROJECT_PATH:?err}:${PYTHONPATH:?err}"
# (NICE TO HAVE) ToDo: refactor PYTHONPATH logic as a fct. Either in DN container-tools or in DN-project

# ....Load library.................................................................................
source /import_dockerized_norlab_container_tools.bash || exit 1
n2st::set_which_python3_version && test -n "${PYTHON3_VERSION}" || exit 1

if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
  n2st::print_msg "Execute $0"
fi

# ====DN-project user defined logic================================================================

# ....Execute DN-project user callback.............................................................
# Sanity check
test -d "/project_entrypoints" || n2st::print_msg_error_and_exit "Dir /project_entrypoints is unreachable"
test -d "/project_entrypoints/project-ci-tests" || n2st::print_msg_error_and_exit "Dir /project_entrypoints/project-ci-tests is unreachable"

if [[ -f /project_entrypoints/dn_entrypoint.global.init.callback.bash ]]; then
  source /project_entrypoints/dn_entrypoint.global.init.callback.bash || exit 1
else
  n2st::print_msg_warning "dn_entrypoint.global.init.callback.bash unavailable"
fi

if [[ -f /project_entrypoints/project-ci-tests/dn_entrypoint.init.callback.bash ]]; then
  source /project_entrypoints/project-ci-tests/dn_entrypoint.init.callback.bash || exit 1
else
  n2st::print_msg_warning "project-ci-tests/dn_entrypoint.init.callback.bash unavailable"
fi

# ====Collect container information relevant when debugging========================================
echo -e "${MSG_DIMMED_FORMAT}"
n2st::draw_horizontal_line_across_the_terminal_window "${MSG_LINE_CHAR_UTIL}"
echo -e "DN container ${DN_CONTAINER_NAME:?err} test environment"
echo -e "Pytest tests will follow"
echo
echo
tree -L 2 "${DN_PROJECT_PATH}/.dockerized_norlab/configuration/project_entrypoints/project-ci-tests"
tree -L 3 "$DN_PROJECT_PATH"
echo
echo
printenv
n2st::draw_horizontal_line_across_the_terminal_window "${MSG_LINE_CHAR_UTIL}"
echo -e "${MSG_END_FORMAT}"

# ====Execute tests================================================================================
declare -a exit_codes=()
test -d "${DN_PROJECT_SERVICE_DIR:?err}/test_jobs" || { echo -e "${dna_error_prefix} ${DN_PROJECT_SERVICE_DIR:?err}/test_jobs is unreachable" 1>&2 && exit 1; }
for each_file_path in "${DN_PROJECT_SERVICE_DIR}"/test_jobs/run_ci_tests.*.bash ; do
  n2st::print_formated_script_header "$(basename $each_file_path)" "${MSG_LINE_CHAR_INSTALLER}"
  bash "${each_file_path}"
  exit_codes+=("$?")
  n2st::print_formated_script_footer "$(basename $each_file_path)" "${MSG_LINE_CHAR_INSTALLER}"
done

# ====Teardown=====================================================================================
for idx in "${exit_codes[@]}" ; do
  if [[ ${idx} != 0 ]]; then
    # Test scripts completed with error!
    exit 1
  else
    # All test scripts have passed!
    exit 0
  fi
done
