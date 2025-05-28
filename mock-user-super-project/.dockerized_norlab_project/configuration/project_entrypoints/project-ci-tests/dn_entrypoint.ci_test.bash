#!/bin/bash
# =================================================================================================
# Dockerfile.ci-tests.native entrypoint.
# Will execute every tests in
# `.dockerized_norlab_project/configuration/project_entrypoints/project-ci-tests/test_jobs`
# directory that follow the patern `run_ci_tests.*.bash`.
#
# Usage:
#   bash dn_entrypoint.ci_test.bash
#
# Globals:
#   Read 'DN_PROJECT_PATH'
#
# =================================================================================================
MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"

# ====Setup========================================================================================
if [[ ! -d "${DN_PROJECT_PATH:?'Required DN environment variable is set and not empty'}/src" ]]; then
  echo -e " ${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} '${DN_PROJECT_PATH}/src' directory unreachable! Current working directory is '$(pwd)'" 1>&2
  exit 1
else
  cd "${DN_PROJECT_PATH}/src" || exit 1
fi

# Add the DN-project path to python path (see header Notes).
export PYTHONPATH="${DN_PROJECT_PATH:?err}:${PYTHONPATH:?err}"
# (NICE TO HAVE) ToDo: refactor PYTHONPATH logic as a fct. Either in DN container-tools or in DN-project

# ====Collect container information relevant when debugging========================================
echo -e "${MSG_DIMMED_FORMAT}"
n2st::draw_horizontal_line_across_the_terminal_window "${MSG_LINE_CHAR_UTIL}"
echo -e "DN container ${DN_CONTAINER_NAME} test environment"
echo -e "Pytest tests will follow"
echo
echo
tree -L 1 "${DN_PROJECT_PATH}/.dockerized_norlab_project/configuration/project_entrypoints/project-ci-tests"
tree -L 3 "$DN_PROJECT_PATH"
echo
echo
printenv
n2st::draw_horizontal_line_across_the_terminal_window "${MSG_LINE_CHAR_UTIL}"
echo -e "${MSG_END_FORMAT}"

# ====Execute tests================================================================================
declare -a EXIT_CODES
test -d "/project-ci-tests/test_jobs" || exit 1
for each_file_path in /project-ci-tests/test_jobs/run_ci_tests.*.bash ; do
  n2st::print_formated_script_header "$(basename $each_file_path)" "${MSG_LINE_CHAR_INSTALLER}"
  bash "${each_file_path}"
  EXIT_CODES+=("$?")
  n2st::print_formated_script_footer "$(basename $each_file_path)" "${MSG_LINE_CHAR_INSTALLER}"
done

# ....Release......................................................................................

for idx in "${EXIT_CODES[@]}" ; do
  if [[ ${idx} != 0 ]]; then
    # Test scripts completed with error!
    exit 1
  else
    # All test scripts have passed!
    exit 0
  fi
done
