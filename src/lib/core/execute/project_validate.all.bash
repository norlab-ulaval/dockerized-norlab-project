#!/bin/bash
# =================================================================================================
# Convenient script for testing config and dry-run build all images specified
# in docker-compose.project.build.native.yaml and docker-compose.project.build.multiarch.yaml
#
# Usage:
#   $ bash project_validate.all.bash
#
# =================================================================================================

function dnp::project_validate_all() {
  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)
  local line_format="${MSG_LINE_CHAR_BUILDER_LVL1}"
  local line_style="${MSG_LINE_STYLE_LVL2}"

  declare -a config_test_compose_file_list=()
  declare -a dryrun_compose_file_list=()
  declare -a config_test_exit_code=()
  declare -a build_test_exit_code=()

  # ....Config test..................................................................................
  n2st::print_msg "Begin config test"

  config_test_compose_file_list=(
    "docker-compose.project.build.native.yaml"
    "docker-compose.project.build.multiarch.yaml"
    "docker-compose.project.run.darwin.yaml"
    "docker-compose.project.run.jetson.yaml"
    "docker-compose.project.run.linux-x86.yaml"
    "docker-compose.project.run.ci-tests.yaml"
    "docker-compose.project.run.slurm.yaml"
  )

  n2st::print_msg "Will config test the following compose files:"
  for idx in "${!config_test_compose_file_list[@]}"; do
    echo "              $idx › ${config_test_compose_file_list[idx]}"
  done

  n2st::print_msg "Begin docker compose config test"
  for each_compose in "${config_test_compose_file_list[@]}"; do
    n2st::print_formated_script_header "Test ${MSG_DIMMED_FORMAT}${each_compose}${MSG_END_FORMAT} config" "\\" "${line_style}"
    declare -a config_flag=()
    config_flag+=("--override-build-cmd" "config")
    config_flag+=("--file" "${each_compose}")
    config_flag+=("--" "--dry-run")
    if [[ "${each_compose}" =~ .*".build.".*".yaml" ]]; then
      dnp::excute_compose "${config_flag[@]}"
    elif [[ "${each_compose}" =~ .*".run.slurm.yaml" ]]; then
      dnp::excute_compose "${config_flag[@]}"
    elif [[ "${each_compose}" =~ .*".run.ci-tests.yaml" ]]; then
      dnp::excute_compose "${config_flag[@]}"
    elif [[ "${each_compose}" =~ .*".run.".*".yaml" ]]; then
      dnp::excute_compose "${config_flag[@]}" "project-develop"
      dnp::excute_compose "${config_flag[@]}" "project-deploy"
    fi
    config_test_exit_code+=("$?")
    n2st::print_formated_script_footer "Test ${MSG_DIMMED_FORMAT}${each_compose}${MSG_END_FORMAT} config" "/" "${line_style}"
  done

  n2st::print_formated_script_footer "testing config" "${line_format}" "${line_style}"

  # ....Dry-run build test...........................................................................
  n2st::print_formated_script_header "build in dry-run mode testing" "${line_format}" "${line_style}"

  dryrun_compose_file_list=(
    "docker-compose.project.build.native.yaml"
    "docker-compose.project.build.multiarch.yaml"
  )

  n2st::print_msg "Will dry-run build the following compose files:"
  for idx in "${!dryrun_compose_file_list[@]}"; do
    echo "              $idx › ${dryrun_compose_file_list[idx]}"
  done

  n2st::print_msg "Begin docker compose build --dry-run test"
  for each_compose in "${dryrun_compose_file_list[@]}"; do
    n2st::print_formated_script_header "Test ${MSG_DIMMED_FORMAT}${each_compose}${MSG_END_FORMAT} config" "\\" "${line_style}"
    declare -a build_flag=()
    build_flag+=("--file" "${each_compose}")
    build_flag+=("--" "--dry-run")
    if [[ "${each_compose}" =~ .*".multiarch.yaml" ]]; then
      dnp::build_services_multiarch --msg-line-level "${MSG_LINE_CHAR_BUILDER_LVL2}" "${build_flag[@]}"
    else
      dnp::build_services --msg-line-level "${MSG_LINE_CHAR_BUILDER_LVL2}" "${build_flag[@]}"
    fi
    build_test_exit_code+=("$?")
    n2st::print_formated_script_footer "Test ${MSG_DIMMED_FORMAT}${each_compose}${MSG_END_FORMAT} config" "/" "${line_style}"
  done

  n2st::print_msg "Completed build in dry-run mode tests"

  # ....Config and dry-run build test summary........................................................
  n2st::norlab_splash "${DNP_SPLASH_NAME_FULL:?err}" "${DNP_GIT_REMOTE_URL}" "negative"

  n2st::print_msg "Config test summary"
  for idx in "${!config_test_exit_code[@]}"; do
    if [[ ${config_test_exit_code[idx]} != 0 ]]; then
      echo -e "    ${MSG_ERROR_FORMAT}${config_test_compose_file_list[idx]} Completed docker compose config test with error${MSG_END_FORMAT}"
    else
      echo -e "    ${MSG_DONE_FORMAT}${config_test_compose_file_list[idx]} Completed docker compose config test${MSG_END_FORMAT}"
    fi
  done

  n2st::print_msg "Dry-run build test summary"
  for idx in "${!build_test_exit_code[@]}"; do
    if [[ ${build_test_exit_code[idx]} != 0 ]]; then
      echo -e "    ${MSG_ERROR_FORMAT}${dryrun_compose_file_list[idx]} dry-run build completed with error${MSG_END_FORMAT}"
    else
      echo -e "    ${MSG_DONE_FORMAT}${dryrun_compose_file_list[idx]} dry-run build completed${MSG_END_FORMAT}"
    fi
  done

  # ....Set exit code................................................................................
  declare -a test_exit_code=()
  test_exit_code=("${config_test_exit_code[@]}" "${build_test_exit_code[@]}")
  local test_exit=0
  for each_build_exit_code in "${test_exit_code[@]}"; do
    test_exit=$((test_exit + each_build_exit_code))
  done


  # ....Teardown...................................................................................
  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  if [[ ${test_exit} != 0 ]]; then
    return 1
  else
    return 0
  fi
}


# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  # ....Source project shell-scripts dependencies..................................................
  script_path="$(realpath -q "${BASH_SOURCE[0]:-.}")"
  script_path_parent="$(dirname "${script_path}")"
  if [[ -z $( declare -f dnp::import_lib_and_dependencies ) ]]; then
    source "${script_path_parent}/../utils/import_dnp_lib.bash" || exit 1
    source "${script_path_parent}/../utils/execute_compose.bash" || exit 1
  fi
  if [[ -z ${SUPER_PROJECT_ROOT} ]]; then
    source "${script_path_parent}/../utils/load_super_project_config.bash" || exit 1
  fi
  source "${script_path_parent}/build.all.bash" || exit 1
  source "${script_path_parent}/build.all.multiarch.bash" || exit 1

  # ....Execute....................................................................................
  if [[ "${DNP_CLEAR_CONSOLE_ACTIVATED}" == "true" ]]; then
    clear
  fi
  n2st::norlab_splash "${DNP_SPLASH_NAME_FULL:?err}" "${DNP_GIT_REMOTE_URL}" "negative"
  n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  dnp::project_validate_all "$@"
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
  test -n "$( declare -f dnp::build_services_multiarch )" || { echo -e "${dnp_error_prefix} The DNP build multiarch lib is not loaded!" ; exit 1 ; }

fi
