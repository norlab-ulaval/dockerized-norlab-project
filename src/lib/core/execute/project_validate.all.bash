#!/bin/bash
# =================================================================================================
# Convenient script for testing config and dry-run build all images specified
# in docker-compose.project.build.native.yaml and docker-compose.project.build.multiarch.yaml
#
# Usage:
#   $ bash project_validate.all.bash
#
# =================================================================================================
if [[ "${DNP_CLEAR_CONSOLE_ACTIVATED}" == "true" ]]; then
  clear
fi
pushd "$(pwd)" >/dev/null || exit 1


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
source "${script_path_parent}/build.all.bash" || exit 1
source "${script_path_parent}/build.all.multiarch.bash" || exit 1

# ====Begin========================================================================================
n2st::norlab_splash "${PROJECT_GIT_NAME} (${DNP_PROMPT_NAME})" "${DNP_GIT_REMOTE_URL}"
n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"

config_test_exit_code=()
build_test_exit_code=()

# ....Config test..................................................................................
n2st::print_msg "Begin config test"

config_test_compose_file_list=(
  "docker-compose.project.build.native.yaml"
  "docker-compose.project.build.multiarch.yaml"
  "docker-compose.project.run.darwin.yaml"
  "docker-compose.project.run.jetson.yaml"
  "docker-compose.project.run.linux-x86.yaml"
  "docker-compose.project.run.slurm.yaml"
)

n2st::print_msg "Will config test the following compose files:"
for idx in "${!config_test_compose_file_list[@]}"; do
  echo "              $idx › ${config_test_compose_file_list[idx]}"
done

n2st::print_msg "Begin docker compose config test"
for each_compose in "${config_test_compose_file_list[@]}"; do
  n2st::print_formated_script_header "Test ${MSG_DIMMED_FORMAT}${each_compose}${MSG_END_FORMAT} config" ">"
  add_docker_flag=()
  add_docker_flag+=("--override-build-cmd" "config")
  add_docker_flag+=("--file" "${each_compose}")
  add_docker_flag+=("--dry-run")
  if [[ "${each_compose}" =~ .*".build.".*".yaml" ]]; then
    dnp::excute_compose_on_dn_project_image "${add_docker_flag[@]}"
  elif [[ "${each_compose}" =~ .*".run.slurm.yaml" ]]; then
    dnp::excute_compose_on_dn_project_image "${add_docker_flag[@]}"
  elif [[ "${each_compose}" =~ .*".run.".*".yaml" ]]; then
    dnp::excute_compose_on_dn_project_image "${add_docker_flag[@]}" "project-develop"
    dnp::excute_compose_on_dn_project_image "${add_docker_flag[@]}" "project-deploy"
  fi
  config_test_exit_code+=("$?")
  n2st::print_formated_script_footer "Test ${MSG_DIMMED_FORMAT}${each_compose}${MSG_END_FORMAT} config" "<"
done

n2st::print_formated_script_footer "testing config" "${MSG_LINE_CHAR_BUILDER_LVL1}"

# ....Dry-run build test...........................................................................
n2st::print_formated_script_header "build in dry-run mode testing" "${MSG_LINE_CHAR_BUILDER_LVL1}"

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
  n2st::print_formated_script_header "Test ${MSG_DIMMED_FORMAT}${each_compose}${MSG_END_FORMAT} config" ">"
  add_docker_flag=()
  add_docker_flag+=("--file" "${each_compose}")
  add_docker_flag+=("--dry-run")
  if [[ "${each_compose}" =~ .*".multiarch.yaml" ]]; then
    dnp::build_dn_project_multiarch_services --msg-line-level "${MSG_LINE_CHAR_BUILDER_LVL2}" "${add_docker_flag[@]}"
  else
    dnp::build_dn_project_services --msg-line-level "${MSG_LINE_CHAR_BUILDER_LVL2}" "${add_docker_flag[@]}"
  fi
  build_test_exit_code+=("$?")
  n2st::print_formated_script_footer "Test ${MSG_DIMMED_FORMAT}${each_compose}${MSG_END_FORMAT} config" "<"
done

n2st::print_msg "Completed build in dry-run mode tests"

# ....Config and dry-run build test summary........................................................
n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"

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

# ====Teardown=====================================================================================
popd >/dev/null || exit 1

# ....Set exit code................................................................................
test_exit_code=("${config_test_exit_code[@]}" "${build_test_exit_code[@]}")
test_exit=0
for each_build_exit_code in "${test_exit_code[@]}"; do
  test_exit=$((test_exit + each_build_exit_code))
done

if [[ ${test_exit} != 0 ]]; then
  exit 1
else
  exit 0
fi
