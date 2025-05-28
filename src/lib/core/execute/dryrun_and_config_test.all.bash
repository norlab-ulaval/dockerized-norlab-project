#!/bin/bash
# =================================================================================================
# Convenient script for testing config and dry-run build all images specified
# in docker-compose.project.build.native.yaml and docker-compose.project.build.multiarch.yaml
#
# Usage:
#   $ bash dryrun_and_config_test.all.bash
#
# =================================================================================================
clear
pushd "$(pwd)" >/dev/null || exit 1


# ....Source project shell-scripts dependencies..................................................
SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]:-'.'}")"
SCRIPT_PATH_PARENT="$(dirname "${SCRIPT_PATH}")"
source "${SCRIPT_PATH_PARENT}/../utils/import_dnp_lib.bash" || exit 1
source "${SCRIPT_PATH_PARENT}/../utils/load_super_project_config.bash" || exit 1
source "${SCRIPT_PATH_PARENT}/../utils/execute_compose.bash" || exit 1
source "${SCRIPT_PATH_PARENT}/build.all.bash" || exit 1
source "${SCRIPT_PATH_PARENT}/build.all.multiarch.bash" || exit 1

# ====Begin========================================================================================
n2st::norlab_splash "${PROJECT_GIT_NAME} (${DNP_PROMPT_NAME})" "${DNP_GIT_REMOTE_URL}"
n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"

CONFIG_TEST_EXIT_CODE=()
BUILD_TEST_EXIT_CODE=()

# ....Config test..................................................................................
n2st::print_msg "Begin config test"

CONFIG_TEST_COMPOSE_FILE_LIST=(
  "docker-compose.project.build.native.yaml"
  "docker-compose.project.build.multiarch.yaml"
  "docker-compose.project.run.darwin.yaml"
  "docker-compose.project.run.jetson.yaml"
  "docker-compose.project.run.linux-x86.yaml"
  "docker-compose.project.run.slurm.yaml"
)

n2st::print_msg "Will config test the following compose files:"
for idx in "${!CONFIG_TEST_COMPOSE_FILE_LIST[@]}"; do
  echo "              $idx › ${CONFIG_TEST_COMPOSE_FILE_LIST[idx]}"
done

n2st::print_msg "Begin docker compose config test"
for each_compose in "${CONFIG_TEST_COMPOSE_FILE_LIST[@]}"; do
  n2st::print_formated_script_header "Test ${MSG_DIMMED_FORMAT}${each_compose}${MSG_END_FORMAT} config" ">"
  ADD_DOCKER_FLAG=()
  ADD_DOCKER_FLAG+=("--override-build-cmd" "config")
  ADD_DOCKER_FLAG+=("--file" "${each_compose}")
  ADD_DOCKER_FLAG+=("--dry-run")
  if [[ "${each_compose}" =~ .*".build.".*".yaml" ]]; then
    dnp::excute_compose_on_dn_project_image "${ADD_DOCKER_FLAG[@]}"
  elif [[ "${each_compose}" =~ .*".run.slurm.yaml" ]]; then
    dnp::excute_compose_on_dn_project_image "${ADD_DOCKER_FLAG[@]}"
  elif [[ "${each_compose}" =~ .*".run.".*".yaml" ]]; then
    dnp::excute_compose_on_dn_project_image "${ADD_DOCKER_FLAG[@]}" "project-develop"
    dnp::excute_compose_on_dn_project_image "${ADD_DOCKER_FLAG[@]}" "project-deploy"
  fi
  CONFIG_TEST_EXIT_CODE+=("$?")
  n2st::print_formated_script_footer "Test ${MSG_DIMMED_FORMAT}${each_compose}${MSG_END_FORMAT} config" "<"
done

n2st::print_formated_script_footer "testing config" "${MSG_LINE_CHAR_BUILDER_LVL1}"

# ....Dry-run build test...........................................................................
n2st::print_formated_script_header "build in dry-run mode testing" "${MSG_LINE_CHAR_BUILDER_LVL1}"

DRYRUN_COMPOSE_FILE_LIST=(
  "docker-compose.project.build.native.yaml"
  "docker-compose.project.build.multiarch.yaml"
)

n2st::print_msg "Will dry-run build the following compose files:"
for idx in "${!DRYRUN_COMPOSE_FILE_LIST[@]}"; do
  echo "              $idx › ${DRYRUN_COMPOSE_FILE_LIST[idx]}"
done

n2st::print_msg "Begin docker compose build --dry-run test"
for each_compose in "${DRYRUN_COMPOSE_FILE_LIST[@]}"; do
  n2st::print_formated_script_header "Test ${MSG_DIMMED_FORMAT}${each_compose}${MSG_END_FORMAT} config" ">"
  ADD_DOCKER_FLAG=()
  ADD_DOCKER_FLAG+=("--file" "${each_compose}")
  ADD_DOCKER_FLAG+=("--dry-run")
  if [[ "${each_compose}" =~ .*".multiarch.yaml" ]]; then
    dnp::build_dn_project_multiarch_services --msg-line-level "${MSG_LINE_CHAR_BUILDER_LVL2}" "${ADD_DOCKER_FLAG[@]}"
  else
    dnp::build_dn_project_services --msg-line-level "${MSG_LINE_CHAR_BUILDER_LVL2}" "${ADD_DOCKER_FLAG[@]}"
  fi
  BUILD_TEST_EXIT_CODE+=("$?")
  n2st::print_formated_script_footer "Test ${MSG_DIMMED_FORMAT}${each_compose}${MSG_END_FORMAT} config" "<"
done

n2st::print_msg "Completed build in dry-run mode tests"

# ....Config and dry-run build test summary........................................................
n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"

n2st::print_msg "Config test summary"
for idx in "${!CONFIG_TEST_EXIT_CODE[@]}"; do
  if [[ ${CONFIG_TEST_EXIT_CODE[idx]} != 0 ]]; then
    echo -e "    ${MSG_ERROR_FORMAT}${CONFIG_TEST_COMPOSE_FILE_LIST[idx]} Completed docker compose config test with error${MSG_END_FORMAT}"
  else
    echo -e "    ${MSG_DONE_FORMAT}${CONFIG_TEST_COMPOSE_FILE_LIST[idx]} Completed docker compose config test${MSG_END_FORMAT}"
  fi
done

n2st::print_msg "Dry-run build test summary"
for idx in "${!BUILD_TEST_EXIT_CODE[@]}"; do
  if [[ ${BUILD_TEST_EXIT_CODE[idx]} != 0 ]]; then
    echo -e "    ${MSG_ERROR_FORMAT}${DRYRUN_COMPOSE_FILE_LIST[idx]} dry-run build completed with error${MSG_END_FORMAT}"
  else
    echo -e "    ${MSG_DONE_FORMAT}${DRYRUN_COMPOSE_FILE_LIST[idx]} dry-run build completed${MSG_END_FORMAT}"
  fi
done

# ====Teardown=====================================================================================
popd >/dev/null || exit 1

# ....Set exit code................................................................................
TEST_EXIT_CODE=("${CONFIG_TEST_EXIT_CODE[@]}" "${BUILD_TEST_EXIT_CODE[@]}")
TEST_EXIT=0
for each_build_exit_code in "${TEST_EXIT_CODE[@]}"; do
  TEST_EXIT=$((TEST_EXIT + each_build_exit_code))
done

if [[ ${TEST_EXIT} != 0 ]]; then
  exit 1
else
  exit 0
fi
