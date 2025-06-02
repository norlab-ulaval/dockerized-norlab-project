#!/bin/bash
# =================================================================================================
# Convenient script for testing config and dry-run build slurm images specified
# in docker-compose.project.build.native.yaml and docker-compose.project.build.multiarch.yaml
#
# Usage:
#   $ bash dryrun_and_config_test.slurm.bash ["<slurm/job/dir/path>"]
#
# Positional argument:
#   <slurm/job/dir/path>     (Optional) The path to the directory containing the slurm job scripts.
#                            Default to ".dockerized_norlab_project/slurm_jobs"
#
# =================================================================================================
if [[ "${DNP_CLEAR_CONSOLE_ACTIVATED}" == "true" ]]; then
  clear
fi

pushd "$(pwd)" >/dev/null || exit 1

# ....Set env variables via positional argument....................................................
SLURM_SCRIPT_JOB_PATH="${*:-".dockerized_norlab_project/slurm_jobs"}"

# ....Source project shell-scripts dependencies....................................................
SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]:-'.'}")"
SCRIPT_PATH_PARENT="$(dirname "${SCRIPT_PATH}")"
source "${SCRIPT_PATH_PARENT}/../utils/import_dnp_lib.bash" || exit 1
source "${SCRIPT_PATH_PARENT}/../utils/load_super_project_config.bash" || exit 1
source "${SCRIPT_PATH_PARENT}/../utils/execute_compose.bash" || exit 1
source "${SCRIPT_PATH_PARENT}/build.all.bash" || exit 1
source "${SCRIPT_PATH_PARENT}/build.all.multiarch.bash" || exit 1

# ....Setup user argument..........................................................................
if [[ ! -d "${SUPER_PROJECT_ROOT:?err}/${SLURM_SCRIPT_JOB_PATH}" ]]; then
  echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} Slurm jobs script directory ${SUPER_PROJECT_ROOT:?err}/${SLURM_SCRIPT_JOB_PATH} is unreachable!" 1>&2
  return 1
fi

# ====Begin========================================================================================
n2st::norlab_splash "${PROJECT_GIT_NAME} (${DNP_PROMPT_NAME})" "${DNP_GIT_REMOTE_URL}"
n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"

declare -a CONFIG_TEST_EXIT_CODE
declare -a BUILD_TEST_EXIT_CODE
declare -a SLURM_JOB_DRYRUN_EXIT_CODE

# ....Config test..................................................................................
n2st::print_msg "Begin config test"

CONFIG_TEST_COMPOSE_FILE_LIST=(
  "docker-compose.project.build.native.yaml"
  "docker-compose.project.build.multiarch.yaml"
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
    ADD_DOCKER_FLAG+=("project-slurm")
    dnp::excute_compose_on_dn_project_image "${ADD_DOCKER_FLAG[@]}"
  elif [[ "${each_compose}" =~ .*".run.slurm.yaml" ]]; then
    dnp::excute_compose_on_dn_project_image "${ADD_DOCKER_FLAG[@]}"
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
  ADD_FCT_FLAG=()
  ADD_FCT_FLAG+=("--service-names" "project-slurm,project-slurm-no-gpu")
  ADD_DOCKER_FLAG=()
  ADD_DOCKER_FLAG+=("--file" "${each_compose}")
  ADD_DOCKER_FLAG+=("--dry-run")
  if [[ "${each_compose}" =~ .*".multiarch.yaml" ]]; then
    dnp::build_dn_project_multiarch_services "${ADD_FCT_FLAG[@]}" --msg-line-level "${MSG_LINE_CHAR_BUILDER_LVL2}" "${ADD_DOCKER_FLAG[@]}"
  else
    dnp::build_dn_project_services "${ADD_FCT_FLAG[@]}" --msg-line-level "${MSG_LINE_CHAR_BUILDER_LVL2}" "${ADD_DOCKER_FLAG[@]}"
  fi
  BUILD_TEST_EXIT_CODE+=("$?")
  n2st::print_formated_script_footer "Test ${MSG_DIMMED_FORMAT}${each_compose}${MSG_END_FORMAT} config" "<"
done

n2st::print_msg "Completed build in dry-run mode tests"

# ....Dry-run SLURM/Mamba jobs.....................................................................
n2st::print_formated_script_header "Dry-run slurm job" "${MSG_LINE_CHAR_BUILDER_LVL1}"
pushd "$(pwd)" >/dev/null || exit 1

SLURM_JOB_FILE_NAME=()
for each_file_path in "${SUPER_PROJECT_ROOT:?err}"/"${SLURM_SCRIPT_JOB_PATH}"/slurm_job.*.bash ; do
  each_file_name="$(basename $each_file_path)"
  SLURM_JOB_FILE_NAME+=("$each_file_name")
#  if [[ "${each_file_name}" != "slurm_job.dryrun.bash" ]] && [[ "${each_file_name}" != "slurm_job.template.bash" ]]; then
#    SLURM_JOB_FILE_NAME+=("$each_file_name")
#  fi
done

n2st::print_msg "Will dry-run the following slurm job files:"
for idx in "${!SLURM_JOB_FILE_NAME[@]}"; do
  echo "              $idx › ${SLURM_JOB_FILE_NAME[idx]}"
done

n2st::print_msg "Begin dry-run slurm job"
SLURM_JOB_FLAGS=()
SLURM_JOB_FLAGS+=("--skip-core-force-rebuild")
SLURM_JOB_FLAGS+=("--hydra-dry-run")
cd "${SUPER_PROJECT_ROOT:?err}/${SLURM_SCRIPT_JOB_PATH}" || exit 1
for each_slurm_job in "${SLURM_JOB_FILE_NAME[@]}" ; do
  n2st::print_formated_script_header "$each_slurm_job" "${MSG_LINE_CHAR_BUILDER_LVL2}"
  bash "${each_slurm_job}" "${SLURM_JOB_FLAGS[@]}"
  SLURM_JOB_DRYRUN_EXIT_CODE+=("$?")
  n2st::print_formated_script_footer "$each_slurm_job" "${MSG_LINE_CHAR_BUILDER_LVL2}"
done

popd >/dev/null || exit 1
n2st::print_msg "Completed slurm job dry-run tests"

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

n2st::print_msg "Dry-run slurm job summary"
for idx in "${!SLURM_JOB_DRYRUN_EXIT_CODE[@]}"; do
  if [[ ${SLURM_JOB_DRYRUN_EXIT_CODE[idx]} != 0 ]]; then
    echo -e "    ${MSG_ERROR_FORMAT}${SLURM_JOB_FILE_NAME[idx]} dry-run slurm job completed with error${MSG_END_FORMAT}"
  else
    echo -e "    ${MSG_DONE_FORMAT}${SLURM_JOB_FILE_NAME[idx]} dry-run slurm job completed${MSG_END_FORMAT}"
  fi
done

# ====Teardown=====================================================================================
popd >/dev/null || exit 1

# ....Set exit code................................................................................
TEST_EXIT_CODE=("${CONFIG_TEST_EXIT_CODE[@]}" "${BUILD_TEST_EXIT_CODE[@]}" "${SLURM_JOB_DRYRUN_EXIT_CODE[@]}")
TEST_EXIT=0
for each_build_exit_code in "${TEST_EXIT_CODE[@]}"; do
  TEST_EXIT=$((TEST_EXIT + each_build_exit_code))
done

if [[ ${TEST_EXIT} != 0 ]]; then
  exit 1
else
  exit 0
fi
