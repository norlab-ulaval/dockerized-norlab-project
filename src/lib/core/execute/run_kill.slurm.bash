#!/bin/bash
# =================================================================================================
# Kill docker compose ressources alocated by run.slurm.bash
#
# Usage:
#   $ bash run_kill.slurm.bash
#
# =================================================================================================
clear
pushd "$(pwd)" >/dev/null || exit 1

# ....Source project shell-scripts dependencies..................................................
SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]:-'.'}")"
SCRIPT_PATH_PARENT="$(dirname "${SCRIPT_PATH}")"
source "${SCRIPT_PATH_PARENT}/../utils/import_dnp_lib.bash" || exit 1
source "${SCRIPT_PATH_PARENT}/../utils/load_super_project_config.bash" || exit 1

# ====Begin========================================================================================
cd "${SUPER_PROJECT_ROOT:?err}" || exit 1

n2st::norlab_splash "${PROJECT_GIT_NAME} (${DNP_PROMPT_NAME})" "${DNP_GIT_REMOTE_URL}"
n2st::print_formated_script_header "$(basename $0) ${MSG_END_FORMAT}on device ${MSG_DIMMED_FORMAT}$(hostname -s)" "${MSG_LINE_CHAR_BUILDER_LVL2}"

n2st::set_which_architecture_and_os
n2st::print_msg "Current image architecture and os: $IMAGE_ARCH_AND_OS"

n2st::set_is_teamcity_run_environment_variable

# ....Device specific config.......................................................................
COMPOSE_PATH=".dockerized_norlab_project/configuration"
THE_COMPOSE_FILE=docker-compose.project.run.slurm.yaml

CONTAINER_ID=$(docker compose -f "${COMPOSE_PATH}/${THE_COMPOSE_FILE}" ps --quiet --all --orphans=false)
echo
echo "Compose project project-slurm running service(s) CONTAINER_ID:"
echo "${CONTAINER_ID[*]}"
echo
echo "Stop the following running container(s):"
# shellcheck disable=SC2068
docker stop ${CONTAINER_ID[@]} 2>/dev/null


# ====Teardown=====================================================================================
n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
popd >/dev/null || exit 1
