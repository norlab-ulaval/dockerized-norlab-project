#!/bin/bash
# =================================================================================================
# Execute docker compose down using the compose file matching the host
#
# Usage:
#   $ bash down.bash
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

if [[ ${IMAGE_ARCH_AND_OS:?err} == 'l4t/arm64' ]]; then
  THE_COMPOSE_FILE=docker-compose.project.run.jetson.yaml
elif [[ $IMAGE_ARCH_AND_OS == 'darwin/arm64' ]]; then
  THE_COMPOSE_FILE=docker-compose.project.run.darwin.yaml
elif [[ $IMAGE_ARCH_AND_OS == 'linux/x86' ]]; then
  THE_COMPOSE_FILE=docker-compose.project.run.linux-x86.yaml
else
  n2st::print_msg_error_and_exit "Support for current host not implemented yet!"
fi

docker compose -f "${COMPOSE_PATH}/${THE_COMPOSE_FILE}" down

# ====Teardown=====================================================================================
n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
popd >/dev/null || exit 1
